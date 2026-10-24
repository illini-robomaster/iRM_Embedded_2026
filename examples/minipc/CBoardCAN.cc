/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2026 RoboMaster.                                          *
 *  Illini RoboMaster @ University of Illinois at Urbana-Champaign          *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                          *
 ****************************************************************************/

// Board-state CAN link example for the DJI Type C board.
//
// The Jetson initiates a session on 0x320.  The board answers on 0x321 with its
// readiness and protocol color, starts a 100 Hz stream on 0x322 after the final
// ACK, and keeps the gimbal control loop running independently of the link.
// Pair it with the io::BoardStateCan receiver in iRM_Vision_2026.
//
// The default target shares CAN1 with the two gimbal motors; the CAN2 target
// puts only the Jetson link on hcan2.  Both use non-blocking TryTransmit() and
// one-fragment-at-a-time scheduling so a busy bus cannot stall the gimbal hold.

#include "main.h"

#include <cstring>

#include "board_state_can_protocol.h"
#include "bsp_can.h"
#include "bsp_imu.h"
#include "cmsis_os.h"
#include "gimbal.h"
#include "i2c.h"
#include "minipc_protocol.h"
#include "utils.h"

#define RX_SIGNAL (1 << 0)

#ifndef BOARD_STATE_CAN_USE_CAN2
#define BOARD_STATE_CAN_USE_CAN2 0
#endif

// Real robots take this from the referee (protocol 0x0207,
// referee->shoot_data.bullet_speed); fixed here so the example has no referee.
constexpr float BULLET_SPEED = 15.0f;

// EXAMPLE ONLY: replace with the robot's actual protocol color.  The color is
// fixed for a session; changing it requires a new session on the Jetson side.
constexpr communication::board_state_can::Color LINK_COLOR = communication::board_state_can::Color::BLUE;

namespace {

/* ========================= CAN RX ring ========================= */

// Handshake frames are tiny and infrequent (START/final ACK), so eight slots are
// plenty.  The ISR only copies into this ring; parsing happens in the comm task.
constexpr uint32_t RX_RING_CAPACITY = 8;
static_assert((RX_RING_CAPACITY & (RX_RING_CAPACITY - 1)) == 0, "ring capacity must be a power of two");

struct RxFrame {
  uint8_t data[8];
  uint32_t dlc;
};

class RxRing {
 public:
  // Single producer (CAN ISR).
  bool PushFromIsr(const uint8_t data[], uint32_t dlc) {
    const uint32_t head = head_;
    const uint32_t next = (head + 1) & (RX_RING_CAPACITY - 1);
    if (next == tail_) return false;  // full: drop the newest frame
    entries_[head].dlc = dlc;
    for (uint32_t i = 0; i < 8; ++i) entries_[head].data[i] = data[i];
    __DMB();
    head_ = next;
    return true;
  }

  // Single consumer (comm task).
  bool Pop(RxFrame* frame) {
    const uint32_t tail = tail_;
    if (tail == head_) return false;
    __DMB();  // acquire: the ISR's payload writes precede its head_ publication
    *frame = entries_[tail];
    __DMB();
    tail_ = (tail + 1) & (RX_RING_CAPACITY - 1);
    return true;
  }

 private:
  RxFrame entries_[RX_RING_CAPACITY] = {};
  volatile uint32_t head_ = 0;
  volatile uint32_t tail_ = 0;
};

RxRing rx_ring;
volatile uint32_t board_state_can_rx_dropped = 0;

void HandshakeRxCallback(const uint8_t data[], uint32_t dlc, void* args) {
  if (!static_cast<RxRing*>(args)->PushFromIsr(data, dlc)) ++board_state_can_rx_dropped;
}

}  // namespace

/* ========================= IMU ========================= */

const osThreadAttr_t imuTaskAttribute = {.name = "imuTask",
                                         .attr_bits = osThreadDetached,
                                         .cb_mem = nullptr,
                                         .cb_size = 0,
                                         .stack_mem = nullptr,
                                         .stack_size = 256 * 4,
                                         .priority = (osPriority_t)osPriorityNormal,
                                         .tz_module = 0,
                                         .reserved = 0};
osThreadId_t imuTaskHandle;

class IMU : public bsp::IMU_typeC {
 public:
  using bsp::IMU_typeC::IMU_typeC;

 protected:
  void RxCompleteCallback() final { osThreadFlagsSet(imuTaskHandle, RX_SIGNAL); }
};

static IMU* imu = nullptr;

/* ========================= comm ========================= */

const osThreadAttr_t commTaskAttribute = {.name = "commTask",
                                          .attr_bits = osThreadDetached,
                                          .cb_mem = nullptr,
                                          .cb_size = 0,
                                          .stack_mem = nullptr,
                                          .stack_size = 256 * 4,
                                          .priority = (osPriority_t)osPriorityHigh,
                                          .tz_module = 0,
                                          .reserved = 0};
osThreadId_t commTaskHandle;

/* ========================= gimbal ========================= */

static bsp::CAN* motor_can = nullptr;
static bsp::CAN* link_can = nullptr;
static control::MotorCANBase* pitch_motor = nullptr;
static control::MotorCANBase* yaw_motor = nullptr;
static control::Gimbal* gimbal = nullptr;

// Debug-visible health counters: bus-off/backpressure must not be silent just
// because this example deliberately does not print over the minipc UART.
volatile uint32_t board_state_can_tx_busy = 0;
volatile uint32_t board_state_can_tx_error = 0;

static bool QueueCanFrame(uint16_t id, const uint8_t data[], uint32_t length) {
  const bsp::CAN::TxResult result = link_can->TryTransmit(id, data, length);
  if (result == bsp::CAN::TxResult::BUSY) ++board_state_can_tx_busy;
  if (result == bsp::CAN::TxResult::ERROR) ++board_state_can_tx_error;
  return result == bsp::CAN::TxResult::QUEUED;
}

/* Sends the same 29-byte image as the minipc UART link. */
static void FillBoardState(communication::board_state_t* state) {
  // Keep the IMU task and CAN feedback ISR from changing sources halfway
  // through one snapshot.  Only a handful of scalar reads occur in this section.
  taskENTER_CRITICAL();
  state->q[0] = imu->INS_quat[0];
  state->q[1] = imu->INS_quat[1];
  state->q[2] = imu->INS_quat[2];
  state->q[3] = imu->INS_quat[3];

  // Measured (not target) motor angles, expressed in the same frame as the
  // angles the Jetson commands.  The offset is read here because calibration
  // completes after the comm task has already started handshaking.
  const control::gimbal_data_t* gimbal_data = gimbal->GetData();
  state->yaw = wrap<float>(yaw_motor->GetTheta() - gimbal_data->yaw_offset_, -PI, PI);
  state->pitch = wrap<float>(pitch_motor->GetTheta() - gimbal_data->pitch_offset_, -PI, PI);

  state->bullet_speed = BULLET_SPEED;
  state->mode[0] = 1;  // opaque to the protocol, e.g. task mode
  taskEXIT_CRITICAL();
}

#ifdef DEBUG
// Focused pure-codec checks against the examples in the protocol document.
static void ValidateCodec() {
  using namespace communication::board_state_can;

  uint8_t frame[FRAGMENT_DLC];

  // ACK+color for session 0x12345678 and blue.
  EncodeAck(frame, 0x12345678, Status::READY, Color::BLUE);
  const uint8_t expected_ack[HANDSHAKE_DLC] = {0x02, 0x01, 0x12, 0x34, 0x56, 0x78, 0x00, 0x02};
  RM_ASSERT_TRUE(memcmp(frame, expected_ack, HANDSHAKE_DLC) == 0, "board-state CAN ACK encoding mismatch");

  // Fragment metadata and the five-byte last fragment with zero padding.
  communication::board_state_t state = {};
  state.bullet_speed = BULLET_SPEED;
  state.mode[0] = 1;
  RM_ASSERT_TRUE(EncodeFragment(frame, state, 0x78, 7, 0), "fragment 0 encoding failed");
  RM_ASSERT_EQ(frame[0], 0x07, "fragment 0 metadata mismatch");
  RM_ASSERT_EQ(frame[1], 0x78, "fragment session tag mismatch");
  RM_ASSERT_TRUE(EncodeFragment(frame, state, 0x78, 7, 4), "fragment 4 encoding failed");
  RM_ASSERT_EQ(frame[0], 0x87, "fragment 4 metadata mismatch");
  RM_ASSERT_EQ(frame[2], 0x00, "bullet_speed byte 0 mismatch");  // 15.0f = 00 00 70 41
  RM_ASSERT_EQ(frame[3], 0x00, "bullet_speed byte 1 mismatch");
  RM_ASSERT_EQ(frame[4], 0x70, "bullet_speed byte 2 mismatch");
  RM_ASSERT_EQ(frame[5], 0x41, "bullet_speed byte 3 mismatch");
  RM_ASSERT_EQ(frame[6], 0x01, "mode byte mismatch");
  RM_ASSERT_EQ(frame[7], 0x00, "fragment 4 padding must be zero");

  // START/final ACK use big-endian integers, and padding is mandatory.
  StartMessage start;
  const uint8_t start_frame[HANDSHAKE_DLC] = {0x01, 0x01, 0x12, 0x34, 0x56, 0x78, 0x00, 0x00};
  RM_ASSERT_TRUE(DecodeStart(start_frame, HANDSHAKE_DLC, &start), "valid START rejected");
  RM_ASSERT_EQ(start.version, PROTOCOL_VERSION, "START version mismatch");
  RM_ASSERT_EQ(start.session_id, 0x12345678u, "START session decoding mismatch");

  const uint8_t bad_start[HANDSHAKE_DLC] = {0x01, 0x01, 0x12, 0x34, 0x56, 0x78, 0x00, 0x01};
  RM_ASSERT_FALSE(DecodeStart(bad_start, HANDSHAKE_DLC, &start), "START with nonzero padding accepted");

  FinalAckMessage final_ack;
  const uint8_t final_ack_frame[HANDSHAKE_DLC] = {0x03, 0x01, 0x12, 0x34, 0x56, 0x78, 0x00, 0x00};
  RM_ASSERT_TRUE(DecodeFinalAck(final_ack_frame, HANDSHAKE_DLC, &final_ack), "valid final ACK rejected");
  RM_ASSERT_EQ(final_ack.session_id, 0x12345678u, "final ACK session decoding mismatch");
}
#endif

void RM_RTOS_Init() {
#ifdef DEBUG
  ValidateCodec();
#endif

  // CAN1 always starts first: CAN2 is the slave instance on STM32F4.
  motor_can = new bsp::CAN(&hcan1, true);
#if BOARD_STATE_CAN_USE_CAN2
  link_can = new bsp::CAN(&hcan2, false);
#else
  link_can = motor_can;
#endif
  pitch_motor = new control::Motor6020(motor_can, 0x205);
  yaw_motor = new control::Motor6020(motor_can, 0x206);

  control::gimbal_t gimbal_init;
  gimbal_init.pitch_motor = pitch_motor;
  gimbal_init.yaw_motor = yaw_motor;
  gimbal_init.model = control::GIMBAL_SENTRY;
  gimbal = new control::Gimbal(gimbal_init);

  bsp::IST8310_init_t IST8310_init;
  IST8310_init.hi2c = &hi2c3;
  IST8310_init.int_pin = DRDY_IST8310_Pin;
  IST8310_init.rst_group = GPIOG;
  IST8310_init.rst_pin = GPIO_PIN_6;

  bsp::BMI088_init_t BMI088_init;
  BMI088_init.hspi = &hspi1;
  BMI088_init.CS_ACCEL_Port = CS1_ACCEL_GPIO_Port;
  BMI088_init.CS_ACCEL_Pin = CS1_ACCEL_Pin;
  BMI088_init.CS_GYRO_Port = CS1_GYRO_GPIO_Port;
  BMI088_init.CS_GYRO_Pin = CS1_GYRO_Pin;

  bsp::heater_init_t heater_init;
  heater_init.htim = &htim10;
  heater_init.channel = 1;
  heater_init.clock_freq = 1000000;
  heater_init.temp = 45;

  bsp::IMU_typeC_init_t imu_init;
  imu_init.IST8310 = IST8310_init;
  imu_init.BMI088 = BMI088_init;
  imu_init.heater = heater_init;
  imu_init.hspi = &hspi1;
  imu_init.hdma_spi_rx = &hdma_spi1_rx;
  imu_init.hdma_spi_tx = &hdma_spi1_tx;
  imu_init.Accel_INT_pin_ = INT1_ACCEL_Pin;
  imu_init.Gyro_INT_pin_ = INT1_GYRO_Pin;

  imu = new IMU(imu_init, false);

  // The motors register their own feedback ids on this same CAN line.  The link
  // registers only 0x320 and uses the DLC-aware callback so malformed frames can
  // be rejected before they reach the session.
  RM_ASSERT_EQ(link_can->RegisterUniqueDlcRxCallback(communication::board_state_can::JETSON_TO_BOARD_ID, HandshakeRxCallback, &rx_ring), 0,
               "Cannot register board-state CAN handshake callback");
}

/* IMU update task: the SPI completion callback wakes us. */
void imuTask(void* arg) {
  UNUSED(arg);

  while (true) {
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) imu->Update();
  }
}

/* Handshake and 100 Hz board-state stream. */
void commTask(void* arg) {
  UNUSED(arg);

  communication::board_state_can::BoardStateCanServer server(LINK_COLOR);
  communication::board_state_t state = {};
  uint8_t frame[communication::board_state_can::FRAGMENT_DLC];
  RxFrame received;

  while (true) {
    const uint32_t now_ms = HAL_GetTick();
    server.SetReady(imu->DataReady() && imu->CaliDone());

    while (rx_ring.Pop(&received)) {
      server.HandleFrame(received.data, received.dlc, now_ms);
    }

    // Handshake responses take priority and are never blocked by telemetry.
    // The 100 ms repeat is driven by the session's due time.
    if (server.IsResponseDue(now_ms)) {
      server.EncodeResponse(frame);
      if (QueueCanFrame(communication::board_state_can::BOARD_TO_JETSON_ACK_ID, frame,
                        communication::board_state_can::HANDSHAKE_DLC)) {
        server.OnResponseQueued(now_ms);
      }
    } else if (server.IsStreaming()) {
      // Capture one snapshot, then queue its five fragments one at a time.  On
      // BUSY the same fragment is retried on the next 1 ms pass.
      if (!server.HasFragment()) {
        FillBoardState(&state);
        server.StartSample(state, now_ms);
      }
      if (server.HasFragment() && server.EncodeFragment(frame)) {
        if (QueueCanFrame(communication::board_state_can::BOARD_TO_JETSON_DATA_ID, frame,
                          communication::board_state_can::FRAGMENT_DLC)) {
          server.OnFragmentQueued();
        }
      }
    }

    osDelay(1);
  }
}

void RM_RTOS_Threads_Init() {
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
  commTaskHandle = osThreadNew(commTask, nullptr, &commTaskAttribute);
}

/* 1 kHz gimbal hold.  It never inspects the CAN link state. */
void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  control::MotorCANBase* motors[2] = {pitch_motor, yaw_motor};

  while (!imu->DataReady()) osDelay(10);
  imu->Calibrate();
  while (!imu->CaliDone()) osDelay(10);

  while (true) {
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors, 2);
    osDelay(1);
  }
}
