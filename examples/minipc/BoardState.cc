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

// Sends the STM32 -> Jetson board_state packet once per cycle: IMU attitude,
// measured gimbal motor angles, bullet speed, and an opaque mode byte vector.
// Pair it with io::Minipc in iRM_Vision_2026 (branch ywh114/hw_vision2026).
//
// Note: huart1 is the minipc link here, so this example deliberately does not
// call print_use_uart on it.

#include "main.h"

#include <memory>

#include "bsp_imu.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "gimbal.h"
#include "i2c.h"
#include "minipc_protocol.h"
#include "utils.h"

#define RX_SIGNAL (1 << 0)

// Real robots take this from the referee (protocol 0x0207,
// referee->shoot_data.bullet_speed); fixed here so the example has no referee.
#define BULLET_SPEED 15.0f

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
                                          .stack_size = 128 * 4,
                                          .priority = (osPriority_t)osPriorityHigh,
                                          .tz_module = 0,
                                          .reserved = 0};
osThreadId_t commTaskHandle;

/* ========================= gimbal ========================= */

static bsp::CAN* can1 = nullptr;
static control::MotorCANBase* pitch_motor = nullptr;
static control::MotorCANBase* yaw_motor = nullptr;
static control::Gimbal* gimbal = nullptr;

void RM_RTOS_Init() {
  can1 = new bsp::CAN(&hcan1, true);
  pitch_motor = new control::Motor6020(can1, 0x205);
  yaw_motor = new control::Motor6020(can1, 0x206);

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
}

/* IMU update task: the SPI completion callback wakes us. */
void imuTask(void* arg) {
  UNUSED(arg);

  while (true) {
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) imu->Update();
  }
}

/* Packs and sends the board state at 100 Hz. */
void commTask(void* arg) {
  UNUSED(arg);

  auto uart = std::make_unique<bsp::UART>(&huart1);  // see cmake for which uart
  uart->SetupTx(50);

  auto minipc_session = communication::MinipcPort();
  communication::board_state_t state;
  uint8_t packet_to_send[minipc_session.MAX_PACKET_LENGTH];

  // Calibration runs in the default task; wait for it before publishing.
  while (!imu->DataReady() || !imu->CaliDone()) osDelay(10);

  // Per-robot calibration that only exists on this side, so the measured motor
  // angles are converted here rather than on the Jetson.
  const float yaw_offset = gimbal->GetData()->yaw_offset_;
  const float pitch_offset = gimbal->GetData()->pitch_offset_;

  while (true) {
    state.q[0] = imu->INS_quat[0];
    state.q[1] = imu->INS_quat[1];
    state.q[2] = imu->INS_quat[2];
    state.q[3] = imu->INS_quat[3];

    // Measured (not target) motor angles, expressed in the same frame as the
    // angles the Jetson commands.
    state.yaw = wrap<float>(yaw_motor->GetTheta() - yaw_offset, -PI, PI);
    state.pitch = wrap<float>(pitch_motor->GetTheta() - pitch_offset, -PI, PI);

    state.bullet_speed = BULLET_SPEED;

    // Opaque to the protocol. The Jetson decides what the byte means; this
    // example just fills it with a plausible flag.
    state.mode[0] = 1;  // e.g. task mode

    minipc_session.Pack(packet_to_send, (void*)&state, communication::BOARD_STATE_CMD_ID);
    uart->Write(packet_to_send, minipc_session.GetPacketLen(communication::BOARD_STATE_CMD_ID));

    osDelay(10);  // 100 Hz
  }
}

void RM_RTOS_Threads_Init() {
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
  commTaskHandle = osThreadNew(commTask, nullptr, &commTaskAttribute);
}

/* Gimbal control loop. It holds position; minipc only publishes state. */
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
