/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2025 RoboMaster.                                          *
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
 *  along with this program. If not, see <http://www.gnu.org/licenses/\>.    *
 *                                                                          *
 ****************************************************************************/

/**
 * @brief Omni robot control for DM_MC_02 board
 *
 * Hardware differences from TypeC (main.cc):
 * - Uses FDCAN instead of CAN (hfdcan1)
 * - Print via UART7 (huart7, ST-Link VCP)
 * - DBUS via UART5 (huart5)
 * - IMU via SPI2 (hspi2) with BMI088 only (no IST8310/heater)
 * - IMU initialized in imuTask (HAL_Delay in BMI088 ctor needs RTOS running)
 */

#include <cmath>
#include <cstring>
#include <memory>
#include <type_traits>

#include "MahonyAHRS.h"
#include "board_state_can_protocol.h"
#include "vision_can_protocol.h"
#include "bsp_imu.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "main.h"
#include "mouse_gimbal_step.h"
#include "motor.h"
#include "protocol.h"
#include "spi.h"
#include "usart.h"

#define IMU_RX_SIGNAL (1 << 0)
#define REFEREE_RX_SIGNAL (1 << 1)

// Convert radians to degrees and vice versa
#define RAD2DEG(x) ((x) * 180.0f / M_PI)
#define DEG2RAD(x) ((x) * M_PI / 180.0f)

// Mouse -> gimbal scaling, per-iteration clamp and the "apply or drop" policy live
// in mouse_gimbal_step.h so they can be regression tested on the host.

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

// IMU data
static bsp::BMI088* bmi088 = nullptr;
static float gyro[3];
static float accel[3];
static float temp;
static float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float INS_angle[3] = {0.0f, 0.0f, 0.0f};  // yaw, pitch, roll
static volatile uint32_t imu_update_ms = 0;
static volatile bool imu_ready = false;

/**
 * @brief IMU Gyroscope interrupt handler class
 */
class IMU_GYRO_INT : public bsp::GPIT {
 public:
  IMU_GYRO_INT() : GPIT(INT1_GYRO_Pin) {}

 protected:
  void IntCallback() override {
    osThreadFlagsSet(imuTaskHandle, IMU_RX_SIGNAL);
  }
};

/**
 * @brief IMU Accelerometer interrupt handler class
 */
class IMU_ACCEL_INT : public bsp::GPIT {
 public:
  IMU_ACCEL_INT() : GPIT(INT1_ACCEL_Pin) {}

 protected:
  void IntCallback() override {
    // Accelerometer data ready - optional handling
  }
};

static IMU_GYRO_INT* gyro_int = nullptr;
static IMU_ACCEL_INT* accel_int = nullptr;

void imuTask(void* arg) {
  UNUSED(arg);

  // Initialize BMI088 here (after RTOS scheduler starts) so HAL_Delay() works
  bmi088 = new bsp::BMI088(&hspi2,
                           CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin,
                           CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);

  // Create interrupt handlers after BMI088 is ready
  gyro_int = new IMU_GYRO_INT();
  accel_int = new IMU_ACCEL_INT();

  while (true) {
    uint32_t flags = osThreadFlagsWait(IMU_RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & IMU_RX_SIGNAL) {
      bmi088->Read(gyro, accel, &temp);

      // BMI088_GYRO_SEN already converts raw gyro readings to rad/s.
      float gx_rad = gyro[0];
      float gy_rad = gyro[1];
      float gz_rad = gyro[2];

      MahonyAHRSupdateIMU(quat, gx_rad, gy_rad, gz_rad,
                          accel[0], accel[1], accel[2]);

      INS_angle[0] = atan2f(2.0f * (quat[0] * quat[3] + quat[1] * quat[2]),
                            1.0f - 2.0f * (quat[2] * quat[2] + quat[3] * quat[3]));  // yaw
      INS_angle[1] = asinf(2.0f * (quat[0] * quat[2] - quat[3] * quat[1]));          // pitch
      INS_angle[2] = atan2f(2.0f * (quat[0] * quat[1] + quat[2] * quat[3]),
                            1.0f - 2.0f * (quat[1] * quat[1] + quat[2] * quat[2]));  // roll
      imu_ready = true;
      imu_update_ms = HAL_GetTick();
    }
  }
}

/* ========================================================================= */
/*                          Referee System UART Task                         */
/* ========================================================================= */

const osThreadAttr_t refereeTaskAttribute = {.name = "refereeTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 512 * 4,
                                             .priority = (osPriority_t)osPriorityNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
osThreadId_t refereeTaskHandle;

class RefereeUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  void RxCompleteCallback() final { osThreadFlagsSet(refereeTaskHandle, REFEREE_RX_SIGNAL); }
};

static communication::Referee* referee = nullptr;
static RefereeUART* referee_uart = nullptr;

static uint8_t prev_pmm_gimbal = 0xFF;
static uint8_t prev_pmm_chassis = 0xFF;
static uint8_t prev_pmm_shooter = 0xFF;
// RM2027 0x0202 no longer carries chassis power (those bytes are reserved and read 0),
// only the buffer energy. The buffer drains whenever the chassis draws more than the
// referee's limit, so it is what the power governor regulates.
static volatile uint16_t chassis_buffer_j = 0;
static volatile uint32_t chassis_buffer_rx_ms = 0;
static volatile bool chassis_buffer_received = false;

static void OnRefereeRx(int cmd_id, const uint8_t* data, int length) {
  switch (cmd_id) {
    case communication::GAME_STATUS: {
      const auto* s = reinterpret_cast<const communication::game_status_t*>(data);
      print("[REF RX 0x0001] GAME_STATUS type=%u prog=%u remain=%us\r\n",
            s->game_type, s->game_progress, s->stage_remain_time);
      break;
    }
    case communication::GAME_RESULT: {
      const auto* s = reinterpret_cast<const communication::game_result_t*>(data);
      print("[REF RX 0x0002] GAME_RESULT winner=%u\r\n", s->winner);
      break;
    }
    case communication::GAME_ROBOT_HP: {
      const auto* s = reinterpret_cast<const communication::game_robot_HP_t*>(data);
      print("[REF RX 0x0003] ROBOT_HP ally1=%u ally3=%u ally4=%u ally7=%u base=%u\r\n",
            s->ally_1_robot_HP, s->ally_3_robot_HP, s->ally_4_robot_HP, s->ally_7_robot_HP, s->ally_base_HP);
      break;
    }
    case communication::REFEREE_WARNING: {
      const auto* s = reinterpret_cast<const communication::referee_warning_t*>(data);
      print("[REF RX 0x0104] WARNING lvl=%u robot=%u cnt=%u\r\n",
            s->level, s->offending_robot_id, s->count);
      break;
    }
    case communication::GAME_ROBOT_STATUS: {
      const auto* s = reinterpret_cast<const communication::game_robot_status_t*>(data);
      if (s->power_management_gimbal_output != prev_pmm_gimbal ||
          s->power_management_chassis_output != prev_pmm_chassis ||
          s->power_management_shooter_output != prev_pmm_shooter) {
        prev_pmm_gimbal = s->power_management_gimbal_output;
        prev_pmm_chassis = s->power_management_chassis_output;
        prev_pmm_shooter = s->power_management_shooter_output;
        print(">>> [PMM STATE CHANGE] Gimbal:%u Chassis:%u Shooter:%u | HP:%u/%u (len=%d) <<<\r\n",
              s->power_management_gimbal_output,
              s->power_management_chassis_output,
              s->power_management_shooter_output,
              s->current_HP, s->maximum_HP, length);
      }
      break;
    }
    case communication::POWER_HEAT_DATA: {
      if (length != 14 && length != 16) break;
      const auto* s = reinterpret_cast<const communication::power_heat_data_t*>(data);
      chassis_buffer_j = s->buffer_energy;
      chassis_buffer_rx_ms = HAL_GetTick();
      chassis_buffer_received = true;
      break;
    }
    case communication::ROBOT_HURT: {
      const auto* s = reinterpret_cast<const communication::robot_hurt_t*>(data);
      print("[REF RX 0x0206] HURT armor=%u reason=%u seq=%u pos=(%d,%d)\r\n",
            s->armor_id, s->HP_deduction_reason, s->hit_sequence, s->hit_position_x, s->hit_position_y);
      break;
    }
    case communication::SHOOT_DATA: {
      const auto* s = reinterpret_cast<const communication::shoot_data_t*>(data);
      print("[REF RX 0x0207] SHOOT type=%u num=%u freq=%u spd=%.2fm/s\r\n",
            s->bullet_type, s->shooter_number, s->launching_frequency, s->initial_speed);
      break;
    }
    case communication::BULLET_REMAINING: {
      const auto* s = reinterpret_cast<const communication::projectile_allowance_t*>(data);
      print("[REF RX 0x0208] ALLOWANCE 17mm=%u 42mm=%u coin=%u fort=%u\r\n",
            s->allowance_17mm, s->allowance_42mm, s->remaining_gold_coin, s->allowance_fortress);
      break;
    }
    default:
      // High-rate packets (position, buffs, RFID, ...) are not logged: they would
      // saturate the shared debug UART.
      break;
  }
}

void refereeTask(void* arg) {
  UNUSED(arg);
  uint8_t* rx_data = nullptr;

  while (true) {
    uint32_t flags = osThreadFlagsWait(REFEREE_RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & REFEREE_RX_SIGNAL) {
      int32_t length = referee_uart->Read(&rx_data);
      if (length > 0 && rx_data != nullptr && referee != nullptr) {
        if (!referee->connection_flag_) {
          print("[REF UART1] Raw rx %ld bytes\r\n", (long)length);
        }
        referee->Receive(communication::package_t{rx_data, (int)length});
      }
    }
  }
}

void yawTask(void* arg);
const osThreadAttr_t yawTaskAttribute = {.name = "yawTask",
                                       .attr_bits = osThreadDetached,
                                       .cb_mem = nullptr,
                                       .cb_size = 0,
                                       .stack_mem = nullptr,
                                       .stack_size = 512 * 4,
                                       .priority = (osPriority_t)osPriorityAboveNormal,
                                       .tz_module = 0,
                                       .reserved = 0};

void RM_RTOS_Threads_Init(void) {
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
  RM_ASSERT_NE(osThreadNew(yawTask, nullptr, &yawTaskAttribute), nullptr,
               "Cannot create yaw task");
}

// Global peripherals
bsp::CAN* can = nullptr;
control::Motor3508* chassis_motor[4];
// Rotor speed / wheel speed, measured 2026-11-01 by turning each rotor an exact
// number of encoder turns and checking the wheel mark (chassis_turn_count):
// IDs 1/3/4 (DM3519 x2 and the M3508) are 268:17; ID2's DM3519 is ~7.80:1.
constexpr float CHASSIS_FEEDBACK_TO_WHEEL_RATIO[4] = {268.0f / 17.0f, 7.80f, 268.0f / 17.0f,
                                                      268.0f / 17.0f};
// Array order: ID3, ID2, ID4, ID1. Reverse ID4 in both feedback and command.
constexpr float CHASSIS_MOTOR_DIRECTION[4] = {1.0f, 1.0f, -1.0f, 1.0f};
// Wheel-frame load feedforward: sign(w) * static + viscous * w, in current counts.
// 80% of the free-air fit from chassis_wheel_test with the measured ratios
// (2026-11-01); the PI trims the rest.
constexpr float CHASSIS_FF_STATIC[4] = {282.0f, 616.0f, 553.0f, 242.0f};
constexpr float CHASSIS_FF_VISCOUS[4] = {12.6f, 13.5f, 11.6f, 10.7f};
constexpr float CHASSIS_FF_STATIC_BAND = 1.0f;  // rad/s over which static fades in
constexpr float CHASSIS_OUTPUT_LIMIT = 8000.0f;

// Wheel speed PI. The integral only runs once the command ramp has settled, so the
// lag during acceleration or reversal does not wind it up into an overshoot.
// Ki is per second so loop-time jitter does not change the integral rate.
struct ChassisPI {
  float kp = 200.0f;
  float ki = 600.0f;
  float integral = 0.0f;
  void Reset() { integral = 0.0f; }
  float Compute(float error, bool integrate, float dt) {
    if (integrate) integral = clip<float>(integral + ki * error * dt, -4000.0f, 4000.0f);
    return clip<float>(kp * error + integral, -6000.0f, 6000.0f);
  }
};

static float ChassisFeedforward(int i, float wheel_setpoint) {
  const float static_scale =
      clip<float>(wheel_setpoint / CHASSIS_FF_STATIC_BAND, -1.0f, 1.0f);
  return CHASSIS_FF_STATIC[i] * static_scale + CHASSIS_FF_VISCOUS[i] * wheel_setpoint;
}
control::Motor4310* gimbal_motor;
control::Motor6020* yaw_motor;
control::Motor3508* flywheel_motor[2];
control::Motor3508* feeder_motor;

remote::DBUS* dbus = nullptr;

const float YAW_MOTOR_OFFSET = 2.417f;  // provisional forward pose: old offset + measured 94.8 deg
// GM6020 voltage-mode back-EMF is ~750-900 counts per rad/s (13.33 rpm/V, 24 V at
// 25000-30000 counts); 550 compensates 60-75% of it.
constexpr float YAW_BACK_EMF_FF = 550.0f;

// Only yawTask sends 0x1ff: sending yaw alone would zero the shooter slots.
struct YawCommand {
  float target = 0.0f;
  bool enabled = false;
  bool spin = false;
  int16_t left = 0;
  int16_t right = 0;
  int16_t feeder = 0;
  uint32_t updated_ms = 0;
};
static YawCommand yaw_command;
static volatile int16_t yaw_last_output = 0;

static void PublishYawCommand(const YawCommand& command) {
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  yaw_command = command;
  __set_PRIMASK(primask);
}

void yawTask(void* arg) {
  UNUSED(arg);
  const uint32_t period = (osKernelGetTickFreq() + 499U) / 500U;
  uint32_t next_tick = osKernelGetTickCount();
  uint32_t last_ms = HAL_GetTick();
  float previous_yaw = 0.0f;
  float filtered_rate = 0.0f;
  bool tracking = false;
  while (true) {
    YawCommand command;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    command = yaw_command;
    __set_PRIMASK(primask);
    const uint32_t now = HAL_GetTick();
    const float dt = clip<float>((now - last_ms) * 0.001f, 0.001f, 0.05f);
    last_ms = now;
    int16_t output = 0;
    const bool fresh = now - command.updated_ms <= 100;
    const bool active = command.enabled && fresh && imu_ready &&
                        now - imu_update_ms <= 100;
    if (active) {
      const float chassis_yaw = INS_angle[0];
      const float motor_omega = yaw_motor->GetOmega();  // gimbal relative to chassis, rad/s
      const float measured = chassis_yaw + yaw_motor->GetTheta() - YAW_MOTOR_OFFSET;
      if (!tracking) {
        previous_yaw = chassis_yaw;
        filtered_rate = 0.0f;
      }
      // World yaw rate = chassis rate (from the Mahony-filtered IMU yaw) + the 6020's
      // own speed feedback. Differentiating the 13-bit encoder angle instead gave
      // ~0.38 rad/s quantization steps at 500 Hz.
      const float chassis_rate = atan2f(sinf(chassis_yaw - previous_yaw),
                                        cosf(chassis_yaw - previous_yaw)) / dt;
      previous_yaw = chassis_yaw;
      // Preserve the former alpha=0.2 at 5 ms, independent of task period.
      const float alpha = 1.0f - expf(-dt / 0.0224071f);
      filtered_rate += alpha * (chassis_rate + motor_omega - filtered_rate);
      const float error = atan2f(sinf(command.target - measured),
                                 cosf(command.target - measured));
      const float limit = command.spin ? 30000.0f : 28000.0f;
      // 0x1ff drives the GM6020 in voltage mode: while the chassis turns underneath,
      // the motor's back-EMF would otherwise have to be supplied by a growing angle
      // error (lag that varies with spin speed -> sway). Partial compensation from
      // the measured motor speed keeps some of the motor's natural damping.
      output = static_cast<int16_t>(clip<float>(22000.0f * error - 1400.0f * filtered_rate +
                                                    YAW_BACK_EMF_FF * motor_omega,
                                                -limit, limit));
    }
    tracking = active;
    yaw_last_output = output;
    if (can != nullptr && yaw_motor != nullptr) {
      const int16_t currents[4] = {output, fresh ? command.feeder : int16_t{0},
                                 fresh ? command.left : int16_t{0},
                                 fresh ? command.right : int16_t{0}};
      uint8_t data[8] = {};
      for (int i = 0; i < 4; ++i) {
        data[2 * i] = static_cast<uint16_t>(currents[i]) >> 8;
        data[2 * i + 1] = static_cast<uint16_t>(currents[i]) & 0xff;
      }
      can->Transmit(0x1ff, data, 8);
    }
    next_tick += period;
    if (static_cast<int32_t>(next_tick - osKernelGetTickCount()) <= 0)
      next_tick = osKernelGetTickCount() + period;
    osDelayUntil(next_tick);
  }
}

// Kinematic conversion ratio from wheel velocity command (rad/s) to physical chassis rotation rate (rad/s)
// For 152mm omni wheels (r ≈ 0.076m) and chassis radius R ≈ 0.22m:
// omega_chassis = (sqrt(2) * r / R) * omega_wheel ≈ 0.46

namespace board_state_can = communication::board_state_can;

// Set this to the robot's actual protocol color before deployment.
board_state_can::BoardStateCanServer board_state_server(board_state_can::Color::BLUE);
static uint8_t board_state_rx_data[8] = {};
static volatile uint32_t board_state_rx_sequence = 0;

static void BoardStateCanRxCallback(const uint8_t data[], void* args) {
  (void)args;
  std::memcpy(board_state_rx_data, data, board_state_can::HANDSHAKE_DLC);
  __DMB();
  ++board_state_rx_sequence;
}

// Called exactly once per RM_RTOS_Default_Task loop. No additional comm task is
// created; CAN remains nonblocking and motor-control timing stays independent.
static void BoardStateCanMainTaskStep() {
  static uint32_t last_rx_sequence = 0;
  uint8_t received[board_state_can::HANDSHAKE_DLC];
  uint8_t frame[board_state_can::FRAGMENT_DLC];
  const uint32_t now_ms = HAL_GetTick();
  board_state_server.SetReady(imu_ready);

  if (board_state_rx_sequence != last_rx_sequence) {
    taskENTER_CRITICAL();
    std::memcpy(received, board_state_rx_data, board_state_can::HANDSHAKE_DLC);
    last_rx_sequence = board_state_rx_sequence;
    taskEXIT_CRITICAL();
    board_state_server.HandleFrame(received, board_state_can::HANDSHAKE_DLC, now_ms);
  }

  if (board_state_server.IsResponseDue(now_ms)) {
    board_state_server.EncodeResponse(frame);
    if (can->Transmit(board_state_can::BOARD_TO_JETSON_ACK_ID, frame,
                      board_state_can::HANDSHAKE_DLC) == board_state_can::HANDSHAKE_DLC) {
      board_state_server.OnResponseQueued(now_ms);
    }
    return;
  }
  if (!board_state_server.IsStreaming()) return;

  if (!board_state_server.HasFragment()) {
    communication::board_state_t state = {};
    taskENTER_CRITICAL();
    std::memcpy(state.q, quat, sizeof(state.q));
    taskEXIT_CRITICAL();
    state.bullet_speed = (referee && referee->connection_flag_)
                             ? referee->shoot_data.initial_speed
                             : 0.0f;
    state.mode[0] = 0;          // Opaque; no mode semantics assigned here.
    board_state_server.StartSample(state, now_ms);
  }

  // The main loop runs every 5 ms, so two slots per tick sustain four fragments
  // per 10 ms sample. BUSY leaves the fragment in place for the next tick.
  for (uint32_t i = 0; i < 2 && board_state_server.HasFragment(); ++i) {
    if (!board_state_server.EncodeFragment(frame)) break;
    if (can->Transmit(board_state_can::BOARD_TO_JETSON_DATA_ID, frame,
                      board_state_can::FRAGMENT_DLC) == board_state_can::FRAGMENT_DLC) {
      board_state_server.OnFragmentQueued();
    } else {
      break;
    }
  }
}

namespace vision_can = communication::vision_can;

static uint32_t vision_session_id = 0x20261030;
static bool vision_session_ready = false;
static uint32_t vision_last_hello_ms = 0;
static uint32_t vision_last_control_ms = 0;
static uint8_t vision_control_seq = 0;

static uint8_t vision_ack_rx_data[8] = {};
static volatile uint32_t vision_ack_rx_seq = 0;

static uint8_t vision_obs_rx_data[8] = {};
static volatile uint32_t vision_obs_rx_seq = 0;

static vision_can::RelativeObservation latest_vision_obs = {};
static uint32_t vision_last_obs_ms = 0;
static bool vision_obs_fresh = false;

static void VisionHelloAckRxCallback(const uint8_t data[], void* args) {
  (void)args;
  std::memcpy(vision_ack_rx_data, data, 8);
  __DMB();
  ++vision_ack_rx_seq;
}

static void VisionRelativeRxCallback(const uint8_t data[], void* args) {
  (void)args;
  std::memcpy(vision_obs_rx_data, data, 8);
  __DMB();
  ++vision_obs_rx_seq;
}

static void VisionCanMainTaskStep(uint32_t now_ms, bool autoaim_requested) {
  static uint32_t last_ack_seq = 0;
  static uint32_t last_obs_seq = 0;
  uint8_t frame[8];

  if (vision_ack_rx_seq != last_ack_seq) {
    uint8_t ack_buf[8];
    taskENTER_CRITICAL();
    std::memcpy(ack_buf, vision_ack_rx_data, 8);
    last_ack_seq = vision_ack_rx_seq;
    taskEXIT_CRITICAL();

    uint32_t rx_session = 0;
    vision_can::Status rx_status = vision_can::Status::NOT_READY;
    if (vision_can::DecodeHelloAck(ack_buf, &rx_session, &rx_status)) {
      if (rx_session == vision_session_id && rx_status == vision_can::Status::READY) {
        vision_session_ready = true;
      }
    }
  }

  if (vision_obs_rx_seq != last_obs_seq) {
    uint8_t obs_buf[8];
    taskENTER_CRITICAL();
    std::memcpy(obs_buf, vision_obs_rx_data, 8);
    last_obs_seq = vision_obs_rx_seq;
    taskEXIT_CRITICAL();

    vision_can::RelativeObservation obs;
    if (vision_can::DecodeRelativeObservation(obs_buf, &obs)) {
      if (obs.valid) {
        latest_vision_obs = obs;
        vision_last_obs_ms = now_ms;
        vision_obs_fresh = true;
      } else {
        vision_obs_fresh = false;
      }
    }
  }

  if (!vision_session_ready) {
    if (now_ms - vision_last_hello_ms >= 100) {
      vision_last_hello_ms = now_ms;
      vision_can::EncodeHello(frame, vision_session_id);
      can->Transmit(vision_can::HELLO_CAN_ID, frame, 8);
    }
  } else {
    if (now_ms - vision_last_control_ms >= 80) {
      vision_last_control_ms = now_ms;
      vision_can::EncodeControl(frame, vision_session_id, autoaim_requested,
                                vision_can::Color::RED, vision_control_seq++);
      can->Transmit(vision_can::CONTROL_CAN_ID, frame, 8);
    }
  }
}

void RM_RTOS_Init() {
  print_use_uart(&huart7);

  // MC02 uses FDCAN instead of CAN
  can = new bsp::CAN(&hfdcan1, 0);
  RM_ASSERT_EQ(can->RegisterRxCallback(board_state_can::JETSON_TO_BOARD_ID,
                                       BoardStateCanRxCallback, nullptr),
               0, "Cannot register Omni board-state CAN callback");
  RM_ASSERT_EQ(can->RegisterRxCallback(vision_can::HELLO_ACK_CAN_ID,
                                       VisionHelloAckRxCallback, nullptr),
               0, "Cannot register vision HELLO_ACK callback");
  RM_ASSERT_EQ(can->RegisterRxCallback(vision_can::RELATIVE_OBS_CAN_ID,
                                       VisionRelativeRxCallback, nullptr),
               0, "Cannot register vision RELATIVE callback");

  /* rx_id = Master id
   * tx_id = CAN id
   * mode:
   *  MIT: MIT mode
   *  POS_VEL: position-velocity mode
   *  VEL: velocity mode  */

  // Chassis requires DJI-compatible DM3519 firmware and motor IDs 1..4.
  chassis_motor[0] = new control::Motor3508(can, 0x203);  // front right, DM3519
  chassis_motor[1] = new control::Motor3508(can, 0x202);  // back left, DM3519
  chassis_motor[2] = new control::Motor3508(can, 0x204);  // back right, DJI M3508
  chassis_motor[3] = new control::Motor3508(can, 0x201);  // front left, DM3519

  gimbal_motor = new control::Motor4310(can, 0x1c, 0x1d, control::POS_VEL);

  yaw_motor = new control::Motor6020(can, 0x205);
  flywheel_motor[0] = new control::Motor3508(can, 0x207);
  flywheel_motor[1] = new control::Motor3508(can, 0x208);
  feeder_motor = new control::Motor3508(can, 0x206);

  // MC02 uses UART5 for DBUS
  dbus = new remote::DBUS(&huart5);

  // Initialize Referee System on UART1 (115200 8N1 standard regular link)
  huart1.Init.BaudRate = 115200;
  HAL_UART_Init(&huart1);

  referee_uart = new RefereeUART(&huart1);
  referee_uart->SetupRx(512);
  referee_uart->SetupTx(512);
  referee = new communication::Referee;
  referee->SetRxCallback(OnRefereeRx);

  // IMU initialization is in imuTask() - HAL_Delay() in BMI088 constructor
  // hangs when called before the RTOS scheduler starts
}

// Last spin's statistics (wheel rad/s, chassis rad/s, buffer J). Kept in D3 SRAM4,
// which neither startup nor a system reset clears, so a ground test run without the
// USB cable can still be read back after plugging in resets the board (a power cycle
// loses it). Plain POD on purpose: no constructor may run at startup and overwrite
// the kept copy. SpinStats{} zero-initializes.
struct SpinStats {
  uint32_t duration_ms;
  uint32_t samples;
  float wheel_sum;
  float max_wheel;
  float chassis_sum;
  float max_chassis;
  float last_imu_yaw;
  float min_buffer;
  float min_scale;
};
static_assert(std::is_trivial<SpinStats>::value, "SpinStats must stay trivial");
struct PersistentSpinStats {
  static constexpr uint32_t MAGIC = 0x5350494e;  // "SPIN"
  uint32_t magic;
  SpinStats stats;
  uint32_t checksum;
  // The SRAM ECC controller holds back the last incomplete write; a reset loses it.
  // Writing these afterwards pushes the real data out (measured: without it one
  // stats word read back corrupted after a reset).
  volatile uint32_t flush[4];
  uint32_t Sum() const {
    uint32_t sum = magic;
    const auto* words = reinterpret_cast<const uint32_t*>(&stats);
    for (size_t i = 0; i < sizeof(stats) / sizeof(uint32_t); ++i) sum = sum * 31u + words[i];
    return sum;
  }
  bool Valid() const { return magic == MAGIC && checksum == Sum(); }
  void Seal() {
    magic = MAGIC;
    checksum = Sum();
    __DSB();
    for (auto& word : flush) word = 0;
    __DSB();
  }
};
static PersistentSpinStats persistent_spin_stats __attribute__((section(".RAM_D3")));

// Event log kept across resets (same SRAM4 rules as the spin stats) so a ground test
// without the USB cable can be reconstructed afterwards.
enum EventCode : uint16_t {
  EV_BOOT = 1,         // value: boot count
  EV_ARM = 2,
  EV_DISARM = 3,       // value: 1 swl UP, 2 RC link lost, 3 not armed
  EV_SPIN_START = 4,
  EV_SPIN_END = 5,     // value: duration in 0.1 s
  EV_WHEEL_STALE = 6,  // value: bitmask of wheels without fresh feedback (FR BL BR FL)
  EV_WHEEL_OK = 7,
  EV_RC_LINK = 8,      // value: 1 up, 0 lost
};
struct LoggedEvent {
  uint32_t ms;
  uint16_t code;
  int16_t value;
};
struct PersistentEvents {
  static constexpr uint32_t MAGIC = 0x45564e54;  // "EVNT"
  static constexpr uint32_t SIZE = 32;
  uint32_t magic;
  uint32_t boot_count;
  uint32_t head;  // total events ever added this log
  LoggedEvent events[SIZE];
  uint32_t checksum;
  volatile uint32_t flush[4];
  uint32_t Sum() const {
    uint32_t sum = magic * 31u + boot_count;
    sum = sum * 31u + head;
    const auto* words = reinterpret_cast<const uint32_t*>(events);
    for (size_t i = 0; i < sizeof(events) / sizeof(uint32_t); ++i) sum = sum * 31u + words[i];
    return sum;
  }
  bool Valid() const { return magic == MAGIC && checksum == Sum(); }
  void Seal() {
    magic = MAGIC;
    checksum = Sum();
    __DSB();
    for (auto& word : flush) word = 0;
    __DSB();
  }
  void Add(EventCode code, int value) {
    events[head % SIZE] = {HAL_GetTick(), code, static_cast<int16_t>(value)};
    ++head;
    Seal();
  }
};
static_assert(std::is_trivial<PersistentEvents>::value, "PersistentEvents must stay trivial");
static PersistentEvents persistent_events __attribute__((section(".RAM_D3")));

static const char* EventName(uint16_t code) {
  switch (code) {
    case EV_BOOT: return "BOOT";
    case EV_ARM: return "ARM";
    case EV_DISARM: return "DISARM";
    case EV_SPIN_START: return "SPIN+";
    case EV_SPIN_END: return "SPIN-";
    case EV_WHEEL_STALE: return "WHEEL_STALE";
    case EV_WHEEL_OK: return "WHEEL_OK";
    case EV_RC_LINK: return "RC";
    default: return "?";
  }
}

static void PrintEvents() {
  const PersistentEvents& log = persistent_events;
  const uint32_t count = log.head < PersistentEvents::SIZE ? log.head : PersistentEvents::SIZE;
  print("[EV] boots=%lu events=%lu (oldest first; ms since that boot):\r\n",
        (unsigned long)log.boot_count, (unsigned long)log.head);
  for (uint32_t k = log.head - count; k < log.head; ++k) {
    const LoggedEvent& e = log.events[k % PersistentEvents::SIZE];
    print("[EV]   %8lu %s %d\r\n", (unsigned long)e.ms, EventName(e.code), e.value);
  }
}

void RM_RTOS_Default_Task(const void* args) {
  /* press reset if no response */
  UNUSED(args);

  control::Motor4310* gimbal_container[] = {gimbal_motor};
  control::MotorCANBase* chassis_container[] = {
      chassis_motor[0], chassis_motor[1], chassis_motor[2], chassis_motor[3]};
  // PI on top of ChassisFeedforward(); gains from the free-air bench test.
  ChassisPI chassis_pid[4];
  // Standard RoboMaster feeder PID (from main_mc02.cc)
  control::PIDController feeder_pid_(40.0f, 5.0f, 10.0f);
  control::ConstrainedPID left_flywheel_pid_(30.0f, 0.8f, 0.0f, 3000.0f, 16384.0f);
  control::ConstrainedPID right_flywheel_pid_(30.0f, 0.8f, 0.0f, 3000.0f, 16384.0f);
  bool flywheel_enabled = false;
  float chassis_speed_setpoint[4] = {};
  float chassis_filtered_setpoint[4] = {};

  // Yaw PD gains from connected component version (smooth, robust, zero-windup)
  const float YAW_MAX_LEAD = 1.2f;    // max rad the target may lead the measured yaw (allows full torque up to 24000)
  const float YAW_SPIN_MAX_LEAD = 2.5f;  // spin-mode bound, kept clear of the +-pi wrap

  // Pitch limits: looking towards the motor shaft from outside, clockwise rotation moves gimbal DOWN,
  // counter-clockwise rotation moves gimbal UP. Upwards is MORE NEGATIVE.
  // Rest position at startup is zeroed at the lower mechanical stop (0.0 rad).
  // GIMBAL_PITCH_MAX is slightly below 0 (-0.05 rad / ~ -2.9 deg) to keep off the lower mechanical rest stop.
  // GIMBAL_PITCH_MIN is the upward pitch limit (-0.75 rad / ~ -43 deg).
  const float GIMBAL_PITCH_MIN = -0.75f;
  const float GIMBAL_PITCH_MAX = -0.05f;

  /* Use SetZeroPos if you want to set current motor position as zero position. If uncommented, the
   * zero position is the zero position set before */
  // DJI current mode needs no DM enable sequence.
  for (auto* motor : chassis_motor) motor->SetOutput(0);
  control::MotorCANBase::TransmitOutput(chassis_container, 4);
  osDelay(1000);
  // Repeat zero-current initialization after the motor controllers have booted.
  for (auto* motor : chassis_motor) motor->SetOutput(0);
  control::MotorCANBase::TransmitOutput(chassis_container, 4);
  gimbal_motor->SetZeroPos();
  osDelay(50);
  gimbal_motor->MotorEnable();
  osDelay(100);
  // Stay unpowered until the operator arms the remote; the enable path re-enables.
  gimbal_motor->MotorDisable();

  bool enabled = false;
  float spin_speed_smoothed = 0.0f;
  float spin_view_offset = 0.0f;
  float yaw_target = INS_angle[0] + yaw_motor->GetTheta() - YAW_MOTOR_OFFSET;
  float gimbal_yaw_measured_in_field_reference = yaw_target;

  uint32_t last_ms = HAL_GetTick();
  int print_count = 0;
  // Artificially push slightly negative (-0.05 rad) to lift off the mechanical rest stop (0.0 rad)
  float gimbal_pos = GIMBAL_PITCH_MAX;
  float desired_yaw = 0.0f;
  float desired_pitch = 0.0f;
  bool autoaim_active = false;
  int feeder_dbg = 0;
  int feeder_jam_detect_count = 0;
  int feeder_jam_pause_count = 0;
  int feeder_jam_retries = 0;
  bool feeder_jam_locked = false;

  bool last_gimbal_pmm_power = true;
  bool last_chassis_pmm_power = false;
  bool last_shooter_pmm_power = true;
  bool rc_armed = false;
  bool last_rc_link = false;
  float power_scale = 1.0f;
  uint32_t power_scale_sample_ms = 0;
  float last_buffer_j = 60.0f;
  // The current spin is accumulated in spin_stats; it replaces the kept copy only once
  // it has lasted MIN_KEPT_SPIN_MS, so short taps of the switch do not overwrite a run.
  constexpr uint32_t MIN_KEPT_SPIN_MS = 3000;
  SpinStats spin_stats = {};
  const SpinStats& kept_spin = persistent_spin_stats.stats;
  const bool spin_stats_restored = persistent_spin_stats.Valid();
  if (!spin_stats_restored) {
    persistent_spin_stats.stats = SpinStats{};
    persistent_spin_stats.Seal();
  }
  if (!persistent_events.Valid()) {
    persistent_events = PersistentEvents{};
  }
  ++persistent_events.boot_count;
  persistent_events.Add(EV_BOOT, static_cast<int>(persistent_events.boot_count));
  bool last_wheels_fresh = true;
  bool spin_stats_active = false;

  while (true) {
    BoardStateCanMainTaskStep();
    // DBUS keeps its last frame when the link drops, so a stale frame must not be
    // read as a command. After boot or a link loss, outputs stay off until the
    // operator has shown swl == UP on a live link.
    const bool rc_link = dbus->connection_flag_ && HAL_GetTick() - dbus->timestamp <= 100;
    if (!rc_link) {
      rc_armed = false;
    } else if (dbus->swl == remote::UP) {
      rc_armed = true;
    }
    if (rc_link != last_rc_link) {
      last_rc_link = rc_link;
      print(rc_link ? "RC link up (set swl UP to arm)\r\n" : "RC link LOST - outputs off\r\n");
      persistent_events.Add(EV_RC_LINK, rc_link ? 1 : 0);
    }
    const bool rc_active = rc_link && rc_armed;
    bool autoaim_requested = rc_active && dbus->swr == remote::UP;
    VisionCanMainTaskStep(HAL_GetTick(), autoaim_requested);

    float vel[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    // Consume the DBUS mouse displacement exactly once per iteration. A DBUS frame
    // carries a displacement, so reading dbus->mouse here would integrate the same
    // frame again on every pass (and keep integrating the last non-zero value after
    // the link dropped), while every frame that arrived between two passes would be
    // lost. TakeMouseDelta() sums all frames since the previous pass, drops
    // displacement that went stale and clears it, so a step that is not applied
    // below is dropped instead of being replayed when the gimbal listens again.
    remote::mouse_t mouse_delta = {};
    dbus->TakeMouseDelta(&mouse_delta);

    // Real loop time (UART printing can stretch the loop well past 5 ms)
    uint32_t now_ms = HAL_GetTick();
    float dt = clip<float>((now_ms - last_ms) / 1000.0f, 0.001f, 0.05f);
    last_ms = now_ms;

    
    // disable logic: swl == UP or no armed RC link shuts down all chassis, gimbal,
    // feeder, and flywheel motors
    if (!rc_active || dbus->swl == remote::UP) {
      YawCommand stopped;
      stopped.updated_ms = HAL_GetTick();
      PublishYawCommand(stopped);
      spin_speed_smoothed = 0.0f;
      spin_view_offset = 0.0f;
      if (enabled) {
        gimbal_motor->MotorDisable();
        print("Disabled \r\n");
        persistent_events.Add(EV_DISARM, !rc_link ? 2 : (dbus->swl == remote::UP ? 1 : 3));
        enabled = false;
        // Nothing of what happened during the shutdown belongs to the next move.
        dbus->DiscardMouseDelta();
      }
      flywheel_enabled = false;
      for (int i = 0; i < 4; ++i) {
        chassis_pid[i].Reset();
        chassis_speed_setpoint[i] = 0.0f;
        chassis_filtered_setpoint[i] = 0.0f;
        chassis_motor[i]->SetOutput(0);
      }
      control::MotorCANBase::TransmitOutput(chassis_container, 4);
      left_flywheel_pid_.Reset();
      right_flywheel_pid_.Reset();
      flywheel_motor[0]->SetOutput(0);
      flywheel_motor[1]->SetOutput(0);
      feeder_motor->SetOutput(0);
      static uint32_t idle_report_ms = 0;
      if (HAL_GetTick() - idle_report_ms >= 250) {
        idle_report_ms = HAL_GetTick();
        print("[IDLE] rc_link=%d armed=%d swl=%d | yaw theta=%.3f omega=%.2f | wheels FR=%d BL=%d BR=%d FL=%d | CAN TEC=%lu REC=%lu LEC=%lu\r\n",
              rc_link, rc_armed, (int)dbus->swl, yaw_motor->GetTheta(), yaw_motor->GetOmega(),
              (int)chassis_motor[0]->connection_flag_, (int)chassis_motor[1]->connection_flag_,
              (int)chassis_motor[2]->connection_flag_, (int)chassis_motor[3]->connection_flag_,
              (unsigned long)(hfdcan1.Instance->ECR & 0xff),
              (unsigned long)((hfdcan1.Instance->ECR >> 8) & 0x7f),
              (unsigned long)(hfdcan1.Instance->PSR & 0x7));
        const float n = kept_spin.samples ? static_cast<float>(kept_spin.samples) : 1.0f;
        print("[LAST SPIN >=3s] %.1fs | wheel mean=%.1f max=%.1f rad/s | chassis mean=%.2f max=%.2f rad/s (%.0f rpm) | "
              "min buffer=%.0fJ min scale=%.2f | ref limit=%uW buffer now=%uJ fresh=%d | %s\r\n",
              kept_spin.duration_ms / 1000.0f, kept_spin.wheel_sum / n, kept_spin.max_wheel,
              kept_spin.chassis_sum / n, kept_spin.max_chassis, kept_spin.chassis_sum / n * 9.549f,
              kept_spin.samples ? kept_spin.min_buffer : 0.0f, kept_spin.min_scale,
              referee ? referee->game_robot_status.chassis_power_limit : 0, chassis_buffer_j,
              chassis_buffer_received && HAL_GetTick() - chassis_buffer_rx_ms <= 500,
              spin_stats_restored ? "kept across reset" : "this boot");
        static uint32_t events_report_ms = 0;
        if (HAL_GetTick() - events_report_ms >= 3000) {
          events_report_ms = HAL_GetTick();
          PrintEvents();
        }
      }
      if (spin_stats_active) {
        persistent_events.Add(EV_SPIN_END, static_cast<int>(spin_stats.duration_ms / 100));
        spin_stats_active = false;
      }
      osDelay(100);
      continue;
    } else {
      if (!enabled) {
        print("Enabled \r\n");
        persistent_events.Add(EV_ARM, 0);
        for (auto& pid : chassis_pid) pid.Reset();
        gimbal_motor->MotorEnable();
        float cur_pitch = gimbal_motor->GetTheta();
        gimbal_pos = clip<float>(cur_pitch < GIMBAL_PITCH_MAX ? cur_pitch : GIMBAL_PITCH_MAX,
                                 GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
        yaw_target = INS_angle[0] + yaw_motor->GetTheta() - YAW_MOTOR_OFFSET;
        osDelay(100);
        // Drop the arm-up window: everything that arrived while the motors were being
        // enabled is cleared here, and the displacement taken at the top of this
        // iteration came from the still disabled period, so it is dropped too.
        dbus->DiscardMouseDelta();
        mouse_delta = {};
        enabled = true;
      }
    }

    // PMM Power Management:
    // If referee is connected and has reported robot status, use PMM output states.
    // Otherwise (standalone bench test), default to power ON.
    bool gimbal_pmm_power = true;
    bool chassis_pmm_power = true;
    bool shooter_pmm_power = true;
    if (referee && referee->connection_flag_ && prev_pmm_chassis != 0xFF) {
      gimbal_pmm_power = (prev_pmm_gimbal != 0);
      chassis_pmm_power = (prev_pmm_chassis != 0);
      shooter_pmm_power = (prev_pmm_shooter != 0);
    }

    // DJI mode resumes through current commands; reset controllers on PMM transitions.
    if (last_chassis_pmm_power != chassis_pmm_power) {
      last_chassis_pmm_power = chassis_pmm_power;
      for (auto& pid : chassis_pid) pid.Reset();
    }

    // PMM Gimbal Power handling (Issue 1):
    // Gimbal (DM4310) and Yaw (GM6020) motors are wired to MiniPC port on PMM,
    // which remains permanently powered. When the referee cuts power to the Gimbal port,
    // we must software disable both motors.
    if (!gimbal_pmm_power) {
      if (last_gimbal_pmm_power) {
        last_gimbal_pmm_power = false;
        print(">>> [PMM] Gimbal power CUT by referee - software disabling gimbal & yaw <<<\r\n");
        gimbal_motor->MotorDisable();
      }
    } else {
      if (!last_gimbal_pmm_power) {
        last_gimbal_pmm_power = true;
        print(">>> [PMM] Gimbal power RESTORED - re-enabling gimbal & yaw <<<\r\n");
        if (enabled) {
          gimbal_motor->MotorEnable();
          float cur_pitch = gimbal_motor->GetTheta();
          gimbal_pos = clip<float>(cur_pitch < GIMBAL_PITCH_MAX ? cur_pitch : GIMBAL_PITCH_MAX,
                                   GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
          yaw_target = INS_angle[0] + yaw_motor->GetTheta() - YAW_MOTOR_OFFSET;
        }
      }
    }

    // PMM Shooter Power handling (Issue 2):
    // Flywheels are wired through MiniPC port (permanently powered).
    // Cutting Ammo Booster power only physically cuts feeder. We must software disable flywheels.
    if (!shooter_pmm_power) {
      if (last_shooter_pmm_power) {
        last_shooter_pmm_power = false;
        print(">>> [PMM] Shooter power CUT by referee - disabling flywheels & feeder <<<\r\n");
      }
    } else {
      if (!last_shooter_pmm_power) {
        last_shooter_pmm_power = true;
        print(">>> [PMM] Shooter power RESTORED <<<\r\n");
      }
    }

    // Auto-spin while keyboard R is held with swr == DOWN (remote wheel dial is broken / ignored).
    // With live referee buffer data the spin asks for the wheel-speed cap and the power
    // governor below holds the chassis at the referee's limit; without it (bench, no
    // referee) nothing bounds the power, so spin stays at a moderate speed.
    const bool buffer_fresh =
        chassis_buffer_received && HAL_GetTick() - chassis_buffer_rx_ms <= 500;
    constexpr float SPIN_WHEEL_LIMIT = 40.0f;
    const float AUTO_SPIN_SPEED = buffer_fresh ? SPIN_WHEEL_LIMIT : 20.0f;  // wheel rad/s
    // Spin while R is held with swr DOWN; releasing either coasts down at the release rate.
    const bool spin_requested = dbus->swr == remote::DOWN && dbus->keyboard.bit.R;
    // Fixed-rate spin entry/release (wheel rad/s^2) and incremental view-offset changes.
    const float spin_goal = spin_requested ? AUTO_SPIN_SPEED : 0.0f;
    const float spin_step = (spin_requested ? 40.0f : 80.0f) * dt;  // release: 40 -> 0 in 0.5 s
    spin_speed_smoothed += clip<float>(spin_goal - spin_speed_smoothed, -spin_step, spin_step);
    // The offset stays while the coast-down is above half spin speed, so a quick R
    // re-press does not shift the view again; it is removed once the spin has mostly died.
    const bool keep_view_offset = spin_requested || spin_speed_smoothed > 0.5f * AUTO_SPIN_SPEED;
    const float view_goal = keep_view_offset ? DEG2RAD(15.0f) : 0.0f;
    const float view_step = DEG2RAD(125.0f) * dt;  // full 15-degree offset in 0.12 s
    const float view_delta = clip<float>(view_goal - spin_view_offset, -view_step, view_step);
    spin_view_offset += view_delta;
    yaw_target += view_delta;
    float auto_yaw_omega = spin_speed_smoothed;
    float chassis_yaw_omega_target = auto_yaw_omega;

    // Field-reference gimbal yaw (from connected component version)
    gimbal_yaw_measured_in_field_reference = INS_angle[0] + yaw_motor->GetTheta() - YAW_MOTOR_OFFSET;

    // Keyboard linear translation (WASD) with optional Shift boost:
    // W/S: forward/backward, A/D: strafe left/right (in field/gimbal reference frame)
    float kb_speed = dbus->keyboard.bit.SHIFT ? 40.0f : 20.0f;
    float kb_y = 0.0f;
    float kb_x = 0.0f;
    if (dbus->keyboard.bit.W) kb_y += kb_speed;
    if (dbus->keyboard.bit.S) kb_y -= kb_speed;
    if (dbus->keyboard.bit.A) kb_x -= kb_speed;
    if (dbus->keyboard.bit.D) kb_x += kb_speed;

    // Right stick vertical (ch1) is forward/back (y); right stick horizontal (ch0) is strafe (x).
    // Unified with keyboard WASD so both inputs move along the exact same axes.
    // Remove center drift and rescale the remaining travel continuously.
    const auto translation_stick = [](int raw) {
      constexpr int deadband = 30;
      if (raw >= -deadband && raw <= deadband) return 0.0f;
      const float magnitude = clip<float>((abs(raw) - deadband) / 630.0f, 0.0f, 1.0f);
      return (raw < 0 ? -1.0f : 1.0f) * magnitude * 40.0f;
    };
    float stick_y = translation_stick(dbus->ch1);
    float stick_x = translation_stick(dbus->ch0);
    float y = stick_y + kb_y;
    float x = stick_x + kb_x;
    y = clip<float>(y, -40.0f, 40.0f);
    x = clip<float>(x, -40.0f, 40.0f);

    // Deadband on ch2 to prevent transmitter stick center drift from nudging yaw CCW
    int16_t ch2_raw = dbus->ch2;
    if (abs(ch2_raw) < 25) {
      ch2_raw = 0;
    }
    float yaw_omega = clip<float>(-ch2_raw / 660.0f * 10.0f, -10.0f, 10.0f);  // yaw (slower)

    bool vision_locked = (now_ms - vision_last_obs_ms <= 120);
    float gimbal_vel = 0.0f;

    // Mouse steps are counted as angle increments (no dt). They are only applied
    // while the outputs are enabled and while autoaim does not own a locked gimbal;
    // the displacement has already been consumed above either way.
    const bool mouse_active = autoaim_requested ? !vision_locked : true;
    const omni::MouseGimbalStep mouse_step =
        omni::ComputeMouseGimbalStep(mouse_delta.x, mouse_delta.y, enabled, mouse_active);

    constexpr float AUTOAIM_YAW_SMOOTH = 0.16f;
    constexpr float AUTOAIM_PITCH_SMOOTH = 0.15f;

    if (autoaim_requested) {
      if (!autoaim_active) {
        desired_yaw = gimbal_yaw_measured_in_field_reference;
        desired_pitch = gimbal_pos;
        autoaim_active = true;
      }

      if (vision_obs_fresh && vision_locked) {
        desired_yaw = gimbal_yaw_measured_in_field_reference + latest_vision_obs.delta_yaw;
        desired_pitch = clip<float>(gimbal_pos + latest_vision_obs.delta_pitch, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
        vision_obs_fresh = false;
      }

      if (vision_locked) {
        float yaw_err = atan2f(sinf(desired_yaw - yaw_target), cosf(desired_yaw - yaw_target));
        yaw_target += AUTOAIM_YAW_SMOOTH * yaw_err;
        gimbal_pos += AUTOAIM_PITCH_SMOOTH * (desired_pitch - gimbal_pos);
        gimbal_pos = clip<float>(gimbal_pos, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
        gimbal_vel = 0.0f;
      } else {
        yaw_target += yaw_omega * dt + mouse_step.yaw;
        gimbal_vel = clip<float>(-dbus->ch3 / 660.0f * 30.0f, -30.0f, 30.0f);
        gimbal_pos += gimbal_vel * dt + mouse_step.pitch;
        gimbal_pos = clip<float>(gimbal_pos, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
      }
    } else {
      if (autoaim_active) {
        autoaim_active = false;
      }
      vision_obs_fresh = false;

      yaw_target += yaw_omega * dt + mouse_step.yaw;
      gimbal_vel = clip<float>(-dbus->ch3 / 660.0f * 30.0f, -30.0f, 30.0f);
      gimbal_pos += gimbal_vel * dt + mouse_step.pitch;
      gimbal_pos = clip<float>(gimbal_pos, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
    }

    // World yaw target changes only from operator/autoaim input, never chassis spin.
    yaw_target = atan2f(sinf(yaw_target), cosf(yaw_target));

    float delta_yaw = yaw_target - gimbal_yaw_measured_in_field_reference;
    float cos = cosf(delta_yaw);
    float sin = sinf(delta_yaw);
    delta_yaw = atan2f(sin, cos);  // wrap to [-pi, pi]

    // Anti-windup: don't let the target run away from the measured heading when the
    // stick is held (otherwise the error saturates and flips sign at +-pi -> violent snap)
    if (!spin_requested && fabsf(delta_yaw) > YAW_MAX_LEAD) {
      delta_yaw = copysignf(YAW_MAX_LEAD, delta_yaw);
      if (fabsf(yaw_omega) > 0.001f || fabsf(mouse_step.yaw) > 0.00001f || autoaim_requested)
        yaw_target = gimbal_yaw_measured_in_field_reference + delta_yaw;
    } else if (spin_requested && fabsf(delta_yaw) > YAW_SPIN_MAX_LEAD) {
      // While spinning the world target is held, but if the gimbal ever lags this far
      // the yaw task's wrapped error would flip sign at +-pi; pull the target back.
      delta_yaw = copysignf(YAW_SPIN_MAX_LEAD, delta_yaw);
      yaw_target = gimbal_yaw_measured_in_field_reference + delta_yaw;
    }

    // Feedback and 0x1ff transmission run independently in the 500 Hz yaw task.
    const int16_t yaw_output = yaw_last_output;

    // If gimbal power is cut on PMM, software disable yaw output and hold tracking
    if (!gimbal_pmm_power || !imu_ready || HAL_GetTick() - imu_update_ms > 100) {
      yaw_target = INS_angle[0] + yaw_motor->GetTheta() - YAW_MOTOR_OFFSET;
      float cur_pitch = gimbal_motor->GetTheta();
      gimbal_pos = clip<float>(cur_pitch < GIMBAL_PITCH_MAX ? cur_pitch : GIMBAL_PITCH_MAX,
                               GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
    }

    // Preserve gimbal-relative control; compensate the chassis orientation by 90 degrees.
    const float chassis_direction_offset = -M_PI / 2.0f;
    float chasis_yaw_measured_in_field_reference =
        gimbal_yaw_measured_in_field_reference - INS_angle[0] + chassis_direction_offset;
    float cos_yaw = cosf(chasis_yaw_measured_in_field_reference);
    float sin_yaw = sinf(chasis_yaw_measured_in_field_reference);
    float chassis_x = x * cos_yaw - y * sin_yaw;
    float chassis_y = x * sin_yaw + y * cos_yaw;
    x = chassis_x;
    y = chassis_y;

    // Original 45-degree wheel-speed decomposition.
    float alpha = (x - y) / 1.4142f;
    float beta = (y + x) / 1.4142f;

    // Wheel-speed cap while spinning (the power governor bounds the actual speed) and
    // 40 rad/s otherwise. A 40 cap measured ~13.4 rad/s mean chassis spin at 120 W.
    const float wheel_limit = spin_requested ? SPIN_WHEEL_LIMIT : 40.0f;

    // Limit omega to available chassis motor headroom while translating
    float max_omega = max(0.0f, wheel_limit - max(fabsf(alpha), fabsf(beta)));
    chassis_yaw_omega_target = clip<float>(chassis_yaw_omega_target, -max_omega, max_omega);

    vel[0] = -beta + chassis_yaw_omega_target;
    vel[1] = beta + chassis_yaw_omega_target;
    vel[2] = -alpha + chassis_yaw_omega_target;
    vel[3] = alpha + chassis_yaw_omega_target;
    // Preserve the translation/spin mix while bounding every wheel to wheel_limit.
    float peak_wheel_speed = wheel_limit;
    for (float speed : vel) peak_wheel_speed = max(peak_wheel_speed, fabsf(speed));
    for (float& speed : vel) speed *= wheel_limit / peak_wheel_speed;

    // Stop chassis commands when referee PMM chassis power is off.
    if (!chassis_pmm_power) {
      vel[0] = 0.0f;
      vel[1] = 0.0f;
      vel[2] = 0.0f;
      vel[3] = 0.0f;
    } else if (buffer_fresh) {
      // Power governor on the referee buffer energy (~10 Hz). The buffer only drains
      // while the chassis is over the referee's limit, so holding it near
      // BUFFER_TARGET_J keeps the average power at that limit, whatever its value.
      // A full buffer means there is headroom: probe the speed scale back up.
      constexpr float BUFFER_TARGET_J = 35.0f;
      constexpr float BUFFER_FULL_J = 55.0f;
      const uint32_t sample_ms = chassis_buffer_rx_ms;
      if (sample_ms != power_scale_sample_ms) {
        power_scale_sample_ms = sample_ms;
        const float buffer = chassis_buffer_j;
        const float drop = last_buffer_j - buffer;  // J per ~100 ms sample
        last_buffer_j = buffer;
        if (drop > 3.0f && buffer < BUFFER_FULL_J + 5.0f) {
          // Draining fast (well over the limit): cut before the buffer is half gone.
          power_scale *= max(0.75f, 1.0f - 0.03f * drop);
        } else if (buffer < BUFFER_TARGET_J) {
          // Deeper below target -> stronger cut, at most 30% per sample.
          power_scale *= max(0.7f, 1.0f - 0.02f * (BUFFER_TARGET_J - buffer));
        } else if (buffer >= BUFFER_FULL_J) {
          power_scale += 0.05f;
        } else {
          power_scale += 0.01f;
        }
        if (buffer < 10.0f) power_scale = min(power_scale, 0.3f);
        power_scale = clip<float>(power_scale, 0.15f, 1.0f);
      }
      for (float& wheel_velocity : vel) wheel_velocity *= power_scale;
    } else {
      power_scale = 1.0f;
    }

    // Spin statistics, printed in [IDLE] so a test run without the USB cable can be
    // read back afterwards.
    if (spin_requested && enabled) {
      if (!spin_stats_active) {
        spin_stats_active = true;
        spin_stats = SpinStats{};
        spin_stats.min_buffer = 255.0f;
        spin_stats.min_scale = 1.0f;
        spin_stats.last_imu_yaw = INS_angle[0];
        persistent_events.Add(EV_SPIN_START, 0);
      }
      float wheel_sum = 0.0f;
      for (int i = 0; i < 4; ++i)
        wheel_sum += fabsf(chassis_motor[i]->GetOmega()) / CHASSIS_FEEDBACK_TO_WHEEL_RATIO[i];
      const float wheel_mean = wheel_sum / 4.0f;
      // Chassis spin rate from the IMU yaw (the IMU is on the chassis).
      const float chassis_rate =
          fabsf(atan2f(sinf(INS_angle[0] - spin_stats.last_imu_yaw),
                       cosf(INS_angle[0] - spin_stats.last_imu_yaw))) / dt;
      spin_stats.last_imu_yaw = INS_angle[0];
      if (spin_stats.samples > 0) {
        spin_stats.max_wheel = max(spin_stats.max_wheel, wheel_mean);
        spin_stats.max_chassis = max(spin_stats.max_chassis, chassis_rate);
      }
      spin_stats.wheel_sum += wheel_mean;
      spin_stats.chassis_sum += chassis_rate;
      spin_stats.samples++;
      if (buffer_fresh) spin_stats.min_buffer = min(spin_stats.min_buffer, (float)chassis_buffer_j);
      spin_stats.min_scale = min(spin_stats.min_scale, power_scale);
      spin_stats.duration_ms += static_cast<uint32_t>(dt * 1000.0f + 0.5f);
      if (spin_stats.duration_ms >= MIN_KEPT_SPIN_MS) {
        persistent_spin_stats.stats = spin_stats;
        persistent_spin_stats.Seal();
      }
    } else if (spin_stats_active) {
      persistent_events.Add(EV_SPIN_END, static_cast<int>(spin_stats.duration_ms / 100));
      spin_stats_active = false;
    }

    // Flywheel Control:
    // Left flywheel (0x207): -750 rad/s
    // Right flywheel (0x208): +750 rad/s
    // swl == DOWN enables flywheels (provided shooter PMM power is active);
    // swl != DOWN or shooter PMM power cut disables them and applies smooth viscous braking
    const float FLYWHEEL_TARGET_SPEED = 750.0f;
    if (dbus->swl == remote::DOWN && shooter_pmm_power) {
      if (!flywheel_enabled) {
        flywheel_enabled = true;
        left_flywheel_pid_.Reset();
        right_flywheel_pid_.Reset();
        print("FLYWHEEL: ON (-750/+750 rad/s)\r\n");
      }
    } else {
      if (flywheel_enabled) {
        flywheel_enabled = false;
        left_flywheel_pid_.Reset();
        right_flywheel_pid_.Reset();
        print("FLYWHEEL: BRAKING TO REST\r\n");
      }
    }

    int16_t left_flywheel_output = 0;
    int16_t right_flywheel_output = 0;
    float left_omega = flywheel_motor[0]->GetOmega();
    float right_omega = flywheel_motor[1]->GetOmega();

    if (flywheel_enabled) {
      // Steady running at -750 and +750 rad/s with anti-windup PI (Kd=0 prevents torque ripple & yaw gyro jitter)
      float left_diff = -FLYWHEEL_TARGET_SPEED - left_omega;
      float right_diff = FLYWHEEL_TARGET_SPEED - right_omega;
      left_flywheel_output = left_flywheel_pid_.ComputeConstrainedOutput(left_diff);
      right_flywheel_output = right_flywheel_pid_.ComputeConstrainedOutput(right_diff);
    } else if (!shooter_pmm_power) {
      // Software disabled: PMM shooter power cut by referee
      left_flywheel_output = 0;
      right_flywheel_output = 0;
    } else {
      // Active viscous dynamic braking:
      // Applies braking current strictly opposing the current spin direction (-k_brake * omega).
      // Decelerates both flywheels smoothly, synchronously, and quickly in ~0.6s without overshoot,
      // windup, or reversing past zero.
      const float k_brake = 3.0f;          // counts per rad/s (~2700 counts / 3.3 A at 900 rad/s)
      const float stop_threshold = 25.0f;  // rad/s (~240 RPM), below which outputs cut off cleanly to 0

      if (left_omega < -stop_threshold) {
        // Left is spinning negative -> positive braking current
        float brake = clip<float>(-k_brake * left_omega, 0.0f, 3500.0f);
        left_flywheel_output = (int16_t)brake;
      } else {
        left_flywheel_output = 0;
      }

      if (right_omega > stop_threshold) {
        // Right is spinning positive -> negative braking current
        float brake = clip<float>(-k_brake * right_omega, -3500.0f, 0.0f);
        right_flywheel_output = (int16_t)brake;
      } else {
        right_flywheel_output = 0;
      }
    }

    // Robust 80 rad/s (~760 RPM) Feedforward + Proportional Control:
    // Uses 1100 counts base feedforward + 25 * error.
    // Clamped strictly to [200, 3500]:
    // - Prevents +/-16384 full-current saturation and 900 rad/s wild oscillation.
    // - At omega=0, delivers 3100 counts breakaway kick immediately.
    // - In steady state, locks right onto 80 rad/s with ~1100 counts.
    // Feeder triggered by Mouse Left Click or Keyboard F / E (R toggles spin)
    // Dial firing disabled until actual center/direction is measured on this DT7.
    bool feeder_requested = dbus->mouse.l ||
                            dbus->keyboard.bit.F ||
                            dbus->keyboard.bit.E;

    // Referee System Regulation: Barrel Heat Limit & Shooter Power Protection (RMUL Rule 3.3.1.3)
    // Each 17mm projectile adds 10 heat.
    // Exceeding limit locks launcher and blinds FPV until heat = 0.
    // Exceeding limit + 100 permanently locks launcher for the entire round.
    if (!shooter_pmm_power) {
      feeder_requested = false;  // 24V shooter power cut by referee / PMM
    } else if (referee && referee->connection_flag_) {
      uint16_t heat_limit = referee->game_robot_status.shooter_barrel_heat_limit;
      uint16_t current_heat = referee->power_heat_data.shooter_17mm_barrel_heat;
      if (heat_limit > 0 && (current_heat + 15 >= heat_limit)) {
        feeder_requested = false;  // Suppress feeding to avoid overheat penalty
      }
    }

    float feeder_target_velocity = 0.0f;
    if (feeder_requested) {
      feeder_target_velocity = 110.0f;
    } else {
      feeder_jam_detect_count = 0;
      feeder_jam_pause_count = 0;
      feeder_jam_retries = 0;
      feeder_jam_locked = false;
    }

    int16_t feeder_output = 0;
    if (feeder_target_velocity > 0.0f) {
      float current_omega = feeder_motor->GetOmega();

      if (feeder_jam_locked) {
        feeder_output = 0;
      } else if (feeder_jam_pause_count > 0) {
        feeder_jam_pause_count--;
        if (feeder_jam_pause_count > 20) {
          // Very gentle micro-reverse (-1.0 A = -820 counts) for 50 ms to un-pinch the wedged ball
          feeder_output = -820;
        } else {
          // Zero-current rest for 100 ms to let mechanism settle
          feeder_output = 0;
        }
      } else {
        if (current_omega < 15.0f) {
          if (++feeder_jam_detect_count >= 70) {  // ~350 ms stalled (gives full tank time to spin up)
            feeder_jam_detect_count = 0;
            feeder_jam_retries++;
            if (feeder_jam_retries >= 3) {
              feeder_jam_locked = true;
              feeder_output = 0;
              print("FEED: HARD JAM LOCKED (toggle switch to reset)\r\n");
            } else {
              feeder_jam_pause_count = 30;  // 150 ms relief (50 ms @ -1A, 100 ms @ 0A)
              feeder_output = -820;
              print("FEED: JAM DETECTED - -1A relief (retry %d/3)\r\n", feeder_jam_retries);
            }
          }
        } else {
          feeder_jam_detect_count = 0;
          if (current_omega > 60.0f) {
            feeder_jam_retries = 0;  // successfully feeding, reset retry count
          }
        }

        if (!feeder_jam_locked && feeder_jam_pause_count == 0) {
          float feeder_diff = feeder_motor->GetOmegaDelta(feeder_target_velocity);
          float raw_out;
          if (current_omega < 25.0f) {
            // Full-tank breakaway boost: delivers ~8400 counts (~10.4 A, 12 Nm) to overcome static ammo column weight
            raw_out = 2000.0f + 40.0f * feeder_diff;
            raw_out = clip<float>(raw_out, 1000.0f, 8500.0f);
          } else {
            // Normal running mode
            raw_out = 2000.0f + 25.0f * feeder_diff;
            raw_out = clip<float>(raw_out, 1000.0f, 6000.0f);
          }
          feeder_output = (int16_t)raw_out;
        }
      }

      if (++feeder_dbg >= 40) {
        feeder_dbg = 0;
        print("FEED tgt=%.1f omega=%.1f out=%d\r\n",
              feeder_target_velocity, current_omega, (int)feeder_output);
      }
    }


    // A missing wheel feedback stops the whole chassis, not just that wheel.
    bool chassis_feedback_fresh = true;
    int stale_mask = 0;
    const uint32_t feedback_now = HAL_GetTick();
    for (int i = 0; i < 4; ++i) {
      const bool fresh = chassis_motor[i]->connection_flag_ &&
          feedback_now - chassis_motor[i]->GetLastFeedbackMs() <= 100;
      chassis_feedback_fresh &= fresh;
      if (!fresh) stale_mask |= 1 << i;
    }
    if (chassis_feedback_fresh != last_wheels_fresh) {
      last_wheels_fresh = chassis_feedback_fresh;
      persistent_events.Add(chassis_feedback_fresh ? EV_WHEEL_OK : EV_WHEEL_STALE, stale_mask);
    }
    float log_wheel_omega[4] = {};
    float log_wheel_output[4] = {};
    for (int i = 0; i < 4; ++i) {
      if (!chassis_pmm_power || !chassis_feedback_fresh) {
        chassis_pid[i].Reset();
        chassis_speed_setpoint[i] = 0.0f;
        chassis_filtered_setpoint[i] = 0.0f;
        chassis_motor[i]->SetOutput(0);
      } else {
        // Short command ramp: full-speed release reaches zero in ~133 ms.
        // Reversal first brakes through zero; emergency disable bypasses this ramp.
        const float previous = chassis_speed_setpoint[i];
        const bool reversing = previous * vel[i] < 0.0f;
        const float target = reversing ? 0.0f : vel[i];
        const bool slowing = reversing || fabsf(target) < fabsf(previous);
        const float step = (slowing ? 300.0f : 200.0f) * dt;
        chassis_speed_setpoint[i] += clip<float>(target - previous, -step, step);
        // DM3519 IDs 3/2/1: first-order target filter with a 10 ms time constant.
        // Index 2 is the DJI M3508 (ID4), which keeps its existing target response.
        if (i != 2) {
          const float filter_alpha = 1.0f - expf(-dt / 0.01f);
          chassis_filtered_setpoint[i] += filter_alpha *
              (chassis_speed_setpoint[i] - chassis_filtered_setpoint[i]);
          if (fabsf(target) < 0.1f && fabsf(chassis_filtered_setpoint[i]) < 0.1f)
            chassis_filtered_setpoint[i] = 0.0f;
        } else {
          chassis_filtered_setpoint[i] = chassis_speed_setpoint[i];
        }
        if (fabsf(chassis_filtered_setpoint[i]) < 0.1f && fabsf(target) < 0.1f)
          chassis_pid[i].Reset();
        const float wheel_omega = CHASSIS_MOTOR_DIRECTION[i] * chassis_motor[i]->GetOmega() /
                                  CHASSIS_FEEDBACK_TO_WHEEL_RATIO[i];
        const float output = clip<float>(
            ChassisFeedforward(i, chassis_filtered_setpoint[i]) +
                chassis_pid[i].Compute(chassis_filtered_setpoint[i] - wheel_omega,
                                       fabsf(target - chassis_filtered_setpoint[i]) < 0.5f, dt),
            -CHASSIS_OUTPUT_LIMIT, CHASSIS_OUTPUT_LIMIT);
        chassis_motor[i]->SetOutput(static_cast<int16_t>(CHASSIS_MOTOR_DIRECTION[i] * output));
        log_wheel_omega[i] = wheel_omega;
        log_wheel_output[i] = output;
      }
    }
    // DM4310 in POS_VEL mode: 2nd argument is velocity limit (rad/s) for profile position tracking.
    // Must be positive non-zero so motor tracks gimbal_pos whether commanded by mouse, stick, or autoaim.
    const float GIMBAL_PITCH_TRACKING_VEL = 20.0f;
    gimbal_motor->SetOutput(gimbal_pos, GIMBAL_PITCH_TRACKING_VEL);
    flywheel_motor[0]->SetOutput(left_flywheel_output);
    flywheel_motor[1]->SetOutput(right_flywheel_output);
    feeder_motor->SetOutput(feeder_output);
    YawCommand command;
    command.target = yaw_target;
    command.enabled = enabled && gimbal_pmm_power;
    command.spin = spin_requested;
    // Match Motor3508::SetOutput's current clamp when packing the shared frame.
    command.left = clip<int16_t>(left_flywheel_output, -14690, 14690);
    command.right = clip<int16_t>(right_flywheel_output, -14690, 14690);
    command.feeder = clip<int16_t>(feeder_output, -14690, 14690);
    command.updated_ms = HAL_GetTick();
    PublishYawCommand(command);
    control::MotorCANBase::TransmitOutput(chassis_container, 4);
    if (gimbal_pmm_power) {
      control::Motor4310::TransmitOutput(gimbal_container, 1);
    }

    // 20 Hz chassis/yaw log for ground tests, wheel order FR(ID3) BL(ID2) BR(ID4) FL(ID1):
    // setpoint/measured/output per wheel (wheel rad/s, counts), then yaw error (deg),
    // yaw output (counts) and 6020 speed relative to the chassis (rad/s).
    static uint32_t chassis_log_ms = 0;
    if (now_ms - chassis_log_ms >= 50) {
      chassis_log_ms = now_ms;
      print("[CH] %lu spin=%d | %.1f %.1f %d | %.1f %.1f %d | %.1f %.1f %d | %.1f %.1f %d | yaw %.1f %d %.2f\r\n",
            (unsigned long)now_ms, spin_requested,
            chassis_filtered_setpoint[0], log_wheel_omega[0], (int)log_wheel_output[0],
            chassis_filtered_setpoint[1], log_wheel_omega[1], (int)log_wheel_output[1],
            chassis_filtered_setpoint[2], log_wheel_omega[2], (int)log_wheel_output[2],
            chassis_filtered_setpoint[3], log_wheel_omega[3], (int)log_wheel_output[3],
            RAD2DEG(delta_yaw), (int)yaw_last_output, yaw_motor->GetOmega());
    }

    // Throttled debug print every 500ms for heading and gimbal alignment verification
    if (++print_count >= 100) {
      print_count = 0;
      float imu_deg = RAD2DEG(INS_angle[0]);
      float field_deg = RAD2DEG(gimbal_yaw_measured_in_field_reference);
      float target_deg = RAD2DEG(yaw_target);
      float muzzle_deg = RAD2DEG(yaw_motor->GetTheta() - YAW_MOTOR_OFFSET);
      print("[CONN] IMU: %5.1f | Field: %5.1f | Target: %5.1f | MuzzleRel: %5.1f deg | Pitch: %5.2f rad (%4.1f deg) | out: %d | err: %5.1f\r\n",
            imu_deg, field_deg, target_deg, muzzle_deg, gimbal_pos, RAD2DEG(gimbal_pos), yaw_output, RAD2DEG(delta_yaw));
      const uint32_t buffer_age = HAL_GetTick() - chassis_buffer_rx_ms;
      print("[SPIN POWER] spin=%d buffer=%uJ fresh=%d age=%lums scale=%.2f | CAN TEC=%lu REC=%lu LEC=%lu\r\n",
            spin_requested, chassis_buffer_j, buffer_fresh, (unsigned long)buffer_age, power_scale,
            (unsigned long)(hfdcan1.Instance->ECR & 0xff),
            (unsigned long)((hfdcan1.Instance->ECR >> 8) & 0x7f),
            (unsigned long)(hfdcan1.Instance->PSR & 0x7));
      print("[CHASSIS] M0(0x203): conn=%d w=%.2f | M1(0x202): conn=%d w=%.2f | M2(0x204): conn=%d w=%.2f | M3(0x201): conn=%d w=%.2f\r\n",
            (int)chassis_motor[0]->connection_flag_, chassis_motor[0]->GetOmega(),
            (int)chassis_motor[1]->connection_flag_, chassis_motor[1]->GetOmega(),
            (int)chassis_motor[2]->connection_flag_, chassis_motor[2]->GetOmega(),
            (int)chassis_motor[3]->connection_flag_, chassis_motor[3]->GetOmega());
      if (referee && referee->connection_flag_) {
        print("[REF] HP:%u/%u Heat:%u/%u Buf:%uJ PwrLim:%uW | ShotSpd:%.1fm/s\r\n",
              referee->game_robot_status.current_HP,
              referee->game_robot_status.maximum_HP,
              referee->power_heat_data.shooter_17mm_barrel_heat,
              referee->game_robot_status.shooter_barrel_heat_limit,
              referee->power_heat_data.buffer_energy,
              referee->game_robot_status.chassis_power_limit,
              referee->shoot_data.initial_speed);
      }
    }

    osDelay(5);
  }
}
