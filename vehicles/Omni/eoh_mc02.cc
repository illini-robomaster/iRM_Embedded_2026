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
 * - Print via UART10 (huart10)
 * - DBUS via UART5 (huart5)
 * - IMU via SPI2 (hspi2) with BMI088 only (no IST8310/heater)
 * - IMU initialized in imuTask (HAL_Delay in BMI088 ctor needs RTOS running)
 */

#include <cmath>
#include <cstring>
#include <memory>

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

// Blocking UART prints stall the control loop (motors hold the last command) -> jerks.
#define EOH_DEBUG_PRINT 0

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

      float gx_rad = DEG2RAD(gyro[0]);
      float gy_rad = DEG2RAD(gyro[1]);
      float gz_rad = DEG2RAD(gyro[2]);

      MahonyAHRSupdateIMU(quat, gx_rad, gy_rad, gz_rad,
                          accel[0], accel[1], accel[2]);

      INS_angle[0] = atan2f(2.0f * (quat[0] * quat[3] + quat[1] * quat[2]),
                            1.0f - 2.0f * (quat[2] * quat[2] + quat[3] * quat[3]));  // yaw
      INS_angle[1] = asinf(2.0f * (quat[0] * quat[2] - quat[3] * quat[1]));          // pitch
      INS_angle[2] = atan2f(2.0f * (quat[0] * quat[1] + quat[2] * quat[3]),
                            1.0f - 2.0f * (quat[1] * quat[1] + quat[2] * quat[2]));  // roll
      imu_ready = true;
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

void refereeTask(void* arg) {
  UNUSED(arg);
  uint8_t* rx_data = nullptr;

  while (true) {
    uint32_t flags = osThreadFlagsWait(REFEREE_RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & REFEREE_RX_SIGNAL) {
      int32_t length = referee_uart->Read(&rx_data);
      if (length > 0 && rx_data != nullptr && referee != nullptr) {
        referee->Receive(communication::package_t{rx_data, (int)length});
      }
    }
  }
}

void RM_RTOS_Threads_Init(void) {
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
}

// Global peripherals
bsp::CAN* can = nullptr;
control::MotorDM3519* chassis_motor[4];
control::Motor4310* gimbal_motor;
control::Motor6020* yaw_motor;
control::Motor3508* flywheel_motor[2];
control::Motor3508* feeder_motor;

remote::DBUS* dbus = nullptr;

const float YAW_MOTOR_OFFSET = 0.762f;  // in rad (calibrated 12 o'clock alignment: 2.857 - 2.094)

// Kinematic conversion ratio from wheel velocity command (rad/s) to physical chassis rotation rate (rad/s)
// For 152mm omni wheels (r ≈ 0.076m) and chassis radius R ≈ 0.22m:
// omega_chassis = (sqrt(2) * r / R) * omega_wheel ≈ 0.46
const float CHASSIS_SPIN_RATIO = 0.46f;

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
  print_use_uart(&huart10);

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

  /* Make sure motor is set to the correct mode (in helper tool). Otherwise, motor won't start */
  chassis_motor[0] = new control::MotorDM3519(can, 0x10, 0x11, control::VEL);  // front right
  chassis_motor[1] = new control::MotorDM3519(can, 0x12, 0x13, control::VEL);  // back left
  chassis_motor[2] = new control::MotorDM3519(can, 0x18, 0x19, control::VEL);  // back right
  chassis_motor[3] = new control::MotorDM3519(can, 0x16, 0x17, control::VEL);  // front left

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

  // IMU initialization is in imuTask() - HAL_Delay() in BMI088 constructor
  // hangs when called before the RTOS scheduler starts
}

void RM_RTOS_Default_Task(const void* args) {
  /* press reset if no response */
  UNUSED(args);

  control::Motor4310* gimbal_container[] = {gimbal_motor};
  control::MotorCANBase* dji_container[] = {yaw_motor, flywheel_motor[0], flywheel_motor[1], feeder_motor};
  // Standard RoboMaster feeder PID (from main_mc02.cc)
  control::PIDController feeder_pid_(40.0f, 5.0f, 10.0f);
  control::ConstrainedPID left_flywheel_pid_(30.0f, 0.8f, 0.0f, 3000.0f, 16384.0f);
  control::ConstrainedPID right_flywheel_pid_(30.0f, 0.8f, 0.0f, 3000.0f, 16384.0f);
  bool flywheel_enabled = false;

  // Yaw PD gains from connected component version (smooth, robust, zero-windup)
  const float YAW_KP = 20000.0f;
  const float YAW_KD = 800.0f;        // per rad/s of measured yaw rate
  const float YAW_MAX_LEAD = 0.6f;    // max rad the target may lead the measured yaw
  const float YAW_RATE_ALPHA = 0.2f;  // low-pass on measured rate

  /* Use SetZeroPos if you want to set current motor position as zero position. If uncommented, the
   * zero position is the zero position set before */
  osDelay(1000);
  chassis_motor[0]->SetZeroPos();
  chassis_motor[0]->MotorEnable();
  chassis_motor[1]->SetZeroPos();
  chassis_motor[1]->MotorEnable();
  chassis_motor[2]->SetZeroPos();
  chassis_motor[2]->MotorEnable();
  chassis_motor[3]->SetZeroPos();
  chassis_motor[3]->MotorEnable();
  gimbal_motor->MotorEnable();
  osDelay(100);

  bool enabled = false;
  float yaw_target = INS_angle[0] + yaw_motor->GetTheta() - YAW_MOTOR_OFFSET;
  float yaw_feedback = 0.0f;
  float prev_yaw_measured = yaw_target;
  float yaw_rate_filtered = 0.0f;
  float gimbal_yaw_measured_in_field_reference = yaw_target;

  uint32_t last_ms = HAL_GetTick();
  int print_count = 0;
  float gimbal_pos = clip<float>(gimbal_motor->GetTheta(), -PI/4, PI/4);
  float desired_yaw = 0.0f;
  float desired_pitch = 0.0f;
  bool autoaim_active = false;
  int feeder_dbg = 0;
  int feeder_jam_detect_count = 0;
  int feeder_jam_pause_count = 0;
  int feeder_jam_retries = 0;
  bool feeder_jam_locked = false;

  while (true) {
    BoardStateCanMainTaskStep();
    bool autoaim_requested = (dbus->swr == remote::UP);
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

    
    // disable logic: swl == UP shuts down all chassis, gimbal, feeder, and flywheel motors
    if (dbus->swl == remote::UP) {
      if (enabled) {
        chassis_motor[0]->MotorDisable();
        chassis_motor[1]->MotorDisable();
        chassis_motor[2]->MotorDisable();
        chassis_motor[3]->MotorDisable();
        gimbal_motor->MotorDisable();
        print("Disabled \r\n");
        enabled = false;
        // Nothing of what happened during the shutdown belongs to the next move.
        dbus->DiscardMouseDelta();
      }
      flywheel_enabled = false;
      left_flywheel_pid_.Reset();
      right_flywheel_pid_.Reset();
      flywheel_motor[0]->SetOutput(0);
      flywheel_motor[1]->SetOutput(0);
      feeder_motor->SetOutput(0);
      control::MotorCANBase::TransmitOutput(dji_container, 4);
      osDelay(100);
      continue;
    } else {
      if (!enabled) {
        print("Enabled \r\n");
        chassis_motor[0]->MotorEnable();
        chassis_motor[1]->MotorEnable();
        chassis_motor[2]->MotorEnable();
        chassis_motor[3]->MotorEnable();
        gimbal_motor->MotorEnable();
        gimbal_pos = clip<float>(gimbal_motor->GetTheta(), -PI/4, PI/4);
        yaw_target = INS_angle[0] + yaw_motor->GetTheta() - YAW_MOTOR_OFFSET;
        prev_yaw_measured = yaw_target;
        yaw_rate_filtered = 0.0f;
        osDelay(100);
        // Drop the arm-up window: everything that arrived while the motors were being
        // enabled is cleared here, and the displacement taken at the top of this
        // iteration came from the still disabled period, so it is dropped too.
        dbus->DiscardMouseDelta();
        mouse_delta = {};
        enabled = true;
      }
    }

    // Auto-spin logic on swr == DOWN
    // Higher spin speed of 9.0 rad/s
    // (Remote wheel dial is broken / ignored).
    const float AUTO_SPIN_SPEED = 15.0f;
    float auto_yaw_omega = (dbus->swr == remote::DOWN) ? AUTO_SPIN_SPEED : 0.0f;
    float chassis_yaw_omega_target = auto_yaw_omega;

    // Field-reference gimbal yaw (from connected component version)
    gimbal_yaw_measured_in_field_reference = INS_angle[0] + yaw_motor->GetTheta() - YAW_MOTOR_OFFSET;

    // Keyboard linear translation (WASD) with optional Shift boost:
    // W/S: forward/backward, A/D: strafe left/right (in field/gimbal reference frame)
    float kb_speed = dbus->keyboard.bit.SHIFT ? 25.0f : 15.0f;
    float kb_y = 0.0f;
    float kb_x = 0.0f;
    if (dbus->keyboard.bit.W) kb_y += kb_speed;
    if (dbus->keyboard.bit.S) kb_y -= kb_speed;
    if (dbus->keyboard.bit.A) kb_x -= kb_speed;
    if (dbus->keyboard.bit.D) kb_x += kb_speed;

    // Right stick vertical (ch1) is forward/back (y); right stick horizontal (ch0) is strafe (x).
    // Unified with keyboard WASD so both inputs move along the exact same axes.
    float stick_y = clip<float>(dbus->ch1 / 660.0f * 30.0f, -30.0f, 30.0f);
    float stick_x = clip<float>(dbus->ch0 / 660.0f * 30.0f, -30.0f, 30.0f);
    float y = stick_y + kb_y;
    float x = stick_x + kb_x;
    y = clip<float>(y, -30.0f, 30.0f);
    x = clip<float>(x, -30.0f, 30.0f);

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
        desired_pitch = clip<float>(gimbal_pos - latest_vision_obs.delta_pitch, -PI/4, PI/4);
        vision_obs_fresh = false;
      }

      if (vision_locked) {
        float yaw_err = atan2f(sinf(desired_yaw - yaw_target), cosf(desired_yaw - yaw_target));
        yaw_target += AUTOAIM_YAW_SMOOTH * yaw_err;
        gimbal_pos += AUTOAIM_PITCH_SMOOTH * (desired_pitch - gimbal_pos);
        gimbal_pos = clip<float>(gimbal_pos, -PI/4, PI/4);
        gimbal_vel = 0.0f;
      } else {
        yaw_target += yaw_omega * dt + mouse_step.yaw;
        gimbal_vel = clip<float>(dbus->ch3 / 660.0 * 30.0, -30, 30);
        gimbal_pos += gimbal_vel * dt + mouse_step.pitch;
        gimbal_pos = clip<float>(gimbal_pos, -PI/4, PI/4);  // existing soft limit
      }
    } else {
      if (autoaim_active) {
        autoaim_active = false;
      }
      vision_obs_fresh = false;

      yaw_target += yaw_omega * dt + mouse_step.yaw;
      gimbal_vel = clip<float>(dbus->ch3 / 660.0 * 30.0, -30, 30);
      gimbal_pos += gimbal_vel * dt + mouse_step.pitch;
      gimbal_pos = clip<float>(gimbal_pos, -PI/4, PI/4);  // existing soft limit
    }

    // Apply constant velocity in opposite direction of chassis spin at the same speed as chassis rotation
    float chassis_spin_rate = CHASSIS_SPIN_RATIO * chassis_yaw_omega_target;
    yaw_target += chassis_spin_rate * dt;

    float delta_yaw = yaw_target - gimbal_yaw_measured_in_field_reference;
    float cos = cosf(delta_yaw);
    float sin = sinf(delta_yaw);
    delta_yaw = atan2f(sin, cos);  // wrap to [-pi, pi]

    // Anti-windup: don't let the target run away from the measured heading when the
    // stick is held (otherwise the error saturates and flips sign at +-pi -> violent snap)
    if (fabsf(delta_yaw) > YAW_MAX_LEAD) {
      delta_yaw = copysignf(YAW_MAX_LEAD, delta_yaw);
      yaw_target = gimbal_yaw_measured_in_field_reference + delta_yaw;
    }

    // PD with derivative on the (filtered) measurement: no kick when yaw_target moves
    float meas_diff = atan2f(sinf(gimbal_yaw_measured_in_field_reference - prev_yaw_measured),
                             cosf(gimbal_yaw_measured_in_field_reference - prev_yaw_measured));
    prev_yaw_measured = gimbal_yaw_measured_in_field_reference;
    yaw_rate_filtered += YAW_RATE_ALPHA * (meas_diff / dt - yaw_rate_filtered);
    yaw_feedback = YAW_KP * delta_yaw - YAW_KD * yaw_rate_filtered;

    float yaw_output_f = clip<float>(yaw_feedback, -25000.0f, 25000.0f);
    int16_t yaw_output = (int16_t)clip<float>(yaw_output_f, -30000.0f, 30000.0f);

    float chasis_yaw_measured_in_field_reference = gimbal_yaw_measured_in_field_reference - INS_angle[0];  // in rad
    float cos_yaw = cosf(chasis_yaw_measured_in_field_reference);
    float sin_yaw = sinf(chasis_yaw_measured_in_field_reference);
    float chassis_x = x * cos_yaw - y * sin_yaw;
    float chassis_y = x * sin_yaw + y * cos_yaw;
    x = chassis_x;
    y = chassis_y;

    // alpha, beta equals to x y rotated by -45 degrees
    float alpha = (x - y) / 1.4142f;
    float beta = (y + x) / 1.4142f;

    // Limit omega to available chassis motor headroom while translating
    float max_omega = abs(30.0f - max(fabsf(alpha), fabsf(beta)));
    chassis_yaw_omega_target = clip<float>(chassis_yaw_omega_target, -max_omega, max_omega);

    vel[0] = -beta + chassis_yaw_omega_target;
    vel[1] = beta + chassis_yaw_omega_target;
    vel[2] = -alpha + chassis_yaw_omega_target;
    vel[3] = alpha + chassis_yaw_omega_target;

    // Referee System Regulation: Chassis Power & 60J Buffer Energy Protection (RMUL Rule 3.3.1.4)
    // If buffer energy falls to 0, referee powers off chassis for 5 seconds.
    // Throttles velocity output proportionally as buffer energy drops below 30 J.
    if (referee && referee->connection_flag_) {
      if (!referee->game_robot_status.power_management_chassis_output) {
        vel[0] = 0.0f;
        vel[1] = 0.0f;
        vel[2] = 0.0f;
        vel[3] = 0.0f;
      } else {
        float buffer_energy = (float)referee->power_heat_data.buffer_energy;
        if (buffer_energy < 30.0f) {
          float pwr_scale = clip<float>(buffer_energy / 30.0f, 0.25f, 1.0f);
          vel[0] *= pwr_scale;
          vel[1] *= pwr_scale;
          vel[2] *= pwr_scale;
          vel[3] *= pwr_scale;
        }
      }
    }

    // Flywheel Control:
    // Left flywheel (0x207): -900 rad/s
    // Right flywheel (0x208): +900 rad/s
    // swl == DOWN enables flywheels; swl != DOWN disables them and applies smooth viscous braking
    const float FLYWHEEL_TARGET_SPEED = 900.0f;
    if (dbus->swl == remote::DOWN) {
      if (!flywheel_enabled) {
        flywheel_enabled = true;
        left_flywheel_pid_.Reset();
        right_flywheel_pid_.Reset();
        print("FLYWHEEL: ON (-900/+900 rad/s)\r\n");
      }
    } else {
      if (flywheel_enabled) {
        flywheel_enabled = false;
        left_flywheel_pid_.Reset();
        right_flywheel_pid_.Reset();
        print("FLYWHEEL: BRAKING TO REST\r\n");
      }
    }

    // Referee System Regulation: If shooter 24V power is cut by referee, brake flywheels
    if (referee && referee->connection_flag_ &&
        !referee->game_robot_status.power_management_shooter_output) {
      if (flywheel_enabled) {
        flywheel_enabled = false;
        left_flywheel_pid_.Reset();
        right_flywheel_pid_.Reset();
      }
    }

    int16_t left_flywheel_output = 0;
    int16_t right_flywheel_output = 0;
    float left_omega = flywheel_motor[0]->GetOmega();
    float right_omega = flywheel_motor[1]->GetOmega();

    if (flywheel_enabled) {
      // Steady running at -900 and +900 rad/s with anti-windup PI (Kd=0 prevents torque ripple & yaw gyro jitter)
      float left_diff = -FLYWHEEL_TARGET_SPEED - left_omega;
      float right_diff = FLYWHEEL_TARGET_SPEED - right_omega;
      left_flywheel_output = left_flywheel_pid_.ComputeConstrainedOutput(left_diff);
      right_flywheel_output = right_flywheel_pid_.ComputeConstrainedOutput(right_diff);
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
    // Feeder triggered by Mouse Left Click or Keyboard F / E / R
    bool feeder_requested = dbus->mouse.l ||
                            dbus->keyboard.bit.F ||
                            dbus->keyboard.bit.E ||
                            dbus->keyboard.bit.R;

    // Referee System Regulation: Barrel Heat Limit & Shooter Power Protection (RMUL Rule 3.3.1.3)
    // Each 17mm projectile adds 10 heat.
    // Exceeding limit locks launcher and blinds FPV until heat = 0.
    // Exceeding limit + 100 permanently locks launcher for the entire round.
    if (referee && referee->connection_flag_) {
      uint16_t heat_limit = referee->game_robot_status.shooter_barrel_heat_limit;
      uint16_t current_heat = referee->power_heat_data.shooter_17mm_barrel_heat;
      if (heat_limit > 0 && (current_heat + 15 >= heat_limit)) {
        feeder_requested = false;  // Suppress feeding to avoid overheat penalty
      }
      if (!referee->game_robot_status.power_management_shooter_output) {
        feeder_requested = false;  // 24V shooter power cut by referee
      }
    }

    float feeder_target_velocity = 0.0f;
    if (feeder_requested) {
      feeder_target_velocity = 160.0f;
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


    chassis_motor[0]->SetOutput(vel[0]);
    chassis_motor[1]->SetOutput(vel[1]);
    chassis_motor[2]->SetOutput(vel[2]);
    chassis_motor[3]->SetOutput(vel[3]);
    // DM4310 in POS_VEL mode: 2nd argument is velocity limit (rad/s) for profile position tracking.
    // Must be positive non-zero so motor tracks gimbal_pos whether commanded by mouse, stick, or autoaim.
    const float GIMBAL_PITCH_TRACKING_VEL = 20.0f;
    gimbal_motor->SetOutput(gimbal_pos, GIMBAL_PITCH_TRACKING_VEL);
    flywheel_motor[0]->SetOutput(left_flywheel_output);
    flywheel_motor[1]->SetOutput(right_flywheel_output);
    feeder_motor->SetOutput(feeder_output);
    yaw_motor->SetOutput(yaw_output);
    control::MotorDM3519::TransmitOutput(chassis_motor, 4);
    control::Motor4310::TransmitOutput(gimbal_container, 1);
    control::MotorCANBase::TransmitOutput(dji_container, 4);

    // Throttled debug print every 500ms for heading and gimbal alignment verification
    if (++print_count >= 100) {
      print_count = 0;
      float imu_deg = RAD2DEG(INS_angle[0]);
      float field_deg = RAD2DEG(gimbal_yaw_measured_in_field_reference);
      float target_deg = RAD2DEG(yaw_target);
      float muzzle_deg = RAD2DEG(yaw_motor->GetTheta() - YAW_MOTOR_OFFSET);
      print("[CONN] IMU: %5.1f | Field: %5.1f | Target: %5.1f | MuzzleRel: %5.1f deg | spin: %.1f | out: %d | err: %5.1f\r\n",
            imu_deg, field_deg, target_deg, muzzle_deg, chassis_yaw_omega_target, yaw_output, RAD2DEG(delta_yaw));
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
