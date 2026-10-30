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
#include "cmsis_os.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "spi.h"

#define RX_SIGNAL (1 << 0)

// Blocking UART prints stall the control loop (motors hold the last command) -> jerks.
#define EOH_DEBUG_PRINT 0

// Convert radians to degrees and vice versa
#define RAD2DEG(x) ((x) * 180.0f / M_PI)
#define DEG2RAD(x) ((x) * M_PI / 180.0f)

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
    osThreadFlagsSet(imuTaskHandle, RX_SIGNAL);
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
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {
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

void RM_RTOS_Threads_Init(void) {
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
}

// Global peripherals
bsp::CAN* can = nullptr;
control::MotorDM3519* chassis_motor[4];
control::Motor4310* gimbal_motor;
control::Motor6020* yaw_motor;
control::Motor3508* flywheel_motor[2];
control::Motor3508* feeder_motor;

remote::DBUS* dbus = nullptr;

const float YAW_MOTOR_OFFSET = 4.54f;  // in rad

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
    state.bullet_speed = 0.0f;  // No referee input on this target yet.
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

  // Yaw PD gains (output units are motor command counts)
  const float YAW_KP = 20000.0f;
  const float YAW_KD = 800.0f;        // per rad/s of measured yaw rate
  const float YAW_MAX_LEAD = 0.6f;    // max rad the target may lead the measured yaw
  const float YAW_RATE_ALPHA = 0.2f;  // low-pass on measured rate
  // Use standard CMSIS PIDController with gentle gains for bare motor
  // Clean anti-windup PI state for bare feeder motor

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
  gimbal_motor->SetZeroPos();
  osDelay(100);

  bool enabled = false;
  float yaw_target = INS_angle[0] + yaw_motor->GetTheta() - YAW_MOTOR_OFFSET;
  float yaw_feedback;
  float prev_yaw_measured = 0.0f;
  float yaw_rate_filtered = 0.0f;
  float gimbal_yaw_measured_in_field_reference = 0.0f;
  prev_yaw_measured = yaw_target;

  uint32_t last_ms = HAL_GetTick();
  int print_count = 0;
  float gimbal_pos = 0;
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
    
    // Real loop time (UART printing can stretch the loop well past 5 ms)
    uint32_t now_ms = HAL_GetTick();
    float dt = clip<float>((now_ms - last_ms) / 1000.0f, 0.001f, 0.05f);
    last_ms = now_ms;

    
    UNUSED(print_count);
    // Throttled debug print (a blocking print every loop jitters the control loop)
#if EOH_DEBUG_PRINT
    if (++print_count >= 40) {
      print_count = 0;
      set_cursor(0, 0);
      clear_screen();
      print("CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d \r\n", dbus->ch0, dbus->ch1, dbus->ch2, dbus->ch3);
    }
#endif

    // disable logic
    if (dbus->swr == remote::DOWN) {
      if (enabled) {
        chassis_motor[0]->MotorDisable();
        chassis_motor[1]->MotorDisable();
        chassis_motor[2]->MotorDisable();
        chassis_motor[3]->MotorDisable();
        print("Disabled \r\n");
        enabled = false;
      }
      osDelay(100);
      continue;
    } else {
      if (!enabled) {
        print("Enabled \r\n");
        chassis_motor[0]->MotorEnable();
        chassis_motor[1]->MotorEnable();
        chassis_motor[2]->MotorEnable();
        chassis_motor[3]->MotorEnable();
        osDelay(100);
        enabled = true;
      }
    }

    // chassis
    gimbal_yaw_measured_in_field_reference = INS_angle[0] + yaw_motor->GetTheta() - YAW_MOTOR_OFFSET;

    float y = -clip<float>(dbus->ch0 / 660.0 * 30.0, -30, 30);                 // forward
    float x = clip<float>(dbus->ch1 / 660.0 * 30.0, -30, 30);                  // left;
    float yaw_omega = clip<float>(-(dbus->ch2 + 16) / 660.0 * 10.0, -10, 10);  // yaw (slower)

    bool vision_locked = (now_ms - vision_last_obs_ms <= 120);
    float gimbal_vel = 0.0f;

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
        yaw_target += yaw_omega * dt;
        gimbal_vel = clip<float>(dbus->ch3 / 660.0 * 30.0, -30, 30);
        gimbal_pos += gimbal_vel * dt;
        gimbal_pos = clip<float>(gimbal_pos, -PI/4, PI/4);
      }
    } else {
      if (autoaim_active) {
        autoaim_active = false;
      }
      vision_obs_fresh = false;

      yaw_target += yaw_omega * dt;
      gimbal_vel = clip<float>(dbus->ch3 / 660.0 * 30.0, -30, 30);
      gimbal_pos += gimbal_vel * dt;
      gimbal_pos = clip<float>(gimbal_pos, -PI/4, PI/4);
    }

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

    float chasis_yaw_measured_in_field_reference = gimbal_yaw_measured_in_field_reference - INS_angle[0];  // in rad

    // rotate the x,y according to the robot's heading
    float temp_x = x * cosf(chasis_yaw_measured_in_field_reference) - y * sinf(chasis_yaw_measured_in_field_reference);
    float temp_y = x * sinf(chasis_yaw_measured_in_field_reference) + y * cosf(chasis_yaw_measured_in_field_reference);
    x = temp_x;
    y = temp_y;

    // alpha, beta equals to x y rotated by -45 degrees
    float alpha = (x - y) / 1.4142f;
    float beta = (y + x) / 1.4142f;

    // max output for omega while ensure translation
    float max_omega = abs(30.0f - max(fabsf(alpha), fabsf(beta)));
    float auto_yaw_omega = 0.0f; //dbus->swl == remote::UP ? 15.0f : 0.0f;
    float chassis_yaw_omega_target = auto_yaw_omega;
    chassis_yaw_omega_target = clip<float>(chassis_yaw_omega_target, -max_omega, max_omega);

    vel[0] = -beta + chassis_yaw_omega_target;
    vel[1] = beta + chassis_yaw_omega_target;
    vel[2] = -alpha + chassis_yaw_omega_target;
    vel[3] = alpha + chassis_yaw_omega_target;

    // Robust 80 rad/s (~760 RPM) Feedforward + Proportional Control:
    // Uses 1100 counts base feedforward + 25 * error.
    // Clamped strictly to [200, 3500]:
    // - Prevents +/-16384 full-current saturation and 900 rad/s wild oscillation.
    // - At omega=0, delivers 3100 counts breakaway kick immediately.
    // - In steady state, locks right onto 80 rad/s with ~1100 counts.
    float feeder_target_velocity = 0.0f;
    if (dbus->swl == remote::DOWN) {
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
    gimbal_motor->SetOutput(gimbal_pos, gimbal_vel);
    flywheel_motor[0]->SetOutput(0);
    flywheel_motor[1]->SetOutput(0);
    feeder_motor->SetOutput(feeder_output);
    float yaw_kF = 400.0f;
    float yaw_output_f = clip<float>(yaw_feedback, -25000.0f, 25000.0f) + chassis_yaw_omega_target * yaw_kF;
    int16_t yaw_output = (int16_t)clip<float>(yaw_output_f, -30000.0f, 30000.0f);

    // print("yaw_output: %-7.2f", yaw_output);  // blocking UART print every loop stalls control
    
    yaw_motor->SetOutput(yaw_output);
    control::MotorDM3519::TransmitOutput(chassis_motor, 4);
    control::Motor4310::TransmitOutput(gimbal_container, 1);
    control::MotorCANBase::TransmitOutput(dji_container, 4);

    osDelay(5);
  }
}
