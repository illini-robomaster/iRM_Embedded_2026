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
*  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
*                                                                          *
****************************************************************************/

/**
 * @brief Omni robot control for DM_MC_02 board
 * 
 * Hardware differences from TypeC:
 * - Uses FDCAN instead of CAN (hfdcan1, hfdcan2)
 * - Print via USB (typeC)
 * - DBUS via UART5 (huart5)
 * - IMU via SPI2 (hspi2)
 */

/*============================================================================
 *                    TEST CONFIGURATION MACROS
 *============================================================================
 * Comment/uncomment macros below to enable/disable subsystems for testing.
 * 
 * ┌─────────────────┬──────────────────────────────────────────────────────┐
 * │     Macro       │              Components Affected                     │
 * ├─────────────────┼──────────────────────────────────────────────────────┤
 * │ ENABLE_CHASSIS  │ - motor[0-3] (MotorDM3519 chassis wheels)            │
 * │                 │ - Chassis velocity calculation (alpha, beta, vel[])  │
 * │                 │ - Field-oriented drive transformation                │
 * │                 │ - Chassis spin (swl UP)                              │
 * │                 │ - MotorDM3519::TransmitOutput                        │
 * ├─────────────────┼──────────────────────────────────────────────────────┤
 * │ ENABLE_GIMBAL   │ - yaw_motor (Motor6020)                              │
 * │                 │ - pitch_motor (Motor4310)                            │
 * │                 │ - Gimbal yaw PID control                             │
 * │                 │ - Gimbal pitch MIT control                           │
 * │                 │ - Motor4310::TransmitOutput                          │
 * │                 │ - yaw_motor in DJI motor transmit                    │
 * ├─────────────────┼──────────────────────────────────────────────────────┤
 * │ ENABLE_SHOOTER  │ - flywheel_motor[0-1] (Motor3508)                    │
 * │                 │ - feeder_motor (Motor3508)                           │
 * │                 │ - Flywheel velocity PID control                      │
 * │                 │ - Feeder velocity PID control                        │
 * │                 │ - Shooter motors in DJI motor transmit               │
 * ├─────────────────┼──────────────────────────────────────────────────────┤
 * │ ENABLE_IMU      │ - BMI088 IMU initialization                          │
 * │                 │ - IMU task thread                                    │
 * │                 │ - MahonyAHRS quaternion/angle updates                │
 * │                 │ - INS_angle[] (yaw, pitch, roll)                     │
 * ├─────────────────┼──────────────────────────────────────────────────────┤
 * │ ENABLE_CAN      │ - bsp::CAN (hfdcan1) FDCAN bus initialization        │
 * │                 │ - Required by all motor types                        │
 * ├─────────────────┼──────────────────────────────────────────────────────┤
 * │ ENABLE_DBUS     │ - remote::DBUS (huart5) RC receiver                  │
 * │                 │ - All joystick/switch inputs                         │
 * │                 │ - Robot enable/disable logic                         │
 * └─────────────────┴──────────────────────────────────────────────────────┘
 *
 * NOTE: When ALL macros are disabled, runs minimal USB test mode.
 *       Enable peripherals one-by-one to find which breaks USB.
 *============================================================================*/

#define ENABLE_CHASSIS   // Comment to disable chassis motors
#define ENABLE_GIMBAL    // Comment to disable gimbal motors
#define ENABLE_SHOOTER   // Comment to disable shooter motors
#define ENABLE_IMU       // Comment to disable IMU (affects FOC and gimbal)
#define ENABLE_CAN       // Comment to disable CAN/FDCAN
#define ENABLE_DBUS      // Comment to disable DBUS remote

#include "main.h"
#include "bsp_usb.h" 
#include "bsp_print.h"
#include "bsp_imu.h"
#include "cmsis_os.h"
#include "motor.h"
#include "spi.h"
#include "dbus.h"
#include "MahonyAHRS.h"
#include <cmath>

#define RX_SIGNAL (1 << 0)

// Convert radians to degrees and vice versa
#define RAD2DEG(x) ((x) * 180.0f / M_PI)
#define DEG2RAD(x) ((x) * M_PI / 180.0f)

#ifdef ENABLE_IMU
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

// yaw, pitch, roll (always needed for fallback)
static float INS_angle[3] = {0.0f, 0.0f, 0.0f};
#endif  // ENABLE_IMU

#ifdef ENABLE_IMU
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

  while (true) {
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {
      // Read IMU data
      bmi088->Read(gyro, accel, &temp);
      
      // Convert gyro from deg/s to rad/s for MahonyAHRS
      float gx_rad = DEG2RAD(gyro[0]);
      float gy_rad = DEG2RAD(gyro[1]);
      float gz_rad = DEG2RAD(gyro[2]);
      
      // Update AHRS
      MahonyAHRSupdateIMU(quat, gx_rad, gy_rad, gz_rad, 
                         accel[0], accel[1], accel[2]);
      
      // Calculate Euler angles from quaternion
      INS_angle[0] = atan2f(2.0f * (quat[0] * quat[3] + quat[1] * quat[2]),
                           1.0f - 2.0f * (quat[2] * quat[2] + quat[3] * quat[3]));  // yaw
      INS_angle[1] = asinf(2.0f * (quat[0] * quat[2] - quat[3] * quat[1]));          // pitch
      INS_angle[2] = atan2f(2.0f * (quat[0] * quat[1] + quat[2] * quat[3]),
                           1.0f - 2.0f * (quat[1] * quat[1] + quat[2] * quat[2]));  // roll
    }
  }
}
#endif  // ENABLE_IMU

void RM_RTOS_Threads_Init(void) {
#ifdef ENABLE_IMU
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
#endif  // ENABLE_IMU
}

// Global peripherals - MC02 uses FDCAN
#ifdef ENABLE_CAN
bsp::CAN* can = nullptr;
#endif  // ENABLE_CAN

#ifdef ENABLE_DBUS
remote::DBUS* dbus = nullptr;
#endif  // ENABLE_DBUS

#ifdef ENABLE_CHASSIS
control::MotorDM3519* motor[4];
#endif  // ENABLE_CHASSIS

#ifdef ENABLE_SHOOTER
control::Motor3508* flywheel_motor[2];
control::Motor3508* feeder_motor;
#endif  // ENABLE_SHOOTER

#ifdef ENABLE_GIMBAL
control::Motor6020* yaw_motor;
control::Motor4310* pitch_motor;
#endif  // ENABLE_GIMBAL

#ifdef ENABLE_CHASSIS
float original_max_output_vel_rad_per_s = 395/60.0f*2*PI; //395 rpm = 41.36 rad/s
float original_gear_ratio = 3591.0f/187.0f; 
float new_gear_ratio = 268.0f/17.0f; 
#endif  // ENABLE_CHASSIS

#ifdef ENABLE_GIMBAL
const float YAW_MOTOR_OFFSET = 4.54f; // in rad
const float PITCH_PHYSICAL_OFFSET = 33.0f * PI / 180.0f; // 0 in encoder is 33 degrees in physical position, also physical maximum
const float PITCH_PHYSICAL_MIN = -24.0f * PI / 180.0f; // -24 degrees in physical position
#endif  // ENABLE_GIMBAL

void RM_RTOS_Init() {
  // Initialize USB - no HAL_Delay here! USB enumeration is interrupt-driven
  // and HAL_Delay before RTOS scheduler starts can interfere with it.
  print_use_usb();
  
#ifdef ENABLE_CAN
  // MC02 uses FDCAN instead of CAN
  can = new bsp::CAN(&hfdcan1, 0);
#endif  // ENABLE_CAN

  /* rx_id = Master id
   * tx_id = CAN id
   * mode:
   *  MIT: MIT mode
   *  POS_VEL: position-velocity mode
   *  VEL: velocity mode  */

  /* Make sure motor is set to the correct mode (in helper tool). Otherwise, motor won't start */
#ifdef ENABLE_CHASSIS
  // DM3519 chassis motors - FDCAN is configured as classical CAN mode
  motor[0] = new control::MotorDM3519(can, 0x10, 0x11, control::VEL); // front right
  motor[1] = new control::MotorDM3519(can, 0x12, 0x13, control::VEL); // back left
  motor[2] = new control::MotorDM3519(can, 0x18, 0x19, control::VEL); // back right
  motor[3] = new control::MotorDM3519(can, 0x16, 0x17, control::VEL); // front left
#endif  // ENABLE_CHASSIS

#ifdef ENABLE_GIMBAL
  yaw_motor = new control::Motor6020(can, 0x205);
  pitch_motor = new control::Motor4310(can, 0x0D, 0x0C, control::MIT);
#endif  // ENABLE_GIMBAL
  
#ifdef ENABLE_DBUS
  // MC02 uses UART5 for DBUS
  dbus = new remote::DBUS(&huart5);
#endif  // ENABLE_DBUS

#ifdef ENABLE_SHOOTER
  // Shooter motors
  flywheel_motor[0] = new control::Motor3508(can, 0x207); // left flywheel
  flywheel_motor[1] = new control::Motor3508(can, 0x208); // right flywheel
  feeder_motor = new control::Motor3508(can, 0x206);
#endif  // ENABLE_SHOOTER

#ifdef ENABLE_IMU
  // IMU initialization - MC02 uses SPI2
  print("Initializing BMI088...\r\n");
  
  bmi088 = new bsp::BMI088(&hspi2, 
                           CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin,
                           CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);
  
  if (!bmi088->IsReady()) {
    print("BMI088 Init FAILED!\r\n");
  } else {
    print("BMI088 Init OK!\r\n");
  }
  
  // Create interrupt handlers
  gyro_int = new IMU_GYRO_INT();
  accel_int = new IMU_ACCEL_INT();
#endif  // ENABLE_IMU
}

void RM_RTOS_Default_Task(const void* args) {
  /* press reset if no response */
  UNUSED(args);
  print("=== MC02 Omni Robot Control ===\r\n");
  // ============ MINIMAL USB TEST MODE ============
  // When all peripherals are disabled, just test USB print
  #if !defined(ENABLE_CHASSIS) && !defined(ENABLE_GIMBAL) && !defined(ENABLE_SHOOTER) && !defined(ENABLE_CAN) && !defined(ENABLE_DBUS) && !defined(ENABLE_IMU)
   
  uint32_t counter = 0;
  while (true) {
    set_cursor(0, 0);
    clear_screen();
    print("=== MC02 USB TEST ===\r\n");
    print("Counter: %lu\r\n", counter++);
    print("Tick: %lu\r\n", HAL_GetTick());
    osDelay(500);
  }
  
  #else
  // ============ NORMAL OPERATION MODE ============

#ifdef ENABLE_CHASSIS
  control::MotorDM3519* motors[] = {motor[0], motor[1], motor[2], motor[3]};
#endif  // ENABLE_CHASSIS

#ifdef ENABLE_GIMBAL
  control::Motor4310* pitch_motors[] = {pitch_motor};
  float pid_params[3] = {20000.0, 1.0, 10.0};
  control::ConstrainedPID pid_(pid_params, 30000, 30000);
#endif  // ENABLE_GIMBAL

#ifdef ENABLE_SHOOTER
  control::PIDController left_flywheel_pid_(40.0, 15.0, 30.0);
  control::PIDController right_flywheel_pid_(40.0, 15.0, 30.0);
  control::PIDController feeder_pid_(40.0, 5.0, 10.0);
#endif  // ENABLE_SHOOTER


#ifdef ENABLE_DBUS
  // Wait for switch to start
  while(dbus->swr == remote::DOWN){
    osDelay(10);
  }
#endif  // ENABLE_DBUS

  osDelay(1000);

  
#ifdef ENABLE_GIMBAL
  // Enable motors
  // Note: Motor4310 methods may need adjustment for FDCAN
  pitch_motor->MotorEnable();
  float pitch_physical = pitch_motor->GetTheta() + PITCH_PHYSICAL_OFFSET;
  osDelay(100);
#endif  // ENABLE_GIMBAL

  bool enabled = false;
#ifdef ENABLE_SHOOTER
  bool flywheel_enabled = false;
#endif  // ENABLE_SHOOTER

#ifdef ENABLE_GIMBAL
  float yaw_target = INS_angle[0] + yaw_motor->GetTheta() - YAW_MOTOR_OFFSET;
  int16_t command = 0;
  float gimbal_yaw_measured_in_field_reference = 0.0f;
#endif  // ENABLE_GIMBAL

#ifdef ENABLE_SHOOTER
  float left_target_velocity = 0;
  float right_target_velocity = 0;
  float feeder_target_velocity = 0;
#endif  // ENABLE_SHOOTER

  while (true) {
    set_cursor(0, 0);
    clear_screen();

#ifdef ENABLE_DBUS
    // Disable logic
    if(dbus->swr == remote::DOWN){
      if(enabled){
#ifdef ENABLE_CHASSIS
        // Disable chassis motors
        for (int i = 0; i < 4; i++) {
          motor[i]->MotorDisable();
        }
#endif  // ENABLE_CHASSIS
#ifdef ENABLE_GIMBAL
        // Disable gimbal motors
        pitch_motor->MotorDisable();
        pitch_physical = PITCH_PHYSICAL_OFFSET;
#endif  // ENABLE_GIMBAL
        enabled = false;

#ifdef ENABLE_SHOOTER
        // Disable flywheel
        left_target_velocity = 0;
        right_target_velocity = 0;
        feeder_target_velocity = 0;
#endif  // ENABLE_SHOOTER
      }
      osDelay(100);
      continue;
    } else {
      if(!enabled){
#ifdef ENABLE_CHASSIS
        // Re-enable chassis motors
        osDelay(500);
        for (int i = 0; i < 4; i++) {
          osDelay(100);
          motor[i]->MotorEnable();
        }
        osDelay(100);
#endif  // ENABLE_CHASSIS
#ifdef ENABLE_GIMBAL
        pitch_motor->MotorEnable();
        osDelay(100);
#endif  // ENABLE_GIMBAL
        enabled = true;
      }

#ifdef ENABLE_SHOOTER
      if(dbus->swr == remote::UP){
        if(!flywheel_enabled){
          left_target_velocity = -900;
          right_target_velocity = 900;
          flywheel_enabled = true;
        }
      } else { // MID
        if(flywheel_enabled){
          left_target_velocity = 0;
          right_target_velocity = 0;
          flywheel_enabled = false;
        }
      }
#endif  // ENABLE_SHOOTER
    }

#ifdef ENABLE_SHOOTER
    if(dbus->swl == remote::DOWN){
      feeder_target_velocity = 80.0f;
    } else {
      feeder_target_velocity = 0.0f;
    }

    // Shooter control
    float left_diff = flywheel_motor[0]->GetOmegaDelta(left_target_velocity);
    float right_diff = flywheel_motor[1]->GetOmegaDelta(right_target_velocity);
    float feeder_diff = feeder_motor->GetOmegaDelta(feeder_target_velocity);
    int16_t left_output = left_flywheel_pid_.ComputeConstrainedOutput(left_diff);
    int16_t right_output = right_flywheel_pid_.ComputeConstrainedOutput(right_diff);
    int16_t feeder_output = feeder_pid_.ComputeConstrainedOutput(feeder_diff);

    flywheel_motor[0]->SetOutput(left_output);
    flywheel_motor[1]->SetOutput(right_output);
    feeder_motor->SetOutput(feeder_output);
    print("L: %.2f R: %.2f F: %.2f\r\n", flywheel_motor[0]->GetOmega(), flywheel_motor[1]->GetOmega(), feeder_motor->GetOmega());
#endif  // ENABLE_SHOOTER
    print("dial: %d\r\n", dbus->wheel);

    // Read DBUS joystick inputs
    float y = -clip<float>(dbus->ch0 / 660.0 * 30.0, -30, 30); // forward
    float x = clip<float>(dbus->ch1 / 660.0 * 30.0, -30, 30); // left
    float yaw_omega = clip<float>(dbus->ch2 / 660.0 * 30.0, -30, 30); // yaw (for both chassis and gimbal)

#ifdef ENABLE_GIMBAL
    float pitch_omega = clip<float>(dbus->ch3 / 660.0 * 15.0, -15, 15); // pitch

    // Gimbal control
    gimbal_yaw_measured_in_field_reference = INS_angle[0] + yaw_motor->GetTheta() - YAW_MOTOR_OFFSET;
    yaw_target += yaw_omega * 0.01f;
    pitch_physical += pitch_omega / 200;
    pitch_physical = clip<float>(pitch_physical, PITCH_PHYSICAL_MIN, PITCH_PHYSICAL_OFFSET);

    float delta_yaw = yaw_target - gimbal_yaw_measured_in_field_reference;
    float cos_val = cosf(delta_yaw);
    float sin_val = sinf(delta_yaw);
    delta_yaw = atan2f(sin_val, cos_val); // wrap to [-pi, pi]
    command = pid_.ComputeOutput(delta_yaw);

    float chasis_yaw_measured_in_field_reference = gimbal_yaw_measured_in_field_reference - INS_angle[0];
#else
    float chasis_yaw_measured_in_field_reference = 0.0f;  // No field-oriented control without gimbal
#endif  // ENABLE_GIMBAL

#ifdef ENABLE_CHASSIS
    // Chassis control
    float vel[4];

    // Rotate x,y according to robot's heading
    float temp_x = x * cosf(chasis_yaw_measured_in_field_reference) - y * sinf(chasis_yaw_measured_in_field_reference);
    float temp_y = x * sinf(chasis_yaw_measured_in_field_reference) + y * cosf(chasis_yaw_measured_in_field_reference);
    x = temp_x;
    y = temp_y;

    // Alpha, beta equals to x y rotated by -45 degrees
    float alpha = (x - y) / 1.4142f;
    float beta = (y + x) / 1.4142f;

    // Use ch2 (yaw_omega) for chassis turning, swl UP for auto-spin
    float max_omega = abs(30.0f - max(fabsf(alpha), fabsf(beta)));
    float chassis_yaw_omega_target = yaw_omega;  // Use joystick ch2 for turning
    if (dbus->swl == remote::UP) {
      chassis_yaw_omega_target = 15.0f;  // Auto-spin mode overrides joystick
    }
    chassis_yaw_omega_target = clip<float>(chassis_yaw_omega_target, -max_omega, max_omega);

    vel[0] = -beta + chassis_yaw_omega_target;
    vel[1] = beta + chassis_yaw_omega_target;
    vel[2] = -alpha + chassis_yaw_omega_target;
    vel[3] = alpha + chassis_yaw_omega_target;

    motor[0]->SetOutput(vel[0]);
    motor[1]->SetOutput(vel[1]);
    motor[2]->SetOutput(vel[2]);
    motor[3]->SetOutput(vel[3]);
    
    control::MotorDM3519::TransmitOutput(motors, 4);
#endif  // ENABLE_CHASSIS

#ifdef ENABLE_GIMBAL
    float yaw_kF = 400.0f;
  #ifdef ENABLE_CHASSIS
    yaw_motor->SetOutput(command + static_cast<int16_t>(chassis_yaw_omega_target * yaw_kF));
  #else
    yaw_motor->SetOutput(command);
  #endif  // ENABLE_CHASSIS

    float pitch_rotor = pitch_physical - PITCH_PHYSICAL_OFFSET;
    pitch_motor->SetOutput(pitch_rotor, pitch_omega, 30, 0.5, 0);
    
    control::Motor4310::TransmitOutput(pitch_motors, 1);
#endif  // ENABLE_GIMBAL

    // Transmit DJI motors (yaw + shooter)
#if defined(ENABLE_GIMBAL) && defined(ENABLE_SHOOTER)
    control::MotorCANBase* dji_motors[] = {yaw_motor, feeder_motor, flywheel_motor[0], flywheel_motor[1]};
    control::Motor3508::TransmitOutput(dji_motors, 4);
#elif defined(ENABLE_GIMBAL)
    control::MotorCANBase* dji_motors[] = {yaw_motor};
    control::Motor6020::TransmitOutput(dji_motors, 1);
#elif defined(ENABLE_SHOOTER)
    control::MotorCANBase* dji_motors[] = {feeder_motor, flywheel_motor[0], flywheel_motor[1]};
    control::Motor3508::TransmitOutput(dji_motors, 3);
#endif

#endif  // ENABLE_DBUS
    print("Tick: %lu\r\n", HAL_GetTick());
    osDelay(10);
  }
  
  #endif  // Minimal USB test mode
}
