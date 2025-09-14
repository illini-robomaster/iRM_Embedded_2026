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

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "math.h"
#include "motor.h"
#include "utils.h"

#define KEY_GPIO_GROUP GPIOA
#define KEY_GPIO_PIN GPIO_PIN_0

#define IDLE_THROTTLE 1500
#define TIM_CLOCK_FREQ 1000000
#define MOTOR_OUT_FREQ 50

#define LOADER_FEED_MOTOR_CONTACT_OUTPUT 660
#define LOADER_FEED_MOTOR_RELEASE_OUTPUT -1000

#define LOADER_SLIDE_MOTOR_DOWN_OUTPUT -20
#define LOADER_SLIDE_MOTOR_UP_OUTPUT 530

#define LOADER_SLIDE_MOTOR_PWM_CHANNEL 2
#define LOADER_FEED_MOTOR_PWM_CHANNEL 3

bsp::CAN* can1 = NULL;
bsp::GPIO* key = nullptr;
control::MotorCANBase* load_motor = NULL;

control::MotorPWMBase* loader_slide_motor;
control::MotorPWMBase* loader_feed_motor;

remote::DBUS* dbus = nullptr;

// Cascade PID controllers
control::ConstrainedPID* position_pid = nullptr;  // Position (outer loop)
control::ConstrainedPID* velocity_pid = nullptr;  // Velocity (inner loop)

// PWM state control
typedef enum {
  LOADER_STATE_RELEASE_UP = 0,   // Feed motor: release, Slide motor: up
  LOADER_STATE_CONTACT_DOWN = 1  // Feed motor: contact, Slide motor: down
} loader_state_t;

loader_state_t current_loader_state = LOADER_STATE_RELEASE_UP;

// State machine for ch2_right_trigger sequence
typedef enum {
  SEQUENCE_IDLE = 0,
  SEQUENCE_MOTOR_POSITION,          // First spin PI/3 - position motor for loading
  SEQUENCE_LOADER_PREPARE,          // Release + Up - prepare loader mechanism
  SEQUENCE_LOADER_GRAB,             // Contact + Up - grab dart with feed motor
  SEQUENCE_LOADER_LOAD,             // Contact + Down - load dart into position
  SEQUENCE_LOADER_EJECT,            // Release + Up - eject mechanism back up
  SEQUENCE_LOADER_RESET,            // Release + Down - reset to starting position
  SEQUENCE_MOTOR_ADVANCE_AND_RESET  // Second spin PI/3 - advance to next position
} sequence_state_t;

sequence_state_t current_sequence_state = SEQUENCE_IDLE;
uint32_t sequence_start_time = 0;

int16_t command = 0;

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  can1 = new bsp::CAN(&hcan1, true);
  load_motor = new control::Motor6020(can1, 0x207);
  dbus = new remote::DBUS(&huart3);

  loader_slide_motor = new control::MotorPWMBase(&htim1, LOADER_SLIDE_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, IDLE_THROTTLE);
  loader_feed_motor = new control::MotorPWMBase(&htim1, LOADER_FEED_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, IDLE_THROTTLE);

  // Initialize cascade PID controllers
  // Position PID (outer loop) - tuned for position control
  float position_pid_params[3] = {30.0, 0.0, 0.6};                          // Kp, Ki, Kd for position
  position_pid = new control::ConstrainedPID(position_pid_params, 0, 2.5);  // Low max_out for velocity reference

  // Velocity PID (inner loop) - tuned for velocity control
  float velocity_pid_params[3] = {1500.0, 10.0, 3.0};                             // Kp, Ki, Kd for velocity
  velocity_pid = new control::ConstrainedPID(velocity_pid_params, 10000, 30000);  // Higher limits for motor output
  loader_feed_motor->SetOutput(LOADER_FEED_MOTOR_RELEASE_OUTPUT);
  loader_slide_motor->SetOutput(LOADER_SLIDE_MOTOR_DOWN_OUTPUT);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  print("ok!\r\n");

  osDelay(500);  // DBUS initialization needs time

  float target_angle = 0.0;
  control::MotorCANBase* motors[] = {load_motor};

  // Edge detectors for dbus channels
  BoolEdgeDetector ch2_left_trigger(false);   // For PWM state control
  BoolEdgeDetector ch2_right_trigger(false);  // For target angle control

  while (true) {
    // Cascade PID Control Implementation
    // Step 1: Position PID (outer loop) - calculates desired velocity
    float position_error = load_motor->GetThetaDelta(target_angle);
    float target_velocity = position_pid->ComputeOutput(position_error);

    // Step 2: Velocity PID (inner loop) - calculates motor command
    float velocity_error = load_motor->GetOmegaDelta(target_velocity);
    command = velocity_pid->ComputeConstrainedOutput(velocity_error);

    // Update edge detectors
    ch2_left_trigger.input(dbus->ch2 < -300);
    ch2_right_trigger.input(dbus->ch2 > 300);

    if (ch2_left_trigger.posEdge()) {
      loader_feed_motor->SetOutput(LOADER_FEED_MOTOR_RELEASE_OUTPUT);
      loader_slide_motor->SetOutput(LOADER_SLIDE_MOTOR_UP_OUTPUT);
      osDelay(1000);
      loader_feed_motor->SetOutput(LOADER_FEED_MOTOR_CONTACT_OUTPUT);
      osDelay(1000);
      loader_slide_motor->SetOutput(LOADER_SLIDE_MOTOR_DOWN_OUTPUT);
      osDelay(1000);
      loader_slide_motor->SetOutput(LOADER_SLIDE_MOTOR_UP_OUTPUT);
      osDelay(1000);
      loader_feed_motor->SetOutput(LOADER_FEED_MOTOR_RELEASE_OUTPUT);
      osDelay(1000);
      loader_slide_motor->SetOutput(LOADER_SLIDE_MOTOR_DOWN_OUTPUT);
    }

    // Start ch2_right_trigger sequence
    if (ch2_right_trigger.posEdge() && current_sequence_state == SEQUENCE_IDLE) {
      print("CH2 Right Trigger: Starting dart loading sequence...\r\n");
      current_sequence_state = SEQUENCE_MOTOR_POSITION;
      sequence_start_time = osKernelGetTickCount();

      // Step 1: Position motor for loading
      target_angle += PI / 3;
      if (target_angle >= 2 * PI) {
        target_angle = 0.0;
      }
      print("Motor Positioning: Spinning to loading angle: %.2f\r\n", target_angle);
    }

    // State machine for ch2_right_trigger sequence
    uint32_t current_time = osKernelGetTickCount();
    switch (current_sequence_state) {
      case SEQUENCE_MOTOR_POSITION:
        if (current_time - sequence_start_time >= 2000) {  // Wait 2s for motor positioning
          current_sequence_state = SEQUENCE_LOADER_PREPARE;
          sequence_start_time = current_time;
          loader_feed_motor->SetOutput(LOADER_FEED_MOTOR_RELEASE_OUTPUT);
          loader_slide_motor->SetOutput(LOADER_SLIDE_MOTOR_UP_OUTPUT);
          print("Loader Prepare: Release + Up - preparing loader mechanism\r\n");
        }
        break;
      case SEQUENCE_LOADER_PREPARE:
        if (current_time - sequence_start_time >= 1000) {
          current_sequence_state = SEQUENCE_LOADER_GRAB;
          sequence_start_time = current_time;
          loader_feed_motor->SetOutput(LOADER_FEED_MOTOR_CONTACT_OUTPUT);
          print("Loader Grab: Contact + Up - grabbing dart\r\n");
        }
        break;
      case SEQUENCE_LOADER_GRAB:
        if (current_time - sequence_start_time >= 1000) {
          current_sequence_state = SEQUENCE_LOADER_LOAD;
          sequence_start_time = current_time;
          loader_slide_motor->SetOutput(LOADER_SLIDE_MOTOR_DOWN_OUTPUT);
          print("Loader Load: Contact + Down - loading dart into position\r\n");
        }
        break;
      case SEQUENCE_LOADER_LOAD:
        if (current_time - sequence_start_time >= 1000) {
          current_sequence_state = SEQUENCE_LOADER_EJECT;
          sequence_start_time = current_time;
          loader_slide_motor->SetOutput(LOADER_SLIDE_MOTOR_UP_OUTPUT);
          print("Loader Eject: Release + Up - ejecting mechanism back up\r\n");
        }
        break;
      case SEQUENCE_LOADER_EJECT:
        if (current_time - sequence_start_time >= 1000) {
          current_sequence_state = SEQUENCE_LOADER_RESET;
          sequence_start_time = current_time;
          loader_feed_motor->SetOutput(LOADER_FEED_MOTOR_RELEASE_OUTPUT);
          print("Loader Reset: Release + Down - resetting to starting position\r\n");
        }
        break;
      case SEQUENCE_LOADER_RESET:
        if (current_time - sequence_start_time >= 1000) {
          current_sequence_state = SEQUENCE_MOTOR_ADVANCE_AND_RESET;
          sequence_start_time = current_time;
          loader_slide_motor->SetOutput(LOADER_SLIDE_MOTOR_DOWN_OUTPUT);
        }
        break;
      case SEQUENCE_MOTOR_ADVANCE_AND_RESET:
        if (current_time - sequence_start_time >= 1000) {  // Wait 2s for final motor positioning
          current_sequence_state = SEQUENCE_IDLE;
          sequence_start_time = current_time;
          // Advance motor to next position
          target_angle += PI / 3;
          if (target_angle >= 2 * PI) {
            target_angle = 0.0;
          }
          print("Motor Advance: Spinning to next position: %.2f\r\n", target_angle);

          print("Dart Loading Sequence: Completed successfully!\r\n");
        }
        break;
      default:
        break;
    }

    // Enhanced debug output for cascade control and switch states
    const char* swr_str = (dbus->swr == remote::UP) ? "UP" : (dbus->swr == remote::MID) ? "MID"
                                                                                        : "DOWN";
    const char* swl_str = (dbus->swl == remote::UP) ? "UP" : (dbus->swl == remote::MID) ? "MID"
                                                                                        : "DOWN";
    print("target_angle: %.2f, actual_angle: %.2f, target_vel: %.2f, actual_vel: %.2f, command: %d, ch3: %d, swr: %s, swl: %s\r\n",
          target_angle, load_motor->GetTheta(), target_velocity, load_motor->GetOmega(), command, dbus->ch3, swr_str, swl_str);

    load_motor->SetOutput(command);

    control::MotorCANBase::TransmitOutput(motors, 1);
    osDelay(2);
  }
}

