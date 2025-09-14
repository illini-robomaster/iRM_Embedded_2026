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
#define LOADER_SLIDE_MOTOR_UP_OUTPUT 514

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
  BoolEdgeDetector ch1_edge_detector(false);  // For PWM state control
  BoolEdgeDetector ch3_edge_detector(false);  // For target angle control

  while (true) {
    // Cascade PID Control Implementation
    // Step 1: Position PID (outer loop) - calculates desired velocity
    float position_error = load_motor->GetThetaDelta(target_angle);
    float target_velocity = position_pid->ComputeOutput(position_error);

    // Step 2: Velocity PID (inner loop) - calculates motor command
    float velocity_error = load_motor->GetOmegaDelta(target_velocity);
    command = velocity_pid->ComputeConstrainedOutput(velocity_error);

    // Update edge detectors
    ch1_edge_detector.input(dbus->ch1 > 300);
    ch3_edge_detector.input(dbus->ch3 > 300);

    // Check for ch3 positive edge to increment target angle
    if (ch3_edge_detector.posEdge()) {
      target_angle += PI / 3;
      if (target_angle >= 2 * PI) {
        target_angle = 0.0;
      }
      print("CH3 Edge detected! New target angle: %.2f\r\n", target_angle);
    }

    // Control slide motor based on swr switch position
    if (dbus->swr == remote::UP) {
      loader_slide_motor->SetOutput(LOADER_SLIDE_MOTOR_UP_OUTPUT);  // Slide motor up when swr is UP
    } else {
      loader_slide_motor->SetOutput(LOADER_SLIDE_MOTOR_DOWN_OUTPUT);  // Slide motor down when swr is not UP
    }

    // Control feed motor based on swl switch position
    if (dbus->swl == remote::UP) {
      loader_feed_motor->SetOutput(LOADER_FEED_MOTOR_CONTACT_OUTPUT);  // Feed motor contact when swl is UP
    } else {
      loader_feed_motor->SetOutput(LOADER_FEED_MOTOR_RELEASE_OUTPUT);  // Feed motor release when swl is not UP
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

