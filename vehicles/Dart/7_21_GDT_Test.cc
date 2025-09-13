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

bsp::CAN* can1 = NULL;
bsp::GPIO* key = nullptr;
control::MotorCANBase* load_motor = NULL;
remote::DBUS* dbus = nullptr;

// Cascade PID controllers
control::ConstrainedPID* position_pid = nullptr;  // Position (outer loop)
control::ConstrainedPID* velocity_pid = nullptr;  // Velocity (inner loop)

int16_t command = 0;

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  can1 = new bsp::CAN(&hcan1, true);
  load_motor = new control::Motor6020(can1, 0x207);
  dbus = new remote::DBUS(&huart3);

  // Initialize cascade PID controllers
  // Position PID (outer loop) - tuned for position control
  float position_pid_params[3] = {30.0, 0.0, 0.3};                          // Kp, Ki, Kd for position
  position_pid = new control::ConstrainedPID(position_pid_params, 0, 2.5);  // Low max_out for velocity reference

  // Velocity PID (inner loop) - tuned for velocity control
  float velocity_pid_params[3] = {3600.0, 20.0, 0.0};                             // Kp, Ki, Kd for velocity
  velocity_pid = new control::ConstrainedPID(velocity_pid_params, 10000, 30000);  // Higher limits for motor output
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  print("ok!\r\n");

  osDelay(500);  // DBUS initialization needs time

  float target_angle = 0.0;
  control::MotorCANBase* motors[] = {load_motor};

  // Edge detector for dbus channel 1 > 300
  BoolEdgeDetector ch1_edge_detector(false);

  while (true) {
    // Cascade PID Control Implementation
    // Step 1: Position PID (outer loop) - calculates desired velocity
    float position_error = load_motor->GetThetaDelta(target_angle);
    float target_velocity = position_pid->ComputeOutput(position_error);

    // Step 2: Velocity PID (inner loop) - calculates motor command
    float velocity_error = load_motor->GetOmegaDelta(target_velocity);
    command = velocity_pid->ComputeConstrainedOutput(velocity_error);

    // Update edge detector with ch1 > 300 condition
    ch1_edge_detector.input(dbus->ch1 > 300);
    
    // Check for positive edge (transition from false to true)
    if (ch1_edge_detector.posEdge()) {
      target_angle += PI/3;
      if (target_angle >= 2 * PI) {
        target_angle = 0.0;
      }
      print("Edge detected! New target angle: %.2f\r\n", target_angle);
    }

    // Enhanced debug output for cascade control
    print("target_angle: %.2f, actual_angle: %.2f, target_vel: %.2f, actual_vel: %.2f, command: %d, ch1: %d\r\n",
          target_angle, load_motor->GetTheta(), target_velocity, load_motor->GetOmega(), command, dbus->ch1);

    load_motor->SetOutput(command);
    control::MotorCANBase::TransmitOutput(motors, 1);
    osDelay(2);
  }
}

