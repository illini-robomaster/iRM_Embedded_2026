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




void RM_RTOS_Init() {
  print_use_uart(&huart1);
  can1 = new bsp::CAN(&hcan1, true);
  load_motor = new control::Motor6020(can1, 0x207);
  dbus = new remote::DBUS(&huart3);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  print("ok!\r\n");
  
  osDelay(500);  // DBUS initialization needs time
  
  float pid_params[3] = {55000.0, 10.0, 100.0};
  control::ConstrainedPID pid_(pid_params, 30000, 30000);

  float target_angle = 0.0;
  control::MotorCANBase* motors[] = {load_motor};
  int16_t command;
  
  // Edge detector for dbus channel 1 > 300
  BoolEdgeDetector ch1_edge_detector(false);

  while (true) {
    command = pid_.ComputeOutput(load_motor->GetThetaDelta(target_angle));
    
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

    // print("command: %d \r\n", command);
    print("target angle: %.2f, actual angle: %.2f, command: %d, ch1: %d\r\n", 
          target_angle, load_motor->GetTheta(), command, dbus->ch1);
    
    load_motor->SetOutput(command);
    control::MotorCANBase::TransmitOutput(motors, 1);
    osDelay(2);
  }
}

