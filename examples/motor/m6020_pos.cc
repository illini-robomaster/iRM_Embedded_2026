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
#include "main.h"
#include "motor.h"

#define KEY_GPIO_GROUP GPIOA
#define KEY_GPIO_PIN GPIO_PIN_0

bsp::CAN* can1 = NULL;
bsp::GPIO* key = nullptr;
control::MotorCANBase* load_motor = NULL;




void RM_RTOS_Init() {
  print_use_uart(&huart1);
  can1 = new bsp::CAN(&hcan1, true);
  load_motor = new control::Motor6020(can1, 0x20B);
  
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  print("ok!\r\n");
  float pid_params[3] = {20000.0, 0.0, 10.0};
  control::ConstrainedPID pid_(pid_params, 30000, 30000);

  float target_angle = 0.0;
  uint32_t current_time = osKernelGetTickCount();
  uint32_t last_update_time = osKernelGetTickCount();
  control::MotorCANBase* motors[] = {load_motor};
  int16_t command;

  while (true) {
    command = pid_.ComputeOutput(load_motor->GetThetaDelta(target_angle));
    
    current_time = osKernelGetTickCount();

    if (current_time - last_update_time >= 3000) {  // 3 seconds = 3000ms
      last_update_time = current_time;
      target_angle += PI/3;
      if (target_angle >= 2 * PI) {
        target_angle = 0.0;
      }
    }

    // print("command: %d \r\n", command);
    print("target angle: %.2f, actual angle: %.2f, command: %d\r\n", target_angle, load_motor->GetTheta(), command);
    load_motor->SetOutput(command);
    control::MotorCANBase::TransmitOutput(motors, 1);
    osDelay(2);
  }
}

