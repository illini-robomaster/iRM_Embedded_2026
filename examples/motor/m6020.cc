/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2023 RoboMaster.                                          *
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

bsp::CAN* can2 = NULL;
bsp::GPIO* key = nullptr;
control::MotorCANBase* motor1 = NULL;
control::MotorCANBase* motor2 = NULL;

void RM_RTOS_Init() {
  print_use_uart(&huart1);

  can2 = new bsp::CAN(&hcan2, true);
  motor1 = new control::Motor6020(can2, 0x206);
  // motor2 = new control::Motor6020(can1, 0x207);
  // key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* motors[] = {motor1};

  // while(!key->Read());

  // while(key->Read());

  print("ok!\r\n");

  while (true) {
    // if (key->Read()) {
      // motor2->SetOutput(0);
      motor1->SetOutput(0);
    // } else {
      // motor1->SetOutput(3200);
      print("%10.4f ", motor1->GetTheta());
    // }
    control::MotorCANBase::TransmitOutput(motors, 1);
    osDelay(2);
  }
}
