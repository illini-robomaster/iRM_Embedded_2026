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

#include "main.h"

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"

static bsp::GPIO* input1 = nullptr;
static bsp::GPIO* input2 = nullptr;
static bsp::GPIO* input3 = nullptr;
static bsp::GPIO* input4 = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  
  input1 = new bsp::GPIO(IN1_GPIO_Port, IN1_Pin);
  input2 = new bsp::GPIO(IN2_GPIO_Port, IN2_Pin);
  input3 = new bsp::GPIO(IN3_GPIO_Port, IN3_Pin);
  input4 = new bsp::GPIO(IN4_GPIO_Port, IN4_Pin);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  while (true) {
    set_cursor(0, 0);
    clear_screen();
    print("sensor: %d %d %d %d\r\n", input1->Read(), input2->Read(), input3->Read(),
          input4->Read());
    osDelay(100);
  }
}
