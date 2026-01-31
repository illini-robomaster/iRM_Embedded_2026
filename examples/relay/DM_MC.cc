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

#include "bsp_print.h"
#include "bsp_gpio.h"
#include "bsp_relay.h"
#include "cmsis_os.h"

static bsp::Relay* relay;

void RM_RTOS_Init(void) {
  print_use_uart(&huart4);
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_DeInit(GPIOC,GPIO_PIN_6);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  relay = new bsp::Relay(GPIOC, GPIO_PIN_6);
    
  relay->Off();
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  while (true) {
    set_cursor(0, 0);
    clear_screen();
    relay->On();
    print("relay on\r\n");
    osDelay(3000);
    set_cursor(0, 0);
    clear_screen();
    relay->Off();
    print("relay off\r\n");
    osDelay(3000);
  }
}
