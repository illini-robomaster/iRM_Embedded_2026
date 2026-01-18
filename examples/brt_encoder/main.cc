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
#include "cmsis_os.h"
#include "encoder.h"

static bsp::CAN* can = nullptr;
static control::BRTEncoder* encoder = nullptr;
static control::BRTEncoder* encoder2 = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&UART_HANDLE);
  can = new bsp::CAN(&hcan1);
  encoder = new control::BRTEncoder(can, 0x01, false);
  encoder2 = new control::BRTEncoder(can, 0x0A, true);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  while (true) {
    // set_cursor(0, 0);
    // clear_screen();
    print("1:");
    encoder->PrintData();
    print("2:");
    encoder2->PrintData();
    osDelay(100);
  }
}
