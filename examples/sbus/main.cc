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
#include "sbus.h"

static remote::SBUS* sbus;

void RM_RTOS_Init(void) {
  print_use_uart(&huart8);
  sbus = new remote::SBUS(&huart1);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  // NOTE(alvin): print is split because of stack usage is almost reaching limits
  while (true) {
    set_cursor(0, 0);
    clear_screen();
    print("CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d ", sbus->ch[0], sbus->ch[1], sbus->ch[2], sbus->ch[3]);
    print("CH4: %-4d CH5: %-4d CH6: %-4d CH7: %-4d ", sbus->ch[4], sbus->ch[5], sbus->ch[6], sbus->ch[7]);
    print("CH8: %-4d CH9: %-4d CH10: %-4d CH11: %-4d ", sbus->ch[8], sbus->ch[9], sbus->ch[10], sbus->ch[11]);
    print("CH12: %-4d CH13: %-4d CH14: %-4d CH15: %-4d ", sbus->ch[12], sbus->ch[13], sbus->ch[14], sbus->ch[15]);
    print("@ %d ms\r\n", sbus->timestamp);
    osDelay(100);
  }
}
