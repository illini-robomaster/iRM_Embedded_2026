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
#include "dbus.h"

static remote::DBUS* dbus;

void RM_RTOS_Init(void) {
  print_use_uart(&huart8);
  dbus = new remote::DBUS(&huart1);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  // NOTE(alvin): print is split because of stack usage is almost reaching limits
  while (true) {
    set_cursor(0, 0);
    clear_screen();
    print("CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d ", dbus->ch0, dbus->ch1, dbus->ch2, dbus->ch3);
    print("SWL: %d SWR: %d DIAL: %d @ %d ms\r\n", dbus->swl, dbus->swr, dbus->wheel, dbus->timestamp);
    osDelay(100);
  }
}
