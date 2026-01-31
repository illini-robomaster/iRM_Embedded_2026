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
/*
 * ============================================================================
 *                        DBUS Channel Mapping
 * ============================================================================
 *
 *     ┌───────────────────────────────────────────────────────────────┐
 *     │   (S1)                                           (S2)         │
 *     │    ○                                              ○           │
 *     │                                                               │
 *     │        ▲                                    ▲                 │
 *     │        │ CH3                                │ CH1             │
 *     │        │ (+)                                │ (+)             │
 *     │   ◄────┼────►  CH2                     ◄────┼────►  CH0       │
 *     │   (-)  │  (+)                          (-)  │  (+)            │
 *     │        │                                    │                 │
 *     │        ▼ (-)                                ▼ (-)             │
 *     │      Left                                 Right               │
 *     │                                                               │
 *     └───────────────────────────────────────────────────────────────┘
 *
 * Joystick Channel Description:
 * - CH0: Right Joystick Left/Right  (Left Negative, Right Positive, Range -660 ~ +660)
 * - CH1: Right Joystick Up/Down     (Down Negative, Up Positive, Range -660 ~ +660)
 * - CH2: Left Joystick Left/Right   (Left Negative, Right Positive, Range -660 ~ +660)
 * - CH3: Left Joystick Up/Down      (Down Negative, Up Positive, Range -660 ~ +660)
 *
 * Switch Description:
 *   - S1 (SWL): Left Top Three-Position Switch
 *   - S2 (SWR): Right Top Three-Position Switch
 *   - Positions: UP = 1, DOWN = 2, MID = 3
 *
 */

#include "main.h"

#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"

static remote::DBUS* dbus;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  dbus = new remote::DBUS(&huart3);
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
