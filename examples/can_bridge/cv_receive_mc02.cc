/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2026 RoboMaster.                                          *
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

// Receive-only CV downlink example for DM_MC_02. Jetson -> MC02 uses 0x323;
// 0x324 is reserved for possible feedback but is never transmitted here.
// CanBridge wraps hfdcan1 in Classical CAN mode; no CAN FD or extra task is used.

#include "bsp_can_bridge.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"

static bsp::CAN* can = nullptr;
static bsp::CanBridge* cv = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart7);
  can = new bsp::CAN(&hfdcan1, 0);
  cv = new bsp::CanBridge(can, 0x323, 0x324);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  while (true) {
    print("CV rel_yaw=%f rel_pitch=%f mode=%d fire_request=%d\r\n",
          cv->relative_angle, cv->relative_pitch, cv->mode, cv->cv_fire);
    osDelay(100);
  }
}
