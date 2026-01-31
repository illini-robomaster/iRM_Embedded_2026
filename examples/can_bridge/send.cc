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

#include "bsp_can_bridge.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "sbus.h"

static bsp::CAN* can = nullptr;
static bsp::CanBridge* send = nullptr;
static remote::SBUS* sbus = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart4);
  can = new bsp::CAN(&hcan2, false);
  send = new bsp::CanBridge(can, 0x20A, 0x20B);
  sbus = new remote::SBUS(&huart3);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  while (true) {
    print("VX: %f, VY: %f, VZ: %f\n", sbus->ch[6]/660.0, sbus->ch[7]/660.0, sbus->ch[8]/660.0);
    send->cmd.id = bsp::VX;
    send->cmd.data_float = sbus->ch[6]/660.0;
    send->TransmitOutput();
    send->cmd.id = bsp::VY;
    send->cmd.data_float = sbus->ch[7]/660.0;
    send->TransmitOutput();
    send->cmd.id = bsp::RELATIVE_ANGLE;
    send->cmd.data_float = sbus->ch[8]/660.0;
    send->TransmitOutput();
    send->cmd.id = bsp::ARM_TRANSLATE;
    send->cmd.data_float = sbus->ch[9]/660.0;
    send->TransmitOutput();
    send->cmd.id = bsp::DEAD;
    send->cmd.data_bool = sbus->ch[11]>100;
    send->TransmitOutput();
    send->cmd.id = bsp::CHASSIS_POWER;
    // send->cmd.data_float = referee->power_heat_data.chassis_power;
    send->cmd.data_float = 50;
    send->TransmitOutput();
    send->cmd.id = bsp::CHASSIS_POWER_LIMIT;
    send->cmd.data_float = 50;
    // send->cmd.data_float = referee->game_robot_status.chassis_power_limit;
    send->TransmitOutput();
    osDelay(20);
  }
}
