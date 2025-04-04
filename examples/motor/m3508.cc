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

#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"
#include "dbus.h"

static bsp::CAN* can = nullptr;
static control::MotorCANBase* motor = nullptr;
remote::DBUS  *dbus = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart1);

  can = new bsp::CAN(&hcan2, false);
  motor = new control::Motor3508(can, 0x205);
  dbus = new remote::DBUS(&huart3); // Initialize DBUS for keyboard input
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* motors[] = {motor};

  while (true) {
    print("dbus->ch0: %d, dbus->ch1: %d, dbus->ch2: %d\r\n", dbus->ch0, dbus->ch1, dbus->ch2);
//    motor->SetOutput(800);
    control::MotorCANBase::TransmitOutput(motors, 1);
    osDelay(100);
  }
}
