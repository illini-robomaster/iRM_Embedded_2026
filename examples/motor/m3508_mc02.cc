/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2024 RoboMaster.                                          *
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

// For DM_MC_02, bsp::CAN is aliased to bsp::FDCAN
static bsp::CAN* can = nullptr;
static control::MotorCANBase* motor = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart7);

  // DM_MC_02 uses FDCAN instead of CAN
  // The bsp::CAN type is automatically aliased to FDCAN on this board
  can = new bsp::CAN(&hfdcan1, 0);  // FDCAN1 with filter ID 0
  motor = new control::Motor3508(can, 0x205);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* motors[] = {motor};

  while (true) {
    motor->SetOutput(800);
    control::MotorCANBase::TransmitOutput(motors, 1);
    osDelay(2);  // Motor control needs high frequency update (500Hz)
  }
}
