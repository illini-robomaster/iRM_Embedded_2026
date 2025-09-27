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
#include "controller.h"
#include "main.h"
#include "motor.h"

#define TARGET_SPEED 30

float kp = 0;
float ki = 0;
float kd = 0;

int16_t out = 0;

bsp::CAN* can = nullptr;
control::MotorCANBase* motor = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  can = new bsp::CAN(&hcan1, true);
  motor = new control::Motor3508(can, 0x201);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  control::MotorCANBase* motors[] = {motor};
  control::PIDController pid(20, 15, 30);

  while (true) {
    float diff = motor->GetOmegaDelta(TARGET_SPEED);
    pid.Reinit(kp, ki, kd);
    out = pid.ComputeConstrainedOutput(diff);
    motor->SetOutput(out);
    control::MotorCANBase::TransmitOutput(motors, 1);
    motor->PrintData();
    osDelay(10);
  }
}
