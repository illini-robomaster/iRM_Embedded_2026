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

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"
#include "sbus.h"

static bsp::CAN* can = nullptr;
static control::Motor4310* motor = nullptr;
remote::SBUS* dbus = nullptr;
bsp::GPIO* key = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart4);
  can = new bsp::CAN(&hcan1, true);
  key = new bsp::GPIO(GPIOA, GPIO_PIN_15);

  /* rx_id = Master id
   * tx_id = CAN id
   * mode:
   *  MIT: MIT mode
   *  POS_VEL: position-velocity mode
   *  VEL: velocity mode  */

  /* Make sure motor is set to the correct mode (in helper tool). Otherwise, motor won't start */
  motor = new control::Motor4310(can, 0x06, 0x05, control::MIT);
  dbus = new remote::SBUS(&huart3);
}

void RM_RTOS_Default_Task(const void* args) {
  /* press reset if no response */
  UNUSED(args);

  control::Motor4310* motors[] = {motor};
  // while(dbus->swr != remote::DOWN){}  // flip swr to start
  print("start\r\n");

  /* Use SetZeroPos if you want to set current motor position as zero position. If uncommented, the
   * zero position is the zero position set before */
  motor->SetZeroPos();
  motor->MotorEnable();

  while(dbus->ch[4] < 100){}  // flip swr t;
  print("motor enabled\r\n" );

  float pos = 0;
  float min_pos = -PI/8;
  float max_pos = PI/6;
  while (true) {
    float vel;
    vel = clip<float>(dbus->ch[1] / 660.0 * 15.0, -15, 15);
    pos += vel / 200;
    pos = clip<float>(pos, min_pos, max_pos);   // clipping position within a range

    set_cursor(0, 0);
    clear_screen();
    print("Vel Set: %f  Pos Set: %f\n", vel, pos);

    motor->SetOutput(pos, vel, 30, 0.5, 0);
    control::Motor4310::TransmitOutput(motors, 1);
    osDelay(10);
  }
}
