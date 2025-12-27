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
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "sbus.h"
#include "bsp_gpio.h"
#include "A1/A1_motor_drive.h"

static remote::SBUS* dbus;
bsp::GPIO* user_key = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart4);
  print("Unitree Motor Test\r\n");
  dbus = new remote::SBUS(&huart3);
  user_key = new bsp::GPIO(GPIOA, GPIO_PIN_15);
}

void RM_RTOS_Threads_Init(void) {
}

double start_time;

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  // print("Press key to start\r\n");
  // while(user_key->Read()); // wait for key press
  osDelay(1000);
  print("Open loop spin test\r\n");


  start_time = HAL_GetTick();
  int last_print_time = HAL_GetTick();
  while(HAL_GetTick() - start_time < 2000){
    modify_stop_cmd(&MotorA1_send,2);
    unitreeA1_rxtx(huart1);

    if(HAL_GetTick() - last_print_time > 100){
      last_print_time = HAL_GetTick();
      print("d: %f, %f\r\n", MotorA1_recv_id02.Pos, MotorA1_recv_id02.W);
    }
    
    osDelay(1);
  }
  modify_speed_cmd(&MotorA1_send,2,0);
  unitreeA1_rxtx(huart1);
  print("finished\r\n");

}
