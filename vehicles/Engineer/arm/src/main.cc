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
#define REFEREE
#define ARM_A1

#include "main.h"
#include "bsp_os.h"
#include "can.h"
#include "rgb.h"
#include "bsp_can_bridge.h"
#include "ui_task.h"
// "arm.h" already included within ui_task.h for angle information
#ifdef REFEREE
#include "referee_task.h"
#endif
#include "bsp_print.h"

#ifdef USING_DBUS
#include "dbus.h"
#else
#include "sbus.h"
#endif
//
osThreadId_t chassisTaskHandle;
osThreadId_t armTranslateTaskHandle;
osThreadId_t armA1TaskHandle;
osThreadId_t UITaskHandle;
osThreadId_t testTaskHandle;
#ifdef REFEREE
osThreadId_t refereeTaskHandle;
#endif


bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;
bsp::CanBridge* send = nullptr;
static bool armIsKilled = false;

#ifdef USING_DBUS
remote::DBUS* dbus = nullptr;
#else
remote::SBUS* sbus = nullptr;
#endif
#ifdef REFEREE
RefereeUART* referee_uart = nullptr;
communication::Referee* referee = nullptr;
#endif





void RM_RTOS_Init() {
  print_use_uart(&huart4);
  bsp::SetHighresClockTimer(&htim5);
#ifdef USING_DBUS
  dbus = new remote::DBUS(&huart3);
#else
  sbus = new remote::SBUS(&huart3);
#endif
  can1 = new bsp::CAN(&hcan1, true);
  can2 = new bsp::CAN(&hcan2, false);
  send = new bsp::CanBridge(can2, 0x20A, 0x20B);
#ifdef REFEREE
  referee_uart = new RefereeUART(&huart5);
  referee_uart->SetupTx(300);
  referee_uart->SetupRx(300);
  referee = new communication::Referee();
#endif


  init_arm_A1();
  init_ui();
  set_cursor(0,0);

  clear_screen();
}


void RM_RTOS_Threads_Init(void) {


  armA1TaskHandle = osThreadNew(armTask, nullptr, &armA1TaskAttribute);


#ifdef REFEREE
  refereeTaskHandle = osThreadNew(refereeTask,nullptr,&refereeTaskAttribute);
#endif
//     UITaskHandle = osThreadNew(UITask,nullptr,&UITaskAttribute);
}


void KillAll() {

  kill_arm();

}

void ReviveAll(){
  revive_arm();

}


void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    while(true){ //if want to print, make sure nothing is print somewhere else
        if(sbus->ch[11]>100 || sbus->ch[11]<-100){
            if(!armIsKilled){
                print("killed\n");
            }
            armIsKilled = true;
            KillAll();
            // print("killed");
            osDelay(10);
        }else if(armIsKilled && (sbus->ch[11]<=100 || sbus->ch[11]<-100)){ // killed to revive
            ReviveAll();
            armIsKilled = false;
        }

#ifdef REFEREE
        // print("ROBOTID: %d",referee->game_robot_status.robot_id);
#endif
        send->cmd.id = bsp::VX;
        send->cmd.data_float = sbus->ch[6]/660.0;
        send->TransmitOutput();
        send->cmd.id = bsp::VY;
        send->cmd.data_float = sbus->ch[7]/660.0;
        send->TransmitOutput();
        send->cmd.id = bsp::RELATIVE_ANGLE;
        send->cmd.data_float = -sbus->ch[8]/660.0;
        send->TransmitOutput();
        send->cmd.id = bsp::ARM_TRANSLATE;
        send->cmd.data_float = sbus->ch[9]/660.0;
        send->TransmitOutput();
        send->cmd.id = bsp::DEAD;
        send->cmd.data_bool = sbus->ch[11]>100;
        send->TransmitOutput();
        send->cmd.id = bsp::CHASSIS_POWER;
        send->cmd.data_float = referee->power_heat_data.chassis_power;
        send->TransmitOutput();
        send->cmd.id = bsp::CHASSIS_POWER_LIMIT;
        send->cmd.data_float = referee->game_robot_status.chassis_power_limit;
        send->TransmitOutput();
        // print("can bridge sent\r\n");
        osDelay(10);
    }
}
