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
// #define REFEREE
#define CHASSIS

#include "main.h"
#include "bsp_os.h"
#include "can.h"
#include "rgb.h"
#include "chassis_task.h"
#include "arm_translate_task.h"
//  #include "ui_task.h"
#ifdef REFEREE
#include "referee_task.h"
#endif
#include "bsp_print.h"
#include "sbus.h"
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
bsp::CanBridge* receive = nullptr;
static bool armIsKilled = false;

remote::SBUS* sbus = nullptr;
#ifdef REFEREE
 RefereeUART* referee_uart = nullptr;
 communication::Referee* referee = nullptr;
#endif





void RM_RTOS_Init() {
    print_use_uart(&huart1);
    bsp::SetHighresClockTimer(&htim5);
    sbus = new remote::SBUS(&huart3);
    can1 = new bsp::CAN(&hcan1, true);
    can2 = new bsp::CAN(&hcan2, false);
    receive = new bsp::CanBridge(can2, 0x20B, 0x20A);
    
#ifdef REFEREE
   referee_uart = new RefereeUART(&huart6);
   referee = new communication::Referee();
#endif

    init_chassis();
    init_arm_translate();

    set_cursor(0,0);
    clear_screen();
}


void RM_RTOS_Threads_Init(void) {
    chassisTaskHandle = osThreadNew(chassisTask,nullptr,&chassisTaskAttribute);
    armTranslateTaskHandle = osThreadNew(armTranslateTask,nullptr,&armTranslateAttribute);
#ifdef REFEREE
    refereeTaskHandle = osThreadNew(refereeTask,nullptr,&refereeTaskAttribute);
#endif
//     UITaskHandle = osThreadNew(UITask,nullptr,&UITaskAttribute);
}


void KillAll() {

    kill_chassis();
    kill_arm_translate();
}

void ReviveAll(){
    revive_chassis();
    revive_arm_translate();
}


void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    while(true){
        // if want to print, make sure nothing is print somewhere else
        if (receive->dead) {
            if (!armIsKilled) {
                print("killed\n");
            }
            armIsKilled = true;
            KillAll();
            osDelay(100);
        } else if (armIsKilled && !receive->dead) {
            // killed to revive
            ReviveAll();
            armIsKilled = false;
        }

#ifdef REFEREE
        // print("ROBOTID: %d",referee->game_robot_status.robot_id);
#endif
        osDelay(10);
    }
}
