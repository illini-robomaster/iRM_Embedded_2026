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


#include "cmsis_os.h"
#include "sbus.h"
#include "bsp_can.h"
typedef struct {
  float base_translate_0;     /* translate 3508 motor                           */
  float base_yaw_rotate_1;   /* rotate the entire arm around an vertical axis  */
  float base_pitch_rotate_2;    /* rotate the entire arm around a horizontal axis */
  float forearm_pitch_3;       /* rotate the forearm on a horizontal axis        */
  float forearm_roll_4;     /* rotate the forearm on its axis                 */
  float wrist_5;       /* rotate the hand around an vertical axis        */
  float end_6;        /* rotate the hand on its axis                    */
} joint_state_t;

extern osThreadId_t armA1TaskHandle;
const osThreadAttr_t armA1TaskAttribute = {.name = "armTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 1024 * 4,
                                             .priority = (osPriority_t)osPriorityHigh,
                                             .tz_module = 0,
                                             .reserved = 0};


/**
 * @brief task to receive data from A1, currently not functioning
*/
void A1Task(void* arg);


/**
 * @brief initialize the arm A1s
*/
void init_arm_A1();

void kill_arm();

/**
 * @brief main A1 task
*/
void armTask(void* args);

/**
 * @brief turn to an absolute position
 * @note software range limitation defined in arm_config.h
 * @return 0 when the command is accepted, 1 otherwise
 */
int ArmSetTargetAngles(joint_state_t target_joint_state);


/** 
 * @brief read encoder readings to current_joint_positions
 * `caution`: this function should be called after A1 cmd is send 
 * */ 
void ArmLoadInput(joint_state_t &current_joint_positions);

void revive_arm();

#ifdef USING_DBUS
extern remote::DBUS* dbus;
#else
extern remote::SBUS* sbus;
#endif

extern bsp::CAN* can1;
extern bsp::CAN* can2;