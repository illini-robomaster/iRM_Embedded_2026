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
#pragma once

#include "main.h"
#include "user_interface.h"
#include "referee_task.h"
#include "cmsis_os.h"
#include "protocol.h"
#include "arm.h"
#include "bsp_gpio.h"
#include <cstring>
#include <cstdio>

#define UI_TASK_DELAY 20

extern osThreadId_t UITaskHandle;
const osThreadAttr_t UITaskAttribute = {.name = "UITask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 1024 * 4,
        .priority = (osPriority_t)osPriorityBelowNormal,
        .tz_module = 0,
        .reserved = 0};



extern communication::Referee* referee;
extern RefereeUART* referee_uart;
extern joint_state_t current_motor_angles;

extern bsp::GPIO* key;

void UITask(void* arg);
void refresh();
void init_ui();
void kill_ui();
