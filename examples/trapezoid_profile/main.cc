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
#include "cmsis_os.h"
#include "trapezoid_profile.h"

#define USE_SBUS 

#ifdef USE_SBUS
#include "sbus.h"
static remote::SBUS* sbus;
#endif

void RM_RTOS_Init(void) {
  print_use_uart(&UART_HANDLE);

#ifdef USE_SBUS
  sbus = new remote::SBUS(&huart3);
#endif
 
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);

    TrapezoidProfile profile(2, 2); // acceleration, cruise velocity, initial position
    kinematics_state current_state = {0,0};
    auto last_loop_time = HAL_GetTick();
    while (true) {
        // get current time
        auto current_time = HAL_GetTick();
        auto delta_t = current_time - last_loop_time; // or use a fixed typical delta t time

        // calculate the new target
#ifdef USE_SBUS
        // get the target from the remote controller
        kinematics_state new_state = profile.calculate(sbus->ch[0]/660.0*5, delta_t, current_state); // target pos, delta_t, current state
#else 
        kinematics_state new_state = profile.calculate(5, delta_t, current_state); // target pos, delta_t, current state
#endif
        last_loop_time = current_time;
        print("t p v: %d %f %f\n", current_time, new_state.position, new_state.velocity);
        current_state = new_state;
        
        osDelay(10);
    }
}
