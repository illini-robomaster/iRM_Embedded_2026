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

/**
 * @file pwm.cc
 * @brief PWM Example for DM-MC02 Board
 * 
 * This example demonstrates servo control using PWM output.
 * 
 * Hardware Setup:
 *   - TIM1_CH3 -> PE13 (PWM output)
 *   - Connect servo signal wire to PE13
 *   - Connect servo power (5V/6V) and GND
 * 
 * Timer Configuration:
 *   - TIM1 clock: 240 MHz
 *   - Prescaler: 24 (set in CubeMX)
 *   - Effective clock: 240 MHz / 25 = 9.6 MHz
 *   - PWM frequency: 50 Hz (standard servo)
 *   - Pulse width: 1000-2000 µs (servo range)
 * 
 * Note: TIM1 is an advanced timer that requires MOE (Main Output Enable)
 *       to be set. The bsp::PWM class handles this automatically.
 */

#include "bsp_pwm.h"
#include "bsp_print.h"
#include "dbus.h"
#include "main.h"
#include "cmsis_os.h"
#define USE_DBUS
// TIM1 clock after prescaler = 240 MHz / 25 = 9.6 MHz
// TIM1_CH3 -> PE13
static bsp::PWM* servo = nullptr;
remote::DBUS* dbus = nullptr;
void RM_RTOS_Init(void) {
  // Initialize PWM: 9.6 MHz clock, 50 Hz output, 1500 µs pulse (center)
  servo = new bsp::PWM(&htim1, 3, 9600000, 50, 1500);
  servo->Start();
  print_use_uart(&huart7);
  #ifdef USE_DBUS
  dbus = new remote::DBUS(&huart5);
  #endif

}

void RM_RTOS_Default_Task(const void* argument) {
  UNUSED(argument);
  #ifndef USE_DBUS
  while (1) {
    // Sweep servo through its range
    servo->SetPulseWidth(1000);  // Min position (-90°)
    osDelay(1000);
    
    servo->SetPulseWidth(1500);  // Center position (0°)
    osDelay(1000);
    
    servo->SetPulseWidth(2000);  // Max position (+90°)
    osDelay(1000);
  }
  #endif
  #ifdef USE_DBUS
  while (1) {
    int16_t ch3 = dbus->ch3;
    // Map CH3 (-660 to +660) to pulse width (500 to 2500 µs)
    // Formula: pulse = (ch3 + 660) * (2000 / 1320) + 500
    // Range: ch3=-660 -> 500µs, ch3=0 -> 1500µs, ch3=+660 -> 2500µs
    uint32_t pulse_width = static_cast<uint32_t>((ch3 + 660) * (2000.0f / 1320.0f) + 500);
    servo->SetPulseWidth(pulse_width);
    print("DBUS CH3: %d -> Pulse Width: %lu us\r\n", ch3, pulse_width);
    osDelay(10);
  }
  #endif
}