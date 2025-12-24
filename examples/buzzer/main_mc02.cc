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

/**
 * @brief DM-MC-02 Buzzer Example
 * 
 * Hardware:
 *   - Buzzer connected to TIM12_CH2 (PB15)
 *   - Clock calculation:
 *     - SYSCLK = 160 MHz (HSE 8MHz / PLLM 2 × PLLN 40 / PLLP 1)
 *     - HCLK = 80 MHz (SYSCLK / 2)
 *     - APB1 = 40 MHz (HCLK / 2)
 *     - APB1 Timer clock = 80 MHz (APB1 × 2, because APB1 divider > 1)
 *     - TIM12 Prescaler = 24
 *     - Effective PWM clock = 80 MHz / 24 ≈ 3.33 MHz
 */

#include "main.h"
#include "bsp_buzzer.h"
#include "tim.h"
#include "cmsis_os.h"

// APB1 Timer clock = 80 MHz, Prescaler = 24
// Effective clock = 80,000,000 / 24 = 3,333,333 Hz
#define BUZZER_CLOCK_FREQ  (80000000 / 24)

using Note = bsp::BuzzerNote;

static bsp::BuzzerNoteDelayed Mario[] = {
    {Note::Mi3M, 80}, {Note::Silent, 80},  {Note::Mi3M, 80}, {Note::Silent, 240},
    {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::Do1M, 80}, {Note::Silent, 80},
    {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::So5M, 80}, {Note::Silent, 560},
    {Note::So5L, 80}, {Note::Silent, 0},   {Note::Finish, 0}};

static bsp::BuzzerNoteDelayed Startup[] = {
    {Note::Do1M, 100}, {Note::Mi3M, 100}, {Note::So5M, 100}, {Note::Do1H, 200},
    {Note::Silent, 0}, {Note::Finish, 0}};

static bsp::Buzzer* buzzer = nullptr;

extern "C" void RM_RTOS_Init(void) {
  buzzer = new bsp::Buzzer(&htim12, 2, BUZZER_CLOCK_FREQ);
  
  // Play startup melody (blocking, before RTOS starts)
  buzzer->SingSong(Startup);
}

extern "C" void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);
  
  // Play Mario theme once after RTOS starts
  osDelay(500);
  buzzer->SingSong(Mario, [](uint32_t ms) { osDelay(ms); });
  
  // Keep buzzer silent
  buzzer->SingTone(Note::Silent);
  
  while (true) {
    osDelay(1000);
  }
}
