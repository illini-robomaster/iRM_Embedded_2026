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
 *
 * IMPORTANT: For best results, play songs in RM_RTOS_Default_Task (after RTOS starts)
 *            using osDelay. Playing in RM_RTOS_Init with HAL_Delay may have timing issues.
 */

#include "main.h"
#include "bsp_buzzer.h"
#include "tim.h"
#include "cmsis_os.h"

// APB1 Timer clock = 80 MHz, Prescaler = 24
// Effective clock = 80,000,000 / 24 = 3,333,333 Hz
#define BUZZER_CLOCK_FREQ  (80000000 / 24)

using Note = bsp::BuzzerNote;

static bsp::Buzzer* buzzer = nullptr;

// Simple test melody: Do-Re-Mi-Fa-So-La-Si-Do
static bsp::BuzzerNoteDelayed TestScale[] = {
    {Note::Do1M, 300}, {Note::Re2M, 300}, {Note::Mi3M, 300}, {Note::Fa4M, 300}, {Note::So5M, 300}, {Note::La6M, 300}, {Note::Si7M, 300}, {Note::Do1H, 300}, {Note::Silent, 0}, {Note::Finish, 0}};

static bsp::BuzzerNoteDelayed Mario[] = {
    {Note::Mi3M, 80}, {Note::Silent, 80}, {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::Do1M, 80}, {Note::Silent, 80}, {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::So5M, 80}, {Note::Silent, 560}, {Note::So5L, 80}, {Note::Silent, 0}, {Note::Finish, 0}};
// uncomment to play the full song
static bsp::BuzzerNoteDelayed War_Cant_of_Mars[] = {
    //    {Note::So5M, 400},   {Note::So5M, 200},  {Note::So5M, 200},
    //    {Note::So5M, 400},   {Note::Fa4M, 200},  {Note::Mi3M, 400},
    //    {Note::So5M, 200},   {Note::Do1H, 400},  {Note::Re2H, 200},
    //    {Note::Mi3H, 400},   {Note::Mi3H, 200},  {Note::Mi3H, 400},
    //    {Note::Re2H, 200},   {Note::Do1H, 400},  {Note::Do1H, 400},
    //    {Note::Si7M, 200},   {Note::La6M, 400},  {Note::La6M, 200},
    //    {Note::La6M, 400},   {Note::Si7M, 200},  {Note::Do1H, 400},
    //    {Note::Si7M, 200},   {Note::Do1H, 400},  {Note::La6M, 200},
    //    {Note::So5M, 400},   {Note::La6M, 200},  {Note::So5M, 400},
    //    {Note::Mi3M, 200},   {Note::So5M, 800},  {Note::So5M, 400},
    //    {Note::So5M, 200},   {Note::So5M, 400},  {Note::So5M, 200},
    //    {Note::So5M, 400},   {Note::Fa4M, 200},  {Note::Mi3M, 400},
    //    {Note::So5M, 200},   {Note::Do1H, 400},  {Note::Re2H, 200},
    //    {Note::Mi3H, 400},   {Note::Mi3H, 200},  {Note::Mi3H, 400},
    //    {Note::Re2H, 200},   {Note::Do1H, 800},  {Note::Do1H, 800},
    //    {Note::Re2H, 800},   {Note::Re2H, 800},  {Note::Do1H, 800},
    //    {Note::Si7M, 800},   {Note::Do1H, 1600}, {Note::Silent, 400},
    {Note::Silent, 400},
    {Note::So5M, 800},
    {Note::Fa4M, 400},
    {Note::Mi3M, 400},
    {Note::So5M, 200},
    {Note::Do1H, 400},
    {Note::Re2H, 200},
    {Note::Mi3H, 1200},
    {Note::Do1H, 800},
    //    {Note::Silent, 400}, {Note::La6M, 800},   {Note::Si7M, 400},
    //    {Note::Do1H, 400},   {Note::Si7M, 200},   {Note::Do1H, 400},
    //    {Note::La6M, 200},   {Note::So5M, 1600},  {Note::Mi3M, 800},
    //    {Note::Silent, 400}, {Note::So5M, 800},   {Note::Fa4M, 400},
    //    {Note::Mi3M, 400},   {Note::So5M, 200},   {Note::Do1H, 400},
    //    {Note::Re2H, 200},   {Note::Mi3H, 1600},  {Note::Do1H, 800},
    //    {Note::Do1H, 800},   {Note::Re2H, 800},   {Note::Re2H, 800},
    //    {Note::Do1H, 800},   {Note::Si7M, 800},   {Note::Do1H, 1600},
    {Note::Silent, 0},
    {Note::Finish, 0},
};

static bsp::BuzzerNoteDelayed StarWarTheme[] = {
    {Note::NOTE_AS4, 250},
    {Note::NOTE_AS4, 250},
    {Note::NOTE_AS4, 250},  // 1
    {Note::NOTE_F5, 1000},
    {Note::NOTE_C6, 1000},
    {Note::NOTE_AS5, 250},
    {Note::NOTE_A5, 250},
    {Note::NOTE_G5, 250},
    {Note::NOTE_F6, 1000},
    {Note::NOTE_C6, 500},
    {Note::NOTE_AS5, 250},
    {Note::NOTE_A5, 250},
    {Note::NOTE_G5, 250},
    {Note::NOTE_F6, 1000},
    {Note::NOTE_C6, 500},
    {Note::NOTE_AS5, 250},
    {Note::NOTE_A5, 250},
    {Note::NOTE_AS5, 250},
    {Note::NOTE_G5, 1000},
    {Note::NOTE_C5, 250},
    {Note::NOTE_C5, 250},
    {Note::NOTE_C5, 250},
    {Note::NOTE_F5, 1000},
    {Note::NOTE_C6, 1000},
    {Note::NOTE_AS5, 250},
    {Note::NOTE_A5, 250},
    {Note::NOTE_G5, 250},
    {Note::NOTE_F6, 1000},
    {Note::NOTE_C6, 500},
    {Note::NOTE_AS5, 250},
    {Note::NOTE_A5, 250},
    {Note::NOTE_G5, 250},
    {Note::NOTE_F6, 1000},
    {Note::NOTE_C6, 500},  // 8
    {Note::NOTE_AS5, 250},
    {Note::NOTE_A5, 250},
    {Note::NOTE_AS5, 250},
    {Note::NOTE_G5, 1000},
    {Note::NOTE_C5, 450},
    {Note::NOTE_C5, 16},
    {Note::NOTE_D5, 900},
    {Note::NOTE_D5, 250},
    {Note::NOTE_AS5, 250},
    {Note::NOTE_A5, 250},
    {Note::NOTE_G5, 250},
    {Note::NOTE_F5, 250},
    {Note::NOTE_F5, 250},
    {Note::NOTE_G5, 250},
    {Note::NOTE_A5, 250},
    {Note::NOTE_G5, 500},
    {Note::NOTE_D5, 250},
    {Note::NOTE_E5, 500},
    {Note::NOTE_C5, 450},
    {Note::NOTE_C5, 16},
    {Note::NOTE_D5, 900},
    {Note::NOTE_D5, 250},
    {Note::NOTE_AS5, 250},
    {Note::NOTE_A5, 250},
    {Note::NOTE_G5, 250},
    {Note::NOTE_F5, 250},
    {Note::NOTE_C6, 450},
    {Note::NOTE_G5, 16},
    {Note::NOTE_G5, 2},
    {Note::Silent, 250},
    {Note::NOTE_C5, 250},  // 13
    {Note::NOTE_D5, 900},
    {Note::NOTE_D5, 250},
    {Note::NOTE_AS5, 250},
    {Note::NOTE_A5, 250},
    {Note::NOTE_G5, 250},
    {Note::NOTE_F5, 250},
    {Note::NOTE_F5, 250},
    {Note::NOTE_G5, 250},
    {Note::NOTE_A5, 250},
    {Note::NOTE_G5, 500},
    {Note::NOTE_D5, 250},
    {Note::NOTE_E5, 500},
    {Note::NOTE_C6, 450},
    {Note::NOTE_C6, 16},
    {Note::NOTE_F6, 500},
    {Note::NOTE_DS6, 250},
    {Note::NOTE_CS6, 500},
    {Note::NOTE_C6, 250},
    {Note::NOTE_AS5, 500},
    {Note::NOTE_GS5, 250},
    {Note::NOTE_G5, 500},
    {Note::NOTE_F5, 250},
    {Note::NOTE_C6, 2000},
    {Note::Silent, 0},
    {Note::Finish, 0},
};

void RM_RTOS_Init(void) {
  UNUSED(Mario);
  UNUSED(War_Cant_of_Mars);
  UNUSED(StarWarTheme);
  UNUSED(TestScale);

  buzzer = new bsp::Buzzer(&htim12, 2, BUZZER_CLOCK_FREQ);

  // Note: For complex melodies, prefer playing in RM_RTOS_Default_Task with osDelay
  // Simple single tone here is OK for startup beep
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  // Play songs in loop using osDelay (recommended for multi-note melodies)
  while (true) {
    // buzzer->SingSong(Mario, [](uint32_t ms) { osDelay(ms); });
    // osDelay(2000);  // Pause between songs
    buzzer->SingSong(StarWarTheme, [](uint32_t ms) { osDelay(ms); });
    osDelay(2000);
    // Uncomment to play other songs:
    // buzzer->SingSong(War_Cant_of_Mars, [](uint32_t ms) { osDelay(ms); });
    // osDelay(2000);
  }
}
