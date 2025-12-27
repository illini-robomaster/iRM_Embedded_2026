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

#pragma once

#include "bsp_pwm.h"



namespace bsp {

enum class BuzzerNote {
    Do1L = 262,  ///*261.63Hz*/    3822us
    Re2L = 294,  ///*293.66Hz*/    3405us
    Mi3L = 330,  ///*329.63Hz*/    3034us
    Fa4L = 349,  ///*349.23Hz*/    2863us
    So5L = 392,  ///*392.00Hz*/    2551us
    La6L = 440,  ///*440.00Hz*/    2272us
    Si7L = 494,  ///*493.88Hz*/    2052us

    Do1M = 523,  ///*523.25Hz*/    1911us
    Re2M = 587,  ///*587.33Hz*/    1703us
    Mi3M = 659,  ///*659.26Hz*/    1517us
    Fa4M = 698,  ///*698.46Hz*/    1432us
    So5M = 784,  ///*784.00Hz*/    1276us
    La6M = 880,  ///*880.00Hz*/    1136us
    Si7M = 988,  ///*987.77Hz*/    1012us

    Do1H = 1047,  ///*1046.50Hz*/   956us
    Re2H = 1175,  ///*1174.66Hz*/   851us
    Mi3H = 1319,  ///*1318.51Hz*/   758us
    Fa4H = 1397,  ///*1396.91Hz*/   716us
    So5H = 1568,  ///*1567.98Hz*/   638us
    La6H = 1760,  ///*1760.00Hz*/   568us
    Si7H = 1976,  ///*1975.53Hz*/   506us

    Silent = 0,
    Finish = -1,
   NOTE_B0  = 31,
   NOTE_C1  = 33,
   NOTE_CS1 = 35,
   NOTE_D1  = 37,
   NOTE_DS1 = 39,
   NOTE_E1  = 41,
   NOTE_F1  = 44,
   NOTE_FS1 = 46,
   NOTE_G1  = 49,
   NOTE_GS1 = 52,
   NOTE_A1  = 55,
   NOTE_AS1 = 58,
   NOTE_B1  = 62,
   NOTE_C2  = 65,
   NOTE_CS2 = 69,
   NOTE_D2  = 73,
   NOTE_DS2 = 78,
   NOTE_E2  = 82,
   NOTE_F2  = 87,
   NOTE_FS2 = 93,
   NOTE_G2  = 98,
   NOTE_GS2 = 104,
   NOTE_A2  = 110,
   NOTE_AS2 = 117,
   NOTE_B2  = 123,
   NOTE_C3  = 131,
   NOTE_CS3 = 139,
   NOTE_D3  = 147,
   NOTE_DS3 = 156,
   NOTE_E3  = 165,
   NOTE_F3  = 175,
   NOTE_FS3 = 185,
   NOTE_G3  = 196,
   NOTE_GS3 = 208,
   NOTE_A3  = 220,
   NOTE_AS3 = 233,
   NOTE_B3  = 247,
   NOTE_C4  = 262,
   NOTE_CS4 = 277,
   NOTE_D4  = 294,
   NOTE_DS4 = 311,
   NOTE_E4  = 330,
   NOTE_F4  = 349,
   NOTE_FS4 = 370,
   NOTE_G4  = 392,
   NOTE_GS4 = 415,
   NOTE_A4  = 440,
   NOTE_AS4 = 466,
   NOTE_B4  = 494,
   NOTE_C5  = 523,
   NOTE_CS5 = 554,
   NOTE_D5  = 587,
   NOTE_DS5 = 622,
   NOTE_E5  = 659,
   NOTE_F5  = 698,
   NOTE_FS5 = 740,
   NOTE_G5  = 784,
   NOTE_GS5 = 831,
   NOTE_A5  = 880,
   NOTE_AS5 = 932,
   NOTE_B5  = 988,
   NOTE_C6  = 1047,
   NOTE_CS6 = 1109,
   NOTE_D6  = 1175,
   NOTE_DS6 = 1245,
   NOTE_E6  = 1319,
   NOTE_F6  = 1397,
   NOTE_FS6 = 1480,
   NOTE_G6  = 1568,
   NOTE_GS6 = 1661,
   NOTE_A6  = 1760,
   NOTE_AS6 = 1865,
   NOTE_B6  = 1976,
   NOTE_C7  = 2093,
   NOTE_CS7 = 2217,
   NOTE_D7  = 2349,
   NOTE_DS7 = 2489,
   NOTE_E7  = 2637,
   NOTE_F7  = 2794,
   NOTE_FS7 = 2960,
   NOTE_G7  = 3136,
   NOTE_GS7 = 3322,
   NOTE_A7  = 3520,
   NOTE_AS7 = 3729,
   NOTE_B7  = 3951,
   NOTE_C8  = 4186,
   NOTE_CS8 = 4435,
   NOTE_D8  = 4699,
   NOTE_DS8 = 4978
};

struct BuzzerNoteDelayed {
  BuzzerNote note;
  uint32_t delay;
};

typedef void (*buzzer_delay_t)(uint32_t milli);

class Buzzer {
 public:
  /**
   * @brief constructor for a buzzer instance
   *
   * @param htim       hal timer handle
   * @param channel    timer channel associated with the timer choose from [1,
   * 2, 3, 4]
   * @param clock_freq clock frequency associated with the timer
   */
  Buzzer(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t clock_freq);

  /**
   * @brief sing a single tone indefinitely long
   *
   * @param note  note frequency to sing
   */
  void SingTone(const BuzzerNote& note);

  /**
   * @brief sing a sequence of delayed notes
   *
   * @param notes       pointer to an array of delayed notes
   * @param delay_func  a void function that can delay arbitrary number of
   * milliseconds,
   *                    defaults to HAL_Delay implementation
   */
  void SingSong(
      const BuzzerNoteDelayed* notes,
      buzzer_delay_t delay_func = [](uint32_t milli) { HAL_Delay(milli); });

 private:
  /* pwm instance associated with the buzzer */
  PWM pwm_;
};

} /* namespace bsp */
