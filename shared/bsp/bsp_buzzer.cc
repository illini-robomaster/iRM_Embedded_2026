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

#include "bsp_buzzer.h"

namespace bsp {

Buzzer::Buzzer(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t clock_freq)
    : pwm_(htim, channel, clock_freq, 0, 0) {
  pwm_.Start();
}

void Buzzer::SingTone(const BuzzerNote& note) {
  if (note == BuzzerNote::Finish) {
    return;
  }

  uint32_t freq = static_cast<uint32_t>(note);
  if (freq == 0) {
    // Silent: just set pulse width to 0 (no output)
    pwm_.SetPulseWidth(0);
  } else {
    pwm_.SetFrequency(freq);
    pwm_.SetPulseWidth(1000000 / freq / 2);  // 50% duty cycle
  }
}

void Buzzer::SingSong(const BuzzerNoteDelayed* delayed_notes, buzzer_delay_t delay_func) {
  while (delayed_notes->note != BuzzerNote::Finish) {
    SingTone(delayed_notes->note);
    delay_func(delayed_notes->delay);
    ++delayed_notes;
  }
}

} /* namespace bsp */
