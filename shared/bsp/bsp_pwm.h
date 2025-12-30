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

#include "tim.h"

namespace bsp {

class PWM {
 public:
  /**
   * @brief constructor for a pwm output manager
   *
   * @param htim         HAL timer handle
   * @param channel      channel associated with the timer, choose from [1,2,3,4]
   * @param clock_freq   clock frequency associated with the timer, in [Hz]
   * @param output_freq  desired pwm output frequency, in [Hz]
   * @param pulse_width  desired pwm output pulse width, is [us]
   */
  PWM(TIM_HandleTypeDef* htim, uint8_t channel, uint32_t clock_freq, uint32_t output_freq,
      uint32_t pulse_width);

  /**
   * @brief start pwm output signal generation
   */
  void Start();

  /**
   * @brief stop pwm output signal generation
   */
  void Stop();

  /**
   * @brief set a new pwm output frequency
   *
   * @param output_freq   desired pwm output frequency, in [Hz]
   */
  void SetFrequency(uint32_t output_freq);

  /**
   * @brief set a new pwm output frequency with duty cycle (for buzzer)
   *
   * @param output_freq   desired pwm output frequency, in [Hz]
   * @param duty_percent  duty cycle percentage (0-100)
   */
  void SetFrequencyWithDutyCycle(uint32_t output_freq, uint32_t duty_percent);

  /**
   * @brief set a new pwm output pulse width
   *
   * @param pulse_width   desired pwm output pulse width, in [us]
   */
  void SetPulseWidth(uint32_t pulse_width);

 private:
  TIM_HandleTypeDef* htim_;
  uint32_t channel_;
  uint32_t clock_freq_;
  uint32_t output_freq_;
  uint32_t pulse_width_;
};

} /* namespace bsp */
