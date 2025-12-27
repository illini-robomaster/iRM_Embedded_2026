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

#include "bsp_pwm.h"

#include "bsp_error_handler.h"
#include "cmsis_os.h"

namespace bsp {

PWM::PWM(TIM_HandleTypeDef* htim, uint8_t channel, uint32_t clock_freq, uint32_t output_freq,
         uint32_t pulse_width)
    : htim_(htim), clock_freq_(clock_freq), output_freq_(output_freq), pulse_width_(pulse_width) {
  switch (channel) {
    case 1:
      channel_ = TIM_CHANNEL_1;
      break;
    case 2:
      channel_ = TIM_CHANNEL_2;
      break;
    case 3:
      channel_ = TIM_CHANNEL_3;
      break;
    case 4:
      channel_ = TIM_CHANNEL_4;
      break;
    default:
      bsp_error_handler(__FUNCTION__, __LINE__, "pwm channel not valid");
  }
  SetFrequency(output_freq);
  SetPulseWidth(pulse_width);
}

void PWM::Start() {
  HAL_TIM_PWM_Start(htim_, channel_);
  // For advanced timers (TIM1, TIM8, TIM15, TIM16, TIM17), enable Main Output (MOE)
  // This is required for the PWM signal to actually appear on the pin
  if (IS_TIM_BREAK_INSTANCE(htim_->Instance)) {
    __HAL_TIM_MOE_ENABLE(htim_);
  }
}

void PWM::Stop() {
  HAL_TIM_PWM_Stop(htim_, channel_);
  // Disable Main Output for advanced timers
  if (IS_TIM_BREAK_INSTANCE(htim_->Instance)) {
    __HAL_TIM_MOE_DISABLE(htim_);
  }
}

void PWM::SetFrequency(uint32_t output_freq) {
  this->output_freq_ = output_freq;
  uint32_t auto_reload = output_freq > 0 ? clock_freq_ / output_freq_ - 1 : 0;
  __HAL_TIM_SET_AUTORELOAD(htim_, auto_reload);
  __HAL_TIM_SET_COUNTER(htim_, 0);
}

void PWM::SetPulseWidth(uint32_t pulse_width) {
  this->pulse_width_ = pulse_width;
  // Use 64-bit arithmetic to avoid overflow
  // pulse_width is in microseconds, clock_freq_ is in Hz
  uint32_t compare = pulse_width > 0
                         ? static_cast<uint32_t>((uint64_t)clock_freq_ * pulse_width_ / 1000000) - 1
                         : 0;
  __HAL_TIM_SET_COMPARE(htim_, channel_, compare);
}

} /* namespace bsp */
