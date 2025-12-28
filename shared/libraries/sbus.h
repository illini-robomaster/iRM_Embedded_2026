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

#include "bsp_uart.h"

namespace remote {

class SBUS : public bsp::UART {
 public:
  /**
   * @brief intialize SBUS the same way as a generic UART peripheral
   * @note like uart, sbus needs time to initialize
   *
   * @param huart uart instance
   */
  SBUS(UART_HandleTypeDef* huart);

  // Add custom rx data handler
  void RxCompleteCallback() override final;

  // rocker channel information
  int16_t ch[16] = {0};

  // timestamp of the update interrupt
  uint32_t timestamp;

  volatile bool connection_flag_ = false;
  const float CHANNEL_MIN = -1024;
  const float CHANNEL_MAX = 1024;
};

} /* namespace remote */
