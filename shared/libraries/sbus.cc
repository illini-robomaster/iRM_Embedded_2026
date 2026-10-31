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

#include "sbus.h"

#include <cmath>

#include "bsp_error_handler.h"

/* rocker range and dead-zones */
#define RC_ROCKER_MID 1024
#define RC_ROCKER_ZERO_DRIFT 20  // Range of possible drift around initial position

static const int SBUS_RX_LEN = 25;
static const uint8_t START_BYTE = 0x0f;
static const uint8_t END_BYTE = 0x00;

namespace remote {

SBUS::SBUS(UART_HandleTypeDef* huart) : bsp::UART(huart) { SetupRx(SBUS_RX_LEN + 1); }

void SBUS::RxCompleteCallback() {
  connection_flag_ = true;

  uint8_t* data = nullptr;
  const int32_t length = this->Read<true>(&data);

  /* A negative length means the burst was discarded (UART reception error), a short
   * one that it is not a whole frame: neither may be decoded. */
  if (length < SBUS_RX_LEN || data == nullptr) return;

  if (data[0] != START_BYTE) return;

  this->ch[0]  = ((data[1] | data[2] << 8) & 0x07FF) - RC_ROCKER_MID;
  this->ch[1]  = ((data[2] >> 3 | data[3] << 5) & 0x07FF) - RC_ROCKER_MID;
  this->ch[2]  = ((data[3] >> 6 | data[4] << 2 | data[5] << 10) & 0x07FF) - RC_ROCKER_MID;
  this->ch[3]  = ((data[5] >> 1 | data[6] << 7) & 0x07FF) - RC_ROCKER_MID;
  this->ch[4]  = ((data[6] >> 4 | data[7] << 4) & 0x07FF) - RC_ROCKER_MID;
  this->ch[5]  = ((data[7] >> 7 | data[8] << 1 | data[9] << 9) & 0x07FF) - RC_ROCKER_MID;
  this->ch[6]  = ((data[9] >> 2 | data[10] << 6) & 0x07FF) - RC_ROCKER_MID;
  this->ch[7]  = ((data[10] >> 5 | data[11] << 3) & 0x07FF) - RC_ROCKER_MID;
  this->ch[8]  = ((data[12] | data[13] << 8) & 0x07FF) - RC_ROCKER_MID;
  this->ch[9]  = ((data[13] >> 3 | data[14] << 5) & 0x07FF) - RC_ROCKER_MID;
  this->ch[10] = ((data[14] >> 6 | data[15] << 2 | data[16] << 10) & 0x07FF) - RC_ROCKER_MID;
  this->ch[11] = ((data[16] >> 1 | data[17] << 7) & 0x07FF) - RC_ROCKER_MID;
  this->ch[12] = ((data[17] >> 4 | data[18] << 4) & 0x07FF) - RC_ROCKER_MID;
  this->ch[13] = ((data[18] >> 7 | data[19] << 1 | data[20] << 9) & 0x07FF) - RC_ROCKER_MID;
  this->ch[14] = ((data[20] >> 2 | data[21] << 6) & 0x07FF) - RC_ROCKER_MID;
  this->ch[15] = ((data[21] >> 5 | data[22] << 3) & 0x07FF) - RC_ROCKER_MID;

  this->ch[0]  = abs(this->ch[0]) <= RC_ROCKER_ZERO_DRIFT ? 0 : this->ch[0];
  this->ch[1]  = abs(this->ch[1]) <= RC_ROCKER_ZERO_DRIFT ? 0 : this->ch[1];
  this->ch[2]  = abs(this->ch[2]) <= RC_ROCKER_ZERO_DRIFT ? 0 : this->ch[2];
  this->ch[3]  = abs(this->ch[3]) <= RC_ROCKER_ZERO_DRIFT ? 0 : this->ch[3];

  this->timestamp = HAL_GetTick();
}

} /* namespace remote */
