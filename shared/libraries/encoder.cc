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

#include "encoder.h"

#include "arm_math.h"

using namespace bsp;

namespace control {

//==================================================================================================
// BRT Encoder
//==================================================================================================

static void can_encoder_callback(const uint8_t data[], void* args) {
  BRTEncoder* encoder = reinterpret_cast<BRTEncoder*>(args);
  encoder->UpdateData(data);
}

BRTEncoder::BRTEncoder(CAN* can, uint16_t rx_id, bool invert) : can_(can), rx_id_(rx_id), invert_(invert) {
  can->RegisterRxCallback(rx_id, can_encoder_callback, this);
  angle_ = 0.0;
}

void BRTEncoder::UpdateData(const uint8_t data[]) {
  const uint32_t raw_angle = data[6] << 24 | data[5] << 16 | data[4] << 8 | data[3];

  constexpr float THETA_SCALE = 2 * PI / 1024;  // digital -> rad 
                                                // the maximum digital value range is [0 - 1023]
  angle_ = raw_angle * THETA_SCALE;
  connection_flag_ = true;
}

void BRTEncoder::PrintData() const { 
  print("angle: % .4f \r\n", getData());
  // print("connection: %d \r\n", connection_flag_);
}

} /* namespace control */
