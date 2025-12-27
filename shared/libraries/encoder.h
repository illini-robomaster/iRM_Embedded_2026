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

#include "bsp_can.h"
#include "controller.h"

namespace control {

//==================================================================================================
// BRT Encoder
//==================================================================================================

/**
 * @brief BRT Encoder class
 */
class BRTEncoder {
 public:
  /* constructor wrapper over MotorCANBase */
  BRTEncoder(bsp::CAN* can, uint16_t rx_id, bool invert);
  /* implements data update callback */
  void UpdateData(const uint8_t data[]);
  /* implements data printout */
  float getData() const {
    return invert_ ? -angle_ : angle_;
  }
  void setInvert(bool inv){
    invert_ = inv;
  }
  void PrintData() const;
  bool is_connected(){
    return connection_flag_;
  }

 private:
  bsp::CAN* can_;
  float angle_;
  uint16_t rx_id_;
  bool invert_;
  bool connection_flag_ = false;
};

} /* namespace control */
