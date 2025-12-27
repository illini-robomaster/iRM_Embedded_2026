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

#include "hrb_supercap.h"

#include <cstring>

namespace control {

static void supercap_callback(const uint8_t data[], void* args) {
  HRB_SuperCap* supercap = reinterpret_cast<HRB_SuperCap*>(args);
  supercap->ReceiveData(data);
}

HRB_SuperCap::HRB_SuperCap(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id) : can_(can), rx_id_(rx_id), tx_id_(tx_id){
  can->RegisterRxCallback(rx_id, supercap_callback, this);
}

void HRB_SuperCap::ReceiveData(const uint8_t* data) {
  memcpy(&info, data, sizeof(cap_recv_message_t));
  connection_flag_ = true;
}

void HRB_SuperCap::SendData(cap_send_message_t data){
    can_->Transmit(tx_id_, reinterpret_cast<uint8_t*>(&data), sizeof(cap_send_message_t));
}

}  // namespace control
