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

#include "bsp_can_bridge.h"

#include <cstring>

namespace bsp {

static void bridge_callback(const uint8_t data[], void* args) {
  CanBridge* bridge = reinterpret_cast<CanBridge*>(args);
  bridge->UpdateData(data);
}

CanBridge::CanBridge(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id) {
  can_ = can;
  rx_id_ = rx_id;
  tx_id_ = tx_id;
  can_->RegisterRxCallback(rx_id_, bridge_callback, this);
}

void CanBridge::UpdateData(const uint8_t* data) {
  memcpy(&cmd, data, sizeof(bridge_data_t));
  switch (cmd.id) {
    case VX:
      vx = cmd.data_float;
      break;
    case VY:
      vy = cmd.data_float;
      break;
    case RELATIVE_ANGLE:
      relative_angle = cmd.data_float;
      break;
    case START:
      start = cmd.data_bool;
      break;
    case MODE:
      mode = cmd.data_int;
      break;
    case DEAD:
      dead = cmd.data_bool;
      break;
    case SHOOTER_POWER:
      shooter_power = cmd.data_bool;
      break;
    case COOLING_HEAT1:
      cooling_heat1 = cmd.data_float;
      break;
    case COOLING_HEAT2:
      cooling_heat2 = cmd.data_float;
      break;
    case COOLING_LIMIT1:
      cooling_limit1 = cmd.data_float;
      break;
    case COOLING_LIMIT2:
      cooling_limit2 = cmd.data_float;
      break;
    case SPEED_LIMIT1:
      speed_limit1 = cmd.data_float;
      break;
    case SPEED_LIMIT2:
      speed_limit2 = cmd.data_float;
      break;
    case CHASSIS_FLAG:
      chassis_flag = cmd.data_uint;
      break;
    case GIMBAL_POWER:
      gimbal_power = cmd.data_uint;
      break;
    case RECALIBRATE:
      recalibrate = cmd.data_bool;
      break;
    case IS_MY_COLOR_BLUE:
      is_my_color_blue = cmd.data_bool;
      break;
    case SELF_CHECK_FLAG:
        self_check_flag = cmd.data_bool;
      break;
    case ARM_TRANSLATE:
      arm_translate = cmd.data_float;
      break;
    case CHASSIS_POWER:
      chassis_power = cmd.data_float;
      break;
    case CHASSIS_POWER_LIMIT:
      chassis_power_limit = cmd.data_uint;
      break;
    default:;
  }
}

void CanBridge::TransmitOutput() { can_->Transmit(tx_id_, (uint8_t*)&cmd, sizeof(bridge_data_t)); }

}  // namespace bsp
