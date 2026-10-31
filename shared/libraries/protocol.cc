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

#include "protocol.h"

#include <algorithm>
#include <cstring>

#include "crc_check.h"

static const uint8_t SOF = 0xA5;
static const int FRAME_HEADER_LEN = 5;
static const int CMD_ID_LEN = 2;
static const int FRAME_TAIL_LEN = 2;
static const int BYTE = 8;

namespace communication {

/* ========================================================================= */
/*       Compile-Time Static Assertions for RM2027 V1.0.0 Struct Sizes       */
/* ========================================================================= */

static_assert(sizeof(game_status_t) == 11, "game_status_t size mismatch (expected 11)");
static_assert(sizeof(game_result_t) == 1, "game_result_t size mismatch (expected 1)");
static_assert(sizeof(game_robot_HP_t) == 24, "game_robot_HP_t size mismatch (expected 24)");
static_assert(sizeof(event_data_t) == 8, "event_data_t size mismatch (expected 8)");
static_assert(sizeof(referee_warning_t) == 3, "referee_warning_t size mismatch (expected 3)");
static_assert(sizeof(dart_remaining_time_t) == 3, "dart_remaining_time_t size mismatch (expected 3)");
static_assert(sizeof(game_robot_status_t) == 17, "game_robot_status_t size mismatch (expected 17)");
static_assert(sizeof(power_heat_data_t) == 14, "power_heat_data_t size mismatch (expected 14)");
static_assert(sizeof(game_robot_pos_t) == 12, "game_robot_pos_t size mismatch (expected 12)");
static_assert(sizeof(buff_t) == 7, "buff_t size mismatch (expected 7)");
static_assert(sizeof(robot_hurt_t) == 7, "robot_hurt_t size mismatch (expected 7)");
static_assert(sizeof(shoot_data_t) == 7, "shoot_data_t size mismatch (expected 7)");
static_assert(sizeof(projectile_allowance_t) == 8, "projectile_allowance_t size mismatch (expected 8)");
static_assert(sizeof(rfid_status_t) == 6, "rfid_status_t size mismatch (expected 6)");
static_assert(sizeof(dart_client_cmd_t) == 6, "dart_client_cmd_t size mismatch (expected 6)");
static_assert(sizeof(ground_robot_position_t) == 40, "ground_robot_position_t size mismatch (expected 40)");
static_assert(sizeof(radar_mark_data_t) == 2, "radar_mark_data_t size mismatch (expected 2)");
static_assert(sizeof(sentry_info_t) == 14, "sentry_info_t size mismatch (expected 14)");
static_assert(sizeof(radar_info_t) == 8, "radar_info_t size mismatch (expected 8)");
static_assert(sizeof(graphic_data_t) == 15, "graphic_data_t size mismatch (expected 15)");
static_assert(sizeof(interaction_figure_t) == 15, "interaction_figure_t size mismatch (expected 15)");
static_assert(sizeof(graphic_delete_t) == 8, "graphic_delete_t size mismatch (expected 8)");
static_assert(sizeof(graphic_single_t) == 21, "graphic_single_t size mismatch (expected 21)");
static_assert(sizeof(graphic_double_t) == 36, "graphic_double_t size mismatch (expected 36)");
static_assert(sizeof(graphic_five_t) == 81, "graphic_five_t size mismatch (expected 81)");
static_assert(sizeof(graphic_seven_t) == 111, "graphic_seven_t size mismatch (expected 111)");
static_assert(sizeof(graphic_character_t) == 51, "graphic_character_t size mismatch (expected 51)");
static_assert(sizeof(sentry_decision_cmd_t) == 10, "sentry_decision_cmd_t size mismatch (expected 10)");
static_assert(sizeof(radar_decision_cmd_t) == 19, "radar_decision_cmd_t size mismatch (expected 19)");
static_assert(sizeof(map_command_t) == 12, "map_command_t size mismatch (expected 12)");
static_assert(sizeof(map_robot_data_t) == 48, "map_robot_data_t size mismatch (expected 48)");
static_assert(sizeof(map_data_t) == 105, "map_data_t size mismatch (expected 105)");
static_assert(sizeof(custom_info_t) == 34, "custom_info_t size mismatch (expected 34)");
static_assert(sizeof(vtm_custom_terminal_tx_t) == 300, "vtm_custom_terminal_tx_t size mismatch (expected 300)");
static_assert(sizeof(vtm_custom_terminal_rx_t) == 60, "vtm_custom_terminal_rx_t size mismatch (expected 60)");
static_assert(sizeof(radar_enemy_pos_t) == 24, "radar_enemy_pos_t size mismatch (expected 24)");
static_assert(sizeof(radar_enemy_hp_t) == 12, "radar_enemy_hp_t size mismatch (expected 12)");
static_assert(sizeof(radar_enemy_allowance_t) == 10, "radar_enemy_allowance_t size mismatch (expected 10)");
static_assert(sizeof(radar_enemy_macro_t) == 12, "radar_enemy_macro_t size mismatch (expected 12)");
static_assert(sizeof(radar_enemy_buffs_t) == 47, "radar_enemy_buffs_t size mismatch (expected 47)");
static_assert(sizeof(radar_enemy_key_t) == 6, "radar_enemy_key_t size mismatch (expected 6)");

/* ========================================================================= */
/*                              Protocol Base                                */
/* ========================================================================= */

bool Protocol::Receive(package_t package) {
  /* A failed read reports a negative length and callers pass their raw read
   * result through, so neither that nor an over-long burst may reach the copy below. */
  if (package.data == nullptr || package.length <= 0 || package.length > MAX_FRAME_LEN) {
    return false;
  }
  connection_flag_ = true;
  memcpy(bufferRx, package.data, package.length);

  int idx = 0;
  const int min_frame_len = FRAME_HEADER_LEN + CMD_ID_LEN + FRAME_TAIL_LEN;
  while (idx <= package.length - min_frame_len) {
    if (bufferRx[idx] != SOF) {
      idx++;
      continue;
    }

    // Verify CRC8 on the 5-byte header
    if (!VerifyHeader(bufferRx + idx, FRAME_HEADER_LEN)) {
      idx++;
      continue;
    }

    // Header verified: extract payload length from header bytes [1..2] (little-endian)
    int data_length = (bufferRx[idx + 2] << BYTE) | bufferRx[idx + 1];
    int total_frame_len = FRAME_HEADER_LEN + CMD_ID_LEN + data_length + FRAME_TAIL_LEN;

    // Check bounds against buffer limits and package boundary
    if (total_frame_len > MAX_FRAME_LEN || idx + total_frame_len > package.length) {
      idx++;
      continue;
    }

    // Verify CRC16 over the whole frame
    if (VerifyFrame(bufferRx + idx, total_frame_len)) {
      int cmd_id = (bufferRx[idx + FRAME_HEADER_LEN + 1] << BYTE) |
                   bufferRx[idx + FRAME_HEADER_LEN];
      ProcessDataRx(cmd_id, bufferRx + idx + FRAME_HEADER_LEN + CMD_ID_LEN, data_length);
      // Advance by total frame length: prevents false re-sync if payload contains 0xA5
      idx += total_frame_len;
    } else {
      idx++;
    }
  }
  return true;
}

package_t Protocol::Transmit(int cmd_id) {
  bufferTx[0] = SOF;
  int DATA_LENGTH = ProcessDataTx(cmd_id, bufferTx + FRAME_HEADER_LEN + CMD_ID_LEN);
  if (DATA_LENGTH < 0 ||
      FRAME_HEADER_LEN + CMD_ID_LEN + DATA_LENGTH + FRAME_TAIL_LEN > MAX_FRAME_LEN) {
    return package_t{nullptr, 0};
  }
  bufferTx[1] = (uint8_t)((uint32_t)DATA_LENGTH & 0xFF);
  bufferTx[2] = (uint8_t)((uint32_t)DATA_LENGTH >> BYTE);
  bufferTx[3] = (uint8_t)(seq++);
  AppendHeader(bufferTx, FRAME_HEADER_LEN);
  bufferTx[5] = (uint8_t)((uint32_t)cmd_id & 0xFF);
  bufferTx[6] = (uint8_t)((uint32_t)cmd_id >> BYTE);
  AppendFrame(bufferTx, FRAME_HEADER_LEN + CMD_ID_LEN + DATA_LENGTH + FRAME_TAIL_LEN);
  return package_t{bufferTx, FRAME_HEADER_LEN + CMD_ID_LEN + DATA_LENGTH + FRAME_TAIL_LEN};
}

bool Protocol::VerifyHeader(const uint8_t* data, int length) {
  return verify_crc8_check_sum(data, length);
}

bool Protocol::VerifyFrame(const uint8_t* data, int length) {
  return verify_crc16_check_sum(data, length);
}

void Protocol::AppendHeader(uint8_t* data, int length) {
  append_crc8_check_sum(data, length);
}

void Protocol::AppendFrame(uint8_t* data, int length) {
  append_crc16_check_sum(data, length);
}

/* ========================================================================= */
/*                          Referee Implementation                           */
/* ========================================================================= */

bool Referee::ProcessDataRx(int cmd_id, const uint8_t* data, int length) {
  switch (cmd_id) {
    case GAME_STATUS:
      if (length <= (int)sizeof(game_status)) memcpy(&game_status, data, length);
      break;
    case GAME_RESULT:
      if (length <= (int)sizeof(game_result)) memcpy(&game_result, data, length);
      break;
    case GAME_ROBOT_HP:
      if (length <= (int)sizeof(game_robot_HP)) memcpy(&game_robot_HP, data, length);
      break;
    case EVENT_DATA:
      if (length <= (int)sizeof(event_data)) memcpy(&event_data, data, length);
      break;
    case SUPPLY_PROJECTILE_ACTION:
      if (length <= (int)sizeof(supply_projectile_action))
        memcpy(&supply_projectile_action, data, length);
      break;
    case REFEREE_WARNING:
      if (length <= (int)sizeof(referee_warning)) memcpy(&referee_warning, data, length);
      break;
    case DART_REMAINING_TIME:
      if (length <= (int)sizeof(dart_remaining_time)) memcpy(&dart_remaining_time, data, length);
      break;
    case GAME_ROBOT_STATUS:
      if (length <= (int)sizeof(game_robot_status)) memcpy(&game_robot_status, data, length);
      break;
    case POWER_HEAT_DATA:
      if (length <= (int)sizeof(power_heat_data)) memcpy(&power_heat_data, data, length);
      break;
    case GAME_ROBOT_POS:
      if (length <= (int)sizeof(game_robot_pos)) memcpy(&game_robot_pos, data, length);
      break;
    case BUFF:
      if (length <= (int)sizeof(buff)) memcpy(&buff, data, length);
      break;
    case AERIAL_ROBOT_ENERGY:
      if (length <= (int)sizeof(aerial_robot_energy)) memcpy(&aerial_robot_energy, data, length);
      break;
    case ROBOT_HURT:
      if (length <= (int)sizeof(robot_hurt)) memcpy(&robot_hurt, data, length);
      break;
    case SHOOT_DATA:
      if (length <= (int)sizeof(shoot_data)) memcpy(&shoot_data, data, length);
      break;
    case BULLET_REMAINING:
      if (length <= (int)sizeof(bullet_remaining)) memcpy(&bullet_remaining, data, length);
      break;
    case RFID_STATUS:
      if (length <= (int)sizeof(rfid_status)) memcpy(&rfid_status, data, length);
      break;
    case DART_CLIENT_CMD:
      if (length <= (int)sizeof(dart_client_cmd)) memcpy(&dart_client_cmd, data, length);
      break;
    case GROUND_ROBOT_POSITION:
      if (length <= (int)sizeof(ground_robot_position))
        memcpy(&ground_robot_position, data, length);
      break;
    case RADAR_MARK_PROGRESS:
      if (length <= (int)sizeof(radar_mark_progress)) memcpy(&radar_mark_progress, data, length);
      break;
    case SENTRY_INFO:
      if (length <= (int)sizeof(sentry_info)) memcpy(&sentry_info, data, length);
      break;
    case RADAR_INFO:
      if (length <= (int)sizeof(radar_info)) memcpy(&radar_info, data, length);
      break;
    case MAP_COMMAND:
      if (length <= (int)sizeof(map_command)) memcpy(&map_command, data, length);
      break;
    case MAP_ROBOT_DATA:
      if (length <= (int)sizeof(map_robot_data)) memcpy(&map_robot_data, data, length);
      break;
    case MAP_DATA:
      if (length <= (int)sizeof(map_data)) memcpy(&map_data, data, length);
      break;
    case CUSTOM_INFO:
      if (length <= (int)sizeof(custom_info)) memcpy(&custom_info, data, length);
      break;
    case VTM_CUSTOM_TERMINAL_RX:
      if (length <= (int)sizeof(vtm_custom_terminal_rx))
        memcpy(&vtm_custom_terminal_rx, data, length);
      break;
    case RADAR_ENEMY_POS:
      if (length <= (int)sizeof(radar_enemy_pos)) memcpy(&radar_enemy_pos, data, length);
      break;
    case RADAR_ENEMY_HP:
      if (length <= (int)sizeof(radar_enemy_hp)) memcpy(&radar_enemy_hp, data, length);
      break;
    case RADAR_ENEMY_ALLOWANCE:
      if (length <= (int)sizeof(radar_enemy_allowance))
        memcpy(&radar_enemy_allowance, data, length);
      break;
    case RADAR_ENEMY_MACRO:
      if (length <= (int)sizeof(radar_enemy_macro)) memcpy(&radar_enemy_macro, data, length);
      break;
    case RADAR_ENEMY_BUFFS:
      if (length <= (int)sizeof(radar_enemy_buffs)) memcpy(&radar_enemy_buffs, data, length);
      break;
    case RADAR_ENEMY_KEY:
      if (length <= (int)sizeof(radar_enemy_key)) memcpy(&radar_enemy_key, data, length);
      break;
    case STUDENT_INTERACTIVE:
      if (length <= (int)sizeof(robot_interactive_data))
        memcpy(&robot_interactive_data, data, length);
      break;
    default:
      return false;
  }
  return true;
}

int Referee::ProcessDataTx(int cmd_id, uint8_t* data) {
  int data_len = -1;
  switch (cmd_id) {
    case STUDENT_INTERACTIVE: {
      switch (graph_content_) {
        case NO_GRAPH:
          data_len = -1;
          break;
        case DELETE_GRAPH:
          data_len = sizeof(graphic_delete_t);
          memcpy(data, &graphic_delete, data_len);
          break;
        case SINGLE_GRAPH:
          data_len = sizeof(graphic_single_t);
          memcpy(data, &graphic_single, data_len);
          break;
        case DOUBLE_GRAPH:
          data_len = sizeof(graphic_double_t);
          memcpy(data, &graphic_double, data_len);
          break;
        case FIVE_GRAPH:
          data_len = sizeof(graphic_five_t);
          memcpy(data, &graphic_five, data_len);
          break;
        case SEVEN_GRAPH:
          data_len = sizeof(graphic_seven_t);
          memcpy(data, &graphic_seven, data_len);
          break;
        case CHAR_GRAPH:
          data_len = sizeof(graphic_character_t);
          memcpy(data, &graphic_character, data_len);
          break;
        case SENTRY_CMD_GRAPH:
          data_len = sizeof(sentry_decision_cmd_t);
          memcpy(data, &sentry_decision_cmd, data_len);
          break;
        case RADAR_CMD_GRAPH:
          data_len = sizeof(radar_decision_cmd_t);
          memcpy(data, &radar_decision_cmd, data_len);
          break;
        case CUSTOM_DATA_GRAPH:
          data_len = sizeof(robot_interactive_data_t);
          memcpy(data, &robot_interactive_data, data_len);
          break;
        default:
          data_len = -1;
      }
      graph_content_ = NO_GRAPH;
      break;
    }
    case MAP_COMMAND:
      data_len = sizeof(map_command_t);
      memcpy(data, &map_command, data_len);
      break;
    case MAP_ROBOT_DATA:
      data_len = sizeof(map_robot_data_t);
      memcpy(data, &map_robot_data, data_len);
      break;
    case MAP_DATA:
      data_len = sizeof(map_data_t);
      memcpy(data, &map_data, data_len);
      break;
    case CUSTOM_INFO:
      data_len = sizeof(custom_info_t);
      memcpy(data, &custom_info, data_len);
      break;
    case VTM_CUSTOM_TERMINAL_TX:
      data_len = sizeof(vtm_custom_terminal_tx_t);
      memcpy(data, &vtm_custom_terminal_tx, data_len);
      break;
    default:
      data_len = -1;
  }
  return data_len;
}

void Referee::PrepareUIContent(content graph_content) {
  graph_content_ = graph_content;
}

/* ========================================================================= */
/*                            Host Implementation                            */
/* ========================================================================= */

bool Host::ProcessDataRx(int cmd_id, const uint8_t* data, int length) {
  switch (cmd_id) {
    case PACK:
      if (length <= (int)sizeof(pack)) memcpy(&pack, data, length);
      break;
    case TARGET_ANGLE:
      if (length <= (int)sizeof(target_angle)) memcpy(&target_angle, data, length);
      break;
    case NO_TARGET_FLAG:
      if (length <= (int)sizeof(no_target_flag)) memcpy(&no_target_flag, data, length);
      break;
    case SHOOT_CMD:
      if (length <= (int)sizeof(shoot_cmd)) memcpy(&shoot_cmd, data, length);
      break;
    default:
      return false;
  }
  return true;
}

int Host::ProcessDataTx(int cmd_id, uint8_t* data) {
  int data_len;
  switch (cmd_id) {
    case PACK:
      data_len = sizeof(pack_t);
      memcpy(data, &pack, data_len);
      break;
    default:
      data_len = -1;
  }
  return data_len;
}

} /* namespace communication */
