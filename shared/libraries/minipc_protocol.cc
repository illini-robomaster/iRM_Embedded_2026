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

#include "minipc_protocol.h"

#include <cstring>
#include <memory>

namespace communication {

// For definitions of constants, check out the documentation at
// illini-robomaster/iRM_Vision_2023/docs/comm_protocol.md
static constexpr uint8_t SEQNUM_OFFSET = 2;
static constexpr uint8_t DATA_LENGTH_OFFSET = SEQNUM_OFFSET + 2;
static constexpr uint8_t CMD_ID_OFFSET = DATA_LENGTH_OFFSET + 1;
static constexpr uint8_t DATA_OFFSET = CMD_ID_OFFSET + 1;

MinipcPort::MinipcPort() {
  buffer_index_ = 0; // current pointer to write
  valid_flag_ = 0;
  seqnum_ = 0;
}

void MinipcPort::Pack(uint8_t* packet, void* data, uint8_t cmd_id) {
  switch (cmd_id) {
    case GIMBAL_CMD_ID:
      PackGimbalData(packet, static_cast<gimbal_data_t*>(data));
      break;
    case COLOR_CMD_ID:
      PackColorData(packet, static_cast<color_data_t*>(data));
      break;
    case CHASSIS_CMD_ID:
      PackChassisData(packet, static_cast<chassis_data_t*>(data));
      break;
    case SELFCHECK_CMD_ID:
      PackSelfcheckData(packet, static_cast<selfcheck_data_t*>(data));
      break;
    case ARM_CMD_ID:
      PackArmData(packet, static_cast<arm_data_t*>(data));
      break;
    case BOARD_STATE_CMD_ID:
      PackBoardState(packet, static_cast<board_state_t*>(data));
      break;
  }
}

void MinipcPort::PackGimbalData(uint8_t* packet, gimbal_data_t* data) {
  AddHeaderTail(packet, GIMBAL_CMD_ID);
  memcpy(&packet[0 + DATA_OFFSET], &data->rel_yaw, sizeof(float));
  memcpy(&packet[4 + DATA_OFFSET], &data->rel_pitch, sizeof(float));
  packet[8 + DATA_OFFSET] = data->mode;
  packet[9 + DATA_OFFSET] = data->debug_int;
  AddCRC8(packet, GIMBAL_CMD_ID);
}

void MinipcPort::PackColorData(uint8_t* packet, color_data_t* data) {
  AddHeaderTail(packet, COLOR_CMD_ID);
  packet[DATA_OFFSET] = data->my_color;
  AddCRC8(packet, COLOR_CMD_ID);
}

void MinipcPort::PackChassisData(uint8_t* packet, chassis_data_t* data) {
  AddHeaderTail(packet, CHASSIS_CMD_ID);
  memcpy(&packet[0 + DATA_OFFSET], &data->vx, sizeof(float));
  memcpy(&packet[4 + DATA_OFFSET], &data->vy, sizeof(float));
  memcpy(&packet[8 + DATA_OFFSET], &data->vw, sizeof(float));
  AddCRC8(packet, CHASSIS_CMD_ID);
}

void MinipcPort::PackSelfcheckData(uint8_t* packet, selfcheck_data_t* data) {
  AddHeaderTail(packet, SELFCHECK_CMD_ID);
  packet[0 + DATA_OFFSET] = data->mode;
  packet[1 + DATA_OFFSET] = data->debug_int;
  AddCRC8(packet, SELFCHECK_CMD_ID);
}

void MinipcPort::PackArmData(uint8_t* packet, arm_data_t* data) {
  AddHeaderTail(packet, ARM_CMD_ID);
  memcpy(&packet[0 + DATA_OFFSET], &data->floats[0], sizeof(float));
  memcpy(&packet[4 + DATA_OFFSET], &data->floats[1], sizeof(float));
  memcpy(&packet[8 + DATA_OFFSET], &data->floats[2], sizeof(float));
  memcpy(&packet[12 + DATA_OFFSET], &data->floats[3], sizeof(float));
  memcpy(&packet[16 + DATA_OFFSET], &data->floats[4], sizeof(float));
  memcpy(&packet[20 + DATA_OFFSET], &data->floats[5], sizeof(float));
  AddCRC8(packet, ARM_CMD_ID);
}

void MinipcPort::PackBoardState(uint8_t* packet, board_state_t* data) {
  AddHeaderTail(packet, BOARD_STATE_CMD_ID);
  memcpy(&packet[0 + DATA_OFFSET], &data->q[0], sizeof(float));
  memcpy(&packet[4 + DATA_OFFSET], &data->q[1], sizeof(float));
  memcpy(&packet[8 + DATA_OFFSET], &data->q[2], sizeof(float));
  memcpy(&packet[12 + DATA_OFFSET], &data->q[3], sizeof(float));
  memcpy(&packet[16 + DATA_OFFSET], &data->yaw, sizeof(float));
  memcpy(&packet[20 + DATA_OFFSET], &data->pitch, sizeof(float));
  memcpy(&packet[24 + DATA_OFFSET], &data->bullet_speed, sizeof(float));
  memcpy(&packet[28 + DATA_OFFSET], data->mode, MODE_LEN);
  AddCRC8(packet, BOARD_STATE_CMD_ID);
}

uint8_t MinipcPort::GetPacketLen(uint8_t cmd_id) {
  // Total length = Data length + header & tail (8 bytes) + crc checksum (1 byte)
  return CMD_TO_LEN[cmd_id] + 9;
}

void MinipcPort::ParseUartBuffer(const uint8_t* data, int32_t length) {
  // Four cases
  // Case 1: everything is fresh with complete package(s)
  // Case 2: everything is fresh; package is incomplete
  // Case 3: package contains half previous package and half new package
  // Case 4: package contains half previous package and new package(s)
  int i = 0;
  if (buffer_index_ > 0) {
    // Case 3 and 4
    // copy the remaining bytes from previous package
    while (i < (int)length) {
      possible_packet[buffer_index_] = data[i];
      // Parse possible packet if detect tail
      if (possible_packet[buffer_index_ - 1] == 'E' &&
          possible_packet[buffer_index_]     == 'D') {
        buffer_index_ = 0;
        VerifyAndParseData();
        break;
      }
      buffer_index_++;
      i++;
      // drop packet if buffer overflow
      if (buffer_index_ >= MAX_PACKET_LENGTH) {
        buffer_index_ = 0;
        break;
      }
    }
  }

  while (i < (int)length) {
    if (buffer_index_ == 0) {
      if (i == (int)length - 1) {
        // A special case to handle the last byte; buffer_index_ must be zero
        if (data[i] == 'S') {
          possible_packet[buffer_index_++] = data[i];
        }
        return;
      }
      if (data[i] == 'S' && data[i + 1] == 'T'){
        // Detect packet head; start copying
        possible_packet[buffer_index_++] = data[i++];
        possible_packet[buffer_index_++] = data[i++];
      } else {
        i++;
      }
    } else {
      possible_packet[buffer_index_] = data[i];
      // Parse possible packet if detect tail
      if (possible_packet[buffer_index_ - 1] == 'E' &&
          possible_packet[buffer_index_]     == 'D') {
        buffer_index_ = 0;
        VerifyAndParseData();
      } else {
      // If not packet end, continue
        buffer_index_++;
        i++;
        // drop packet if buffer overflow
        if (buffer_index_ >= MAX_PACKET_LENGTH) {
          buffer_index_ = 0;
        }
      }
    }
  }
}

uint8_t MinipcPort::GetCmdId(void) {
    return cmd_id_;
}

const status_data_t* MinipcPort::GetStatus(void) {
  return &status_;
}

uint8_t MinipcPort::GetValidFlag(void) {
  uint8_t temp = valid_flag_;
  valid_flag_ = 0;
  return temp;
}

uint16_t MinipcPort::GetSeqnum(void) {
  return seqnum_;
}

uint32_t MinipcPort::GetValidPacketCnt(void) {
  return valid_packet_cnt_;
}

/**
 * Every function below is a private function
 */


// 8 bytes
void MinipcPort::AddHeaderTail (uint8_t* packet, uint8_t cmd_id) {
  // Add header
  packet[0] = 'S';
  packet[1] = 'T';
  // dummy seq num
  packet[SEQNUM_OFFSET] = 0;
  packet[SEQNUM_OFFSET + 1] = 0 >> 8;
  packet[DATA_LENGTH_OFFSET] = CMD_TO_LEN[cmd_id];
  packet[CMD_ID_OFFSET] = cmd_id;

  // Add tail WITHOUT crc8
  const int EOF_OFFSET = DATA_OFFSET + CMD_TO_LEN[cmd_id] + 1;
  packet[EOF_OFFSET] = 'E';
  packet[EOF_OFFSET + 1] = 'D';
}

// 1 bytes
void MinipcPort::AddCRC8 (uint8_t* packet, int8_t cmd_id) {
  const int CRC8_OFFSET = DATA_OFFSET + CMD_TO_LEN[cmd_id];
  packet[CRC8_OFFSET] = get_crc8_check_sum(packet, CRC8_OFFSET, 0);
}

void MinipcPort::VerifyAndParseData(void) {
  // TODO: implement thread-safe logic here (use a lock to handle changes from interrupt)
  // here we can assume that the package is complete
  // in the possible_packet buffer

  // A minor issue with current implementation: imagine the following case:
  //  Two packets arrive in two UART calls.
  //  The first packet misses 1 byte, but the second one is complete.
  //  In this case, when the possible_packet buffer is filled
  //  (the last bytes are from the second packet), VerifyAndParseData() will be called. The whole buffer
  //  would be tossed, resulting in two unusable packets. However, if we implement this logic, we would be
  //  able to recover the second packet.

  // This is a minor issue because
  //    1) we don't observe this even when packets are sent at 200Hz
  //    2) probability of this happening is very low. The second packet has to be sent in two slices to
  //       trigger this issue. (first slice: S/T is sent to possible_packet; second slide: the rest)
  // Note: The issue discussed above might be deprecated.

  // if packet Head or Tail is corrupted
  uint8_t cmd_id = possible_packet[CMD_ID_OFFSET];
  if (possible_packet[0] != 'S' || possible_packet[1] != 'T' ||
      possible_packet[CMD_TO_LEN[cmd_id] + HT_LEN - 2] != 'E' ||
      possible_packet[CMD_TO_LEN[cmd_id] + HT_LEN - 1] != 'D') {
    valid_flag_ = 0;
    return;
  }

  // if packet crc8 checksum is valid
  if (verify_crc8_check_sum(possible_packet, CMD_TO_LEN[cmd_id] + HT_LEN - 2)) {
    ParseData(cmd_id);
    valid_packet_cnt_++;
    valid_flag_ = 1;
  } else {
    valid_flag_ = 0;
  }
}



void MinipcPort::ParseData(uint8_t cmd_id) {
  // Update member variable
  cmd_id_ = cmd_id;

  // Update seqnum
  memcpy(&seqnum_, &possible_packet[SEQNUM_OFFSET], sizeof(uint16_t));

  // Update data section
  switch (cmd_id) {
    case GIMBAL_CMD_ID:
      memcpy(&(status_.rel_yaw),   &possible_packet[0 + DATA_OFFSET], sizeof(float));
      memcpy(&(status_.rel_pitch), &possible_packet[4 + DATA_OFFSET], sizeof(float));
      status_.mode = possible_packet[8 + DATA_OFFSET];
      status_.debug_int = possible_packet[9 + DATA_OFFSET];
      break;
    case COLOR_CMD_ID:
      status_.my_color = possible_packet[DATA_OFFSET];
      break;
    case CHASSIS_CMD_ID:
      memcpy(&(status_.vx), &possible_packet[0 + DATA_OFFSET], sizeof(float));
      memcpy(&(status_.vy), &possible_packet[4 + DATA_OFFSET], sizeof(float));
      memcpy(&(status_.vw), &possible_packet[8 + DATA_OFFSET], sizeof(float));
      break;
    case SELFCHECK_CMD_ID:
      status_.mode = possible_packet[0 + DATA_OFFSET];
      status_.debug_int = possible_packet[1 + DATA_OFFSET];
      break;
    case ARM_CMD_ID:
      memcpy(&(status_.floats[0]),   &possible_packet[0 + DATA_OFFSET], sizeof(float));
      memcpy(&(status_.floats[1]),   &possible_packet[4 + DATA_OFFSET], sizeof(float));
      memcpy(&(status_.floats[2]),   &possible_packet[8 + DATA_OFFSET], sizeof(float));
      memcpy(&(status_.floats[3]),   &possible_packet[12 + DATA_OFFSET], sizeof(float));
      memcpy(&(status_.floats[4]),   &possible_packet[16 + DATA_OFFSET], sizeof(float));
      memcpy(&(status_.floats[5]),   &possible_packet[20 + DATA_OFFSET], sizeof(float));
      break;
  }
}


}  // namespace communication

