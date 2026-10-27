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

#include <cstddef>

#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "crc8.h"

namespace communication {

// WARNING: CRC8 works only for total length < 64 bytes.
typedef struct {
  char header[2];
  uint16_t seq_num;
  // data_length might not be necessary because all command length are fixed
  uint8_t data_length; // length of data as in bytes, data length as an arry * 4 = data_length
  uint8_t cmd_id;
  int32_t *data;
  uint8_t crc8_checksum;
  char tail[2];
} __packed minipc_data_t;
// This might be not necessary at all

// RED is 0; BLUE is one
typedef struct {
  uint8_t my_color;
} __packed color_data_t;

typedef struct {
  float rel_yaw;
  float rel_pitch;
  // Search Target is 0. Move Yoke is 1.
  uint8_t mode;
  uint8_t debug_int;
} __packed gimbal_data_t;

typedef struct {
  float vx;
  float vy;
  float vw;
} __packed chassis_data_t;

typedef struct {
  // FLUSH is 0. ECHO is 1. ID is 2.
  uint8_t mode;
  uint8_t debug_int;
} __packed selfcheck_data_t;

typedef struct {
  float floats[6];
} __packed arm_data_t;

// Atomic STM32 -> Jetson board-state image shared by UART and Classical CAN.
// Motor yaw/pitch are intentionally not duplicated; orientation is carried by q.
constexpr uint8_t MODE_LEN = 1;
typedef struct {
  float q[4];  // IMU attitude, w x y z
  float bullet_speed;
  uint8_t mode[MODE_LEN];  // opaque
} __packed board_state_t;

static_assert(sizeof(board_state_t) == 21, "board_state_t layout must stay 21 bytes");
static_assert(offsetof(board_state_t, q) == 0, "board_state_t.q offset changed");
static_assert(offsetof(board_state_t, bullet_speed) == 16, "board_state_t.bullet_speed offset changed");
static_assert(offsetof(board_state_t, mode) == 20, "board_state_t.mode offset changed");

// summary of all information transmitted between minipc and stm32
typedef struct {
  // RED is 0; BLUE is one
  uint8_t my_color;
  float rel_yaw;
  float rel_pitch;
  // Search Target is 0. Move Yoke is 1.
  uint8_t mode;
  uint8_t debug_int;
  float vx;
  float vy;
  float vw;
  float floats[6];
} __packed status_data_t;

// GIMBAL_CMD_ID  : 0x00 Autoaim gimbal RelYaw RelPitch
// COLOR_CMD_ID   : 0x01
// CHASSIS_CMD_ID : 0x02
// SELFCHECK_CMD_ID :0x03
// ARM_CMD-ID     : 0x04
// TOTAL_NUM_OF_ID: length of the enum
enum CMD_ID {GIMBAL_CMD_ID,
             COLOR_CMD_ID,
             CHASSIS_CMD_ID,
             SELFCHECK_CMD_ID,
             ARM_CMD_ID,
             TOTAL_NUM_OF_ID};

// WARNING: THIS CLASS IS NOT THREAD SAFE!!!
// See docs/comm_protocol.md in vision repo for docs
class MinipcPort {
 public:
  MinipcPort();

  /**
   * @brief Pack data into a packet array
   * @note For the smallest length of the packet, see CMD_TO_LEN[] or GetPacketLength()
   */
  void Pack(uint8_t* packet, void* data, uint8_t cmd_id);
  void PackGimbalData(uint8_t* packet, gimbal_data_t* data);
  void PackColorData(uint8_t* packet, color_data_t* data);
  void PackChassisData(uint8_t* packet, chassis_data_t* data);
  void PackSelfcheckData(uint8_t* packet, selfcheck_data_t* data);
  void PackArmData(uint8_t* packet, arm_data_t* data);

  /**
   * @brief Total length of packet in bytes
   *  Header/tail/crc8 included.
   */
  uint8_t GetPacketLen(uint8_t cmd_id);

  /**
   * @brief parse and handle the uart buffer
   * @param data: pointer to the uart buffer
   *        len:  length of the data received in bytes
   */
  void ParseUartBuffer(const uint8_t* data, int32_t len);

  /**
   * @brief Return the cmd_id of the most recently parsed packet
   */
  uint8_t GetCmdId(void);

  /**
   * @brief Get command status of the robot
   */
  const status_data_t* GetStatus(void);

  /**
   * @brief Get the valid flag, 1 when the packet is valid, 0 otherwise
   * @note  Flag can only be acquired once. Once asked, flag will be reset to 0 (invalid)
   */
  uint8_t GetValidFlag(void);
  uint16_t GetSeqnum(void);
  uint32_t GetValidPacketCnt(void);

  /**
   * Length of the data section ONLY in bytes. Header/tail/crc8 (total len = 9) NOT included.
   * Gimbal    CMD: id = 0x00, total packet length = 19 - 9 = 10
   * Color     CMD: id = 0x01, total packet length = 10 - 9 = 1
   * Chassis   CMD: id = 0x02, total packet length = 21 - 9 = 12
   * Selfcheck CMD: id = 0x03, total packet length = 11 - 9 = 2
   * Arm       CMD: id = 0x04, total packet length = 33 - 9 = 24
   */
  static constexpr uint8_t CMD_TO_LEN[TOTAL_NUM_OF_ID] = {
                                                sizeof(gimbal_data_t),
                                                sizeof(color_data_t),
                                                sizeof(chassis_data_t),
                                                sizeof(selfcheck_data_t),
                                                sizeof(arm_data_t),
                                                };
  static constexpr uint8_t MAX_PACKET_LENGTH = 33;
  static constexpr uint8_t MIN_PACKET_LENGTH = 10;
  // sum of header and tail = 9. Total packet length = data length (CMD_TO_LEN) + 9
  static constexpr uint8_t HT_LEN = 9;

 private:
  /**
   * @brief Add header and tail to the packet array based on cmd_id
   * @note For the smallest length of the packet, see CMD_TO_LEN[]
   *       The sum of length for header and tail is 8 bytes
   */
  void AddHeaderTail (uint8_t* packet, uint8_t cmd_id);

  /**
   * @brief Add CRC8 checksum for the packet array based on cmd_id
   *       CRC8 calulated based on the entire array except the tail ('ED')
   * @note Only call this function after packet has data and header/tails written
   *       The length of CRC8 checksum is 1 byte
   */
  void AddCRC8 (uint8_t* packet, int8_t cmd_id);

  // Wrapper of ParseData(uint8_t), do some verification.
  void VerifyAndParseData();

  // Assume that the possible_packet is a complete and verified message
  void ParseData(uint8_t cmd_id);

  uint8_t cmd_id_;
  status_data_t status_;
  uint8_t possible_packet[MAX_PACKET_LENGTH];
  // keep track of the index of the current packet
  // in case of 1 packet being sent in multiple uart transmissions
  int buffer_index_;

  // Least current available sequence number
  uint16_t seqnum_;
  uint8_t valid_flag_;
  uint32_t valid_packet_cnt_ = 0;
}; /* class MinipcPort */

} /* namespace communication */

