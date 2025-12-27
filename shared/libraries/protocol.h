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

#include "bsp_error_handler.h"

namespace communication {

constexpr int MAX_FRAME_LEN = 300;

typedef struct {
  uint8_t* data;
  int length;
} package_t;

class Protocol {
 public:
  /**
   * @brief update the information from referee system
   *
   * @param data      address for received data read from UART for referee
   * @param length    number of bytes in received data
   * @return true for success; false for failure
   */
  bool Receive(package_t package);

  /**
   * @brief prepare the information to be sent and zip as a package
   *
   * @param cmd_id    command id
   * @return package that includes data and length
   */
  package_t Transmit(int cmd_id);

  volatile bool connection_flag_ = false;

 private:
  int seq = 0;

  uint8_t bufferRx[MAX_FRAME_LEN] = {0};
  uint8_t bufferTx[MAX_FRAME_LEN] = {0};

  /**
   * @brief verify the header of frame with crc8
   *
   * @param data      address for header data
   * @param length    number of bytes in header data
   * @return true for success; false for failure
   */
  bool VerifyHeader(const uint8_t* data, int length);

  /**
   * @brief verify the frame with crc16
   *
   * @param data      address for frame data`
   * @param length    number of bytes in frame data
   * @return true for success; false for failure
   */
  bool VerifyFrame(const uint8_t* data, int length);

  /**
   * @brief process the data for certain command and update corresponding status variables
   *
   * @param cmd_id    command id
   * @param data      address for command data
   * @param length    number of bytes in command data
   * @return true for success; false for failure
   */
  virtual bool ProcessDataRx(int cmd_id, const uint8_t* data, int length) = 0;

  /**
   * @brief append the header of frame with crc8
   *
   * @param data        address for header data
   * @param length      number of bytes in header data
   */
  void AppendHeader(uint8_t* data, int length);

  /**
   * @brief append the frame with crc16
   *
   * @param data        address for frame data
   * @param length      number of bytes in frame data
   */
  void AppendFrame(uint8_t* data, int length);

  /**
   * @brief process the information for certain command and copy it into the buffer named as data
   *
   * @param cmd_id
   * @param data
   * @return length of the data that is copied into buffer
   */
  virtual int ProcessDataTx(int cmd_id, uint8_t* data) = 0;
};

/* Command for Referee */

/*
 * 0x0001 GAME_STATUS
 * 0x0002 GAME_RESULT
 * 0x0003 GAME_ROBOT_HP
 * 0x0005 ICRA_BUFF_DEBUFF_ZONE_STATUS [X]
 * 0x0101 EVENT_DATA
 * 0x0102 SUPPLY_PROJECTILE_ACTION
 * 0x0104 REFEREE_WARNING
 * 0x0105 DART_REMAINING_TIME
 * 0x0201 GAME_ROBOT_STATUS
 * 0x0202 POWER_HEAT_DATA
 * 0x0203 GAME_ROBOT_POS
 * 0x0204 BUFF
 * 0x0205 AERIAL_ROBOT_ENERGY
 * 0x0206 ROBOT_HURT
 * 0x0207 SHOOT_DATA
 * 0x0208 BULLET_REMAINING
 * 0x0209 RFID_STATUS
 * 0x020A DART_CLIENT_CMD
 * 0x0301 STUDENT_INTERACTIVE
 * 0x0302 ROBOT_INTERACTIVE [x]
 * 0x0303 ROBOT_COMMAND [x]
 * 0x0304 ROBOT_COMMAND [x]
 * 0x0305 CLIENT_MAP_COMMAND [x]
 */

/* TODO(neo): above information with [x] should be implemented when needed in the future*/

typedef enum {
  GAME_STATUS = 0x0001,
  GAME_RESULT = 0x0002,
  GAME_ROBOT_HP = 0x003,
  EVENT_DATA = 0x0101,
  SUPPLY_PROJECTILE_ACTION = 0x0102,
  REFEREE_WARNING = 0x0104,
  DART_REMAINING_TIME = 0x0105,
  GAME_ROBOT_STATUS = 0x0201,
  POWER_HEAT_DATA = 0x0202,
  GAME_ROBOT_POS = 0x0203,
  BUFF = 0x0204,
  AERIAL_ROBOT_ENERGY = 0x0205,
  ROBOT_HURT = 0x0206,
  SHOOT_DATA = 0x0207,
  BULLET_REMAINING = 0x0208,
  RFID_STATUS = 0x0209,
  DART_CLIENT_CMD = 0x020A,
  GROUND_ROBOT_POSITION = 0x020B,
  RADAR_MARK_PROGRESS = 0x020C,
  SENTRY_INFO = 0x020D,
  RADAR_INFO = 0x020E,
  STUDENT_INTERACTIVE = 0x0301,

} referee_cmd;

/* ===== GAME_STATUS 0x0001 1Hz ===== */
typedef struct {
  uint8_t game_type : 4;
  uint8_t game_progress : 4;
  uint16_t stage_remain_time;

  uint64_t SyncTimeStamp;
} __packed game_status_t;

/* ===== GAME_RESULT 0x0002 ===== */
typedef struct {
  uint8_t winner;
} __packed game_result_t;

/* ===== GAME_ROBOT_HP 0x0003 1Hz ===== */
typedef struct {
  uint16_t red_1_robot_HP;
  uint16_t red_2_robot_HP;
  uint16_t red_3_robot_HP;
  uint16_t red_4_robot_HP;
  uint16_t red_5_robot_HP;
  uint16_t red_6_robot_HP;
  uint16_t red_7_robot_HP;

  uint16_t red_outpost_HP;
  uint16_t red_base_HP;

  uint16_t blue_1_robot_HP;
  uint16_t blue_2_robot_HP;
  uint16_t blue_3_robot_HP;
  uint16_t blue_4_robot_HP;
  uint16_t blue_5_robot_HP;
  uint16_t blue_7_robot_HP;

  uint16_t blue_outpost_HP;
  uint16_t blue_base_HP;
} __packed game_robot_HP_t;

/* ===== EVENT_DATA 0x0101 1Hz ===== */
typedef struct {
  uint32_t event_type;
} __packed event_data_t;

/* ===== SUPPLY_PROJECTILE_ACTION 0x0102 ===== */
typedef struct {
  uint8_t supply_projectile_id;
  uint8_t supply_robot_id;
  uint8_t supply_projectile_step;
  uint8_t supply_projectile_num;
} __packed supply_projectile_action_t;

/* ===== REFEREE_WARNING 0x0104 ===== */
typedef struct {
  uint8_t level;
  uint8_t foul_robot_id;
  uint8_t count;
} __packed referee_warning_t;

/* ===== DART_REMAINING_TIME 0x0105 1Hz ===== */
typedef struct {
  uint8_t dart_remaining_time;
  uint16_t dart_info;
} __packed dart_remaining_time_t;

/* ===== GAME_ROBOT_STATUS 0x0201 10Hz ===== */
typedef struct {
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t remain_HP;
  uint16_t max_HP;

  uint16_t shooter_barrel_cooling_value;
  uint16_t shooter_barrel_heat_limit;

  uint16_t chassis_power_limit;
  uint8_t mains_power_gimbal_output : 1;
  uint8_t mains_power_chassis_output : 1;
  uint8_t mains_power_shooter_output : 1;
} __packed game_robot_status_t;

/* ===== POWER_HEAT_DATA 0x0202 50Hz ===== */
typedef struct {
  uint16_t chassis_volt;
  uint16_t chassis_current;
  float chassis_power;
  uint16_t chassis_power_buffer;
  uint16_t shooter_id1_17mm_cooling_heat;
  uint16_t shooter_id2_17mm_cooling_heat;
  uint16_t shooter_id1_42mm_cooling_heat;
} __packed power_heat_data_t;

/* ===== GAME_ROBOT_POS 0x0203 10Hz ===== */
typedef struct {
  float x;
  float y;
  float yaw;
} __packed game_robot_pos_t;

/* ===== BUFF 0x0204 1Hz ===== */
typedef struct {
  uint8_t recovery_buff;
  uint8_t cooling_buff;
  uint8_t defence_buff;
  uint8_t vulnerability_buff;
  uint16_t attack_buff;
} __packed buff_t;

/* ===== AERIAL_ROBOT_ENERGY 0x0205 10Hz ===== */
typedef struct {
  uint8_t airforce_status;
  uint8_t time_remain;
} __packed aerial_robot_energy_t;

/* ===== ROBOT_HURT 0x0206 ===== */
typedef struct {
  uint8_t armor_id : 4;
  uint8_t hurt_type : 4;
} __packed robot_hurt_t;

/* ===== SHOOT_DATA 0x0207 ===== */
typedef struct {
  uint8_t bullet_type;
  uint8_t shooter_id;
  uint8_t bullet_freq;
  float bullet_speed;
} __packed shoot_data_t;

/* ===== BULLET_REMAINING 0x0208 10Hz ===== */
typedef struct {
  uint16_t bullet_remaining_num_17mm;
  uint16_t bullet_remaining_num_42mm;
  uint16_t coin_remaining_num;
} __packed bullet_remaining_t;

/* ===== RFID_STATUS 0x0209 1Hz ===== */
typedef struct {
  uint32_t rfid_status;
} __packed rfid_status_t;

/* ===== DART_CLIENT_CMD 0x020A 10Hz ===== */
typedef struct {
  uint8_t dart_launch_opening_status;
  uint8_t dart_attack_target;
  uint16_t target_change_time;
  uint16_t operate_launch_cmd_time;
} __packed dart_client_cmd_t;


/* ====== GROUND_ROBOT_POSITION 0x020B 1Hz ====== */
typedef struct {
  float hero_x;
  float hero_y;
  float engiener_x;
  float engineer_y;
  float standard_3_x;
  float standard_3_y;
  float standard_4_x;
  float standard_4_y;
  float standard_5_x;
  float standard_5_y;

} __packed ground_robot_position_t;


/* ====== RADAR_MARK_PROGRESS 0x020C 1Hz ======= */
typedef struct 
{ 
  uint8_t mark_hero_progress; 
  uint8_t mark_engineer_progress; 
  uint8_t mark_standard_3_progress; 
  uint8_t mark_standard_4_progress; 
  uint8_t mark_standard_5_progress; 
  uint8_t mark_sentry_progress; 
} __packed radar_mark_data_t;

/* ====== SENTRY_INFO 0x020D 1Hz ======= */
typedef  struct 
{ 
  uint32_t sentry_info; 
  uint16_t sentry_info_2; 
} __packed sentry_info_t;

/* ====== RADAR_INFO 0x020E 1Hz =======  */
typedef struct {
  uint8_t radar_info;
} __packed radar_info_t;

typedef struct {
  uint16_t data_cmd_id;
  uint16_t sender_ID;
  uint16_t receiver_ID;
} __packed UI_header_data_t;

typedef struct {
  UI_header_data_t header;
  uint8_t data[];
} __packed robot_interactive_data_t;

typedef struct {
  UI_header_data_t header;
  uint8_t operate_type;
  uint8_t layer;
} __packed graphic_delete_t;

typedef struct {
  uint8_t graphic_name[3];
  uint32_t operate_type : 3;
  uint32_t graphic_type : 3;
  uint32_t layer : 4;
  uint32_t color : 4;
  uint32_t start_angle : 9;
  uint32_t end_angle : 9;
  uint32_t width : 10;
  uint32_t start_x : 11;
  uint32_t start_y : 11;
  uint32_t radius : 10;
  uint32_t end_x : 11;
  uint32_t end_y : 11;
} __packed graphic_data_t;

typedef struct {
  UI_header_data_t header;
  graphic_data_t graphic_data_struct;
} __packed graphic_single_t;

typedef struct {
  UI_header_data_t header;
  graphic_data_t graphic_data_struct[2];
} __packed graphic_double_t;

typedef struct {
  UI_header_data_t header;
  graphic_data_t graphic_data_struct[5];
} __packed graphic_five_t;

typedef struct {
  UI_header_data_t header;
  graphic_data_t graphic_data_struct[7];
} __packed graphic_seven_t;

typedef struct {
  UI_header_data_t header;
  graphic_data_t graphic_data_struct;
  uint8_t data[30];
} __packed graphic_character_t;

typedef enum {
  NO_GRAPH,
  DELETE_GRAPH,
  SINGLE_GRAPH,
  DOUBLE_GRAPH,
  FIVE_GRAPH,
  SEVEN_GRAPH,
  CHAR_GRAPH,
} content;

class Referee : public Protocol {
 public:
  game_status_t game_status{};
  game_result_t game_result{};
  game_robot_HP_t game_robot_HP{};
  event_data_t event_data{};
  supply_projectile_action_t supply_projectile_action{};
  referee_warning_t referee_warning{};
  dart_remaining_time_t dart_remaining_time{};
  game_robot_status_t game_robot_status{};
  power_heat_data_t power_heat_data{};
  game_robot_pos_t game_robot_pos{};
  buff_t buff{};
  aerial_robot_energy_t aerial_robot_energy{};
  robot_hurt_t robot_hurt{};
  shoot_data_t shoot_data{};
  bullet_remaining_t bullet_remaining{};
  rfid_status_t rfid_status{};
  dart_client_cmd_t dart_client_cmd{};
  ground_robot_position_t ground_robot_position{};
  radar_mark_data_t radar_mark_progress{};
  sentry_info_t sentry_info{};
  radar_info_t radar_info{};

  graphic_delete_t graphic_delete{};
  graphic_single_t graphic_single{};
  graphic_double_t graphic_double{};
  graphic_five_t graphic_five{};
  graphic_seven_t graphic_seven{};
  graphic_character_t graphic_character{};

  void PrepareUIContent(content graph_content);

 private:
  /**
   * @brief process the data for certain command and update corresponding status variables
   *
   * @param cmd_id    command id
   * @param data      address for command data
   * @param length    number of bytes in command data
   * @return true for success; false for failure
   */
  bool ProcessDataRx(int cmd_id, const uint8_t* data, int length) final;

  /**
   * @brief process the information for certain command and copy it into the buffer named as data
   *
   * @param cmd_id
   * @param data
   * @return length of the data that is copied into buffer
   */
  int ProcessDataTx(int cmd_id, uint8_t* data) final;

  content graph_content_ = NO_GRAPH;
};

/* Command for Host */

/*
 * 0x0401 PACK
 */

typedef enum {
  PACK = 0x0401,
  TARGET_ANGLE = 0x0402,
  NO_TARGET_FLAG = 0x0403,
  SHOOT_CMD = 0x0404,
} host_cmd;

/* ===== PACK 0x0401 ===== */
typedef struct {
  char chars[256];  // a string with maximum 256 chars
} __packed pack_t;

/* ===== TARGET_ANGLE 0x0402 ===== */
typedef struct {
  float pitch;  // TODO: decide RAD / degree with CV group
  float yaw;
} __packed target_angle_t;

/* ===== NO_TARGET_FLAG 0x0403 ===== */
typedef struct {
  char dummy;  // no actual meaning
} __packed no_target_flag_t;

/* ===== SHOOT_CMD 0x0404 ===== */
typedef struct {
  char dummy;  // no actual meaning
} __packed shoot_cmd_t;

class Host : public Protocol {
 public:
  pack_t pack{};
  target_angle_t target_angle{};
  no_target_flag_t no_target_flag{};
  shoot_cmd_t shoot_cmd{};

 private:
  /**
   * @brief process the data for certain command and update corresponding status variables
   *
   * @param cmd_id    command id
   * @param data      address for command data
   * @param length    number of bytes in command data
   * @return true for success; false for failure
   */
  bool ProcessDataRx(int cmd_id, const uint8_t* data, int length) final;

  /**
   * @brief process the information for certain command and copy it into the buffer named as data
   *
   * @param cmd_id
   * @param data
   * @return length of the data that is copied into buffer
   */
  int ProcessDataTx(int cmd_id, uint8_t* data) final;
};

} /* namespace communication */