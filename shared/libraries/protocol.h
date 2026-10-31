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

#include <cstdint>
#include "bsp_error_handler.h"

namespace communication {

// RM2027 maximum frame length: VTM frame 0x0310 payload is 300 bytes + 9 bytes overhead = 309.
// 512 bytes buffer provides safe headroom for all links.
constexpr int MAX_FRAME_LEN = 512;

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
   * @param data      address for frame data
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

/* ========================================================================= */
/*                   RoboMaster 2027 Referee System Commands                 */
/* ========================================================================= */

typedef enum {
  // Regular link (0x00xx - 0x03xx)
  GAME_STATUS = 0x0001,
  GAME_RESULT = 0x0002,
  GAME_ROBOT_HP = 0x0003,
  EVENT_DATA = 0x0101,
  SUPPLY_PROJECTILE_ACTION = 0x0102,  // Legacy compatibility
  REFEREE_WARNING = 0x0104,
  DART_REMAINING_TIME = 0x0105,
  GAME_ROBOT_STATUS = 0x0201,
  POWER_HEAT_DATA = 0x0202,
  GAME_ROBOT_POS = 0x0203,
  BUFF = 0x0204,
  AERIAL_ROBOT_ENERGY = 0x0205,       // Legacy compatibility
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
  MAP_COMMAND = 0x0303,
  MAP_ROBOT_DATA = 0x0305,
  MAP_DATA = 0x0307,
  CUSTOM_INFO = 0x0308,

  // VTM link (921600 baud)
  VTM_CUSTOM_TERMINAL_TX = 0x0310,
  VTM_CUSTOM_TERMINAL_RX = 0x0311,

  // Radar wireless link (intel from referee signal source)
  RADAR_ENEMY_POS = 0x0A01,
  RADAR_ENEMY_HP = 0x0A02,
  RADAR_ENEMY_ALLOWANCE = 0x0A03,
  RADAR_ENEMY_MACRO = 0x0A04,
  RADAR_ENEMY_BUFFS = 0x0A05,
  RADAR_ENEMY_KEY = 0x0A06,
} referee_cmd;

/* ===== GAME_STATUS 0x0001 (11 B, 1 Hz) ===== */
typedef struct {
  uint8_t game_type : 4;       // 1 RMUC, 2 Single, 3 ICRA RMUA, 4 RMUL 3v3, 5 RMUL infantry
  uint8_t game_progress : 4;   // 0 not started, 1 prep, 2 self-check, 3 countdown, 4 in game, 5 settling
  uint16_t stage_remain_time;  // s
  uint64_t SyncTimeStamp;      // UNIX timestamp (valid after NTP sync)
} __packed game_status_t;

/* ===== GAME_RESULT 0x0002 (1 B, On match end) ===== */
typedef struct {
  uint8_t winner;  // 0 draw, 1 red, 2 blue
} __packed game_result_t;

/* ===== GAME_ROBOT_HP 0x0003 (24 B, 3 Hz) ===== */
typedef struct {
  uint16_t ally_1_robot_HP;       // Heavy robot (slot 1)
  uint16_t reserved;              // Slot 2 reserved (former Engineer)
  uint16_t ally_3_robot_HP;       // Infantry 3
  uint16_t ally_4_robot_HP;       // Infantry 4
  int16_t damage_difference;      // Own total damage minus enemy total damage
  uint16_t ally_7_robot_HP;       // Sentry robot
  uint16_t outpost_R_HP;          // Red-side outpost HP at match start
  uint16_t ally_base_HP;          // Ally base HP
  uint16_t outpost_B_HP;          // Blue-side outpost HP at match start
  uint16_t enemy_base_HP;         // Enemy base HP
  uint16_t ally_control_degree;   // Cumulative ally control degree
  uint16_t enemy_control_degree;  // Cumulative enemy control degree
} __packed game_robot_HP_t;

/* ===== EVENT_DATA 0x0101 (8 B, 1 Hz) ===== */
typedef struct {
  union {
    uint32_t event_data;
    uint32_t event_type;  // Legacy alias
  };
  uint32_t event_data_2;
} __packed event_data_t;

/* ===== SUPPLY_PROJECTILE_ACTION 0x0102 (Legacy compatibility) ===== */
typedef struct {
  uint8_t supply_projectile_id;
  uint8_t supply_robot_id;
  uint8_t supply_projectile_step;
  uint8_t supply_projectile_num;
} __packed supply_projectile_action_t;

/* ===== REFEREE_WARNING 0x0104 (3 B, On penalty / 1 Hz) ===== */
typedef struct {
  uint8_t level;  // 1 both yellow, 2 yellow, 3 red, 4 forfeit
  union {
    uint8_t offending_robot_id;
    uint8_t foul_robot_id;  // Legacy alias
  };
  uint8_t count;
} __packed referee_warning_t;

/* ===== DART_REMAINING_TIME / DART_INFO 0x0105 (3 B, 1 Hz) ===== */
typedef struct {
  uint8_t dart_remaining_time;
  uint16_t dart_info;
} __packed dart_remaining_time_t;
using dart_info_t = dart_remaining_time_t;

/* ===== GAME_ROBOT_STATUS 0x0201 (17 B, 10 Hz) ===== */
typedef struct {
  uint8_t robot_id;
  uint8_t robot_level;
  union {
    uint16_t current_HP;
    uint16_t remain_HP;  // Legacy alias
  };
  union {
    uint16_t maximum_HP;
    uint16_t max_HP;     // Legacy alias
  };
  uint16_t shooter_barrel_cooling_value;
  uint16_t shooter_barrel_heat_limit;
  uint16_t chassis_power_limit;
  float bullet_speed_limit;
  union {
    struct {
      uint8_t power_management_gimbal_output : 1;
      uint8_t power_management_chassis_output : 1;
      uint8_t power_management_shooter_output : 1;
    };
    struct {
      uint8_t mains_power_gimbal_output : 1;
      uint8_t mains_power_chassis_output : 1;
      uint8_t mains_power_shooter_output : 1;
    };
  };
} __packed game_robot_status_t;
using robot_status_t = game_robot_status_t;

/* ===== POWER_HEAT_DATA 0x0202 (14 B, 10 Hz) ===== */
// Note: In RM2027 bytes 0-7 are reserved; chassis voltage/current/power are no longer provided.
typedef struct {
  union {
    struct {
      uint16_t reserved0;
      uint16_t reserved1;
      uint16_t reserved2;
      uint16_t reserved3;
    };
    struct {
      uint16_t chassis_volt;
      uint16_t chassis_current;
      float chassis_power;
    };
  };
  union {
    uint16_t buffer_energy;
    uint16_t chassis_power_buffer;  // Legacy alias
  };
  union {
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_id1_17mm_cooling_heat;  // Legacy alias
  };
  union {
    uint16_t shooter_42mm_barrel_heat;
    uint16_t shooter_id1_42mm_cooling_heat;  // Legacy alias
  };
} __packed power_heat_data_t;

/* ===== GAME_ROBOT_POS 0x0203 (12 B, 1 Hz) ===== */
typedef struct {
  float x;  // m
  float y;  // m
  union {
    float angle;  // deg (speed-module heading, 0 = north)
    float yaw;    // Legacy alias
  };
} __packed game_robot_pos_t;
using robot_pos_t = game_robot_pos_t;

/* ===== BUFF 0x0204 (7 B, 3 Hz) ===== */
typedef struct {
  uint8_t recovery_buff;        // % max HP/s
  uint16_t cooling_buff;        // cooling heat/s
  uint8_t defence_buff;         // %
  uint8_t vulnerability_buff;   // %
  uint16_t attack_buff;         // %
} __packed buff_t;

/* ===== AERIAL_ROBOT_ENERGY 0x0205 (Legacy compatibility) ===== */
typedef struct {
  uint8_t airforce_status;
  uint8_t time_remain;
} __packed aerial_robot_energy_t;

/* ===== ROBOT_HURT / HURT_DATA 0x0206 (7 B, On damage event) ===== */
typedef struct {
  union {
    struct {
      uint8_t armor_id : 4;
      uint8_t HP_deduction_reason : 4;  // 0 projectile, 1 armor offline, 2 cap offline, 5 collision
    };
    struct {
      uint8_t armor_type : 4;
      uint8_t hurt_type : 4;  // Legacy alias
    };
  };
  uint16_t hit_sequence;
  int16_t hit_position_x;  // mm (origin = armor face bottom-left)
  int16_t hit_position_y;  // mm
} __packed robot_hurt_t;
using hurt_data_t = robot_hurt_t;

/* ===== SHOOT_DATA 0x0207 (7 B, On each shot) ===== */
typedef struct {
  uint8_t bullet_type;  // 1 = 17mm, 2 = 42mm
  union {
    uint8_t shooter_number;  // 1 = 17mm, 2 reserved, 3 = 42mm
    uint8_t shooter_id;      // Legacy alias
  };
  union {
    uint8_t launching_frequency;  // Hz
    uint8_t bullet_freq;          // Legacy alias
  };
  union {
    float initial_speed;  // m/s
    float bullet_speed;   // Legacy alias
  };
} __packed shoot_data_t;

/* ===== BULLET_REMAINING / PROJECTILE_ALLOWANCE 0x0208 (8 B, 10 Hz) ===== */
typedef struct {
  union {
    uint16_t allowance_17mm;
    uint16_t bullet_remaining_num_17mm;  // Legacy alias
  };
  union {
    uint16_t allowance_42mm;
    uint16_t bullet_remaining_num_42mm;  // Legacy alias
  };
  union {
    uint16_t remaining_gold_coin;
    uint16_t coin_remaining_num;         // Legacy alias
  };
  uint16_t allowance_fortress;
} __packed projectile_allowance_t;
using bullet_remaining_t = projectile_allowance_t;

/* ===== RFID_STATUS 0x0209 (6 B, 3 Hz) ===== */
typedef struct {
  uint32_t rfid_status;
  uint16_t rfid_status_2;
} __packed rfid_status_t;

/* ===== DART_CLIENT_CMD 0x020A (6 B, 3 Hz) ===== */
typedef struct {
  uint8_t dart_launch_opening_status;  // 0 open, 1 closed, 2 opening/closing
  union {
    uint8_t reserved;
    uint8_t dart_attack_target;        // Legacy alias
  };
  uint16_t target_change_time;
  union {
    uint16_t latest_launch_cmd_time;
    uint16_t operate_launch_cmd_time;   // Legacy alias
  };
} __packed dart_client_cmd_t;

/* ===== GROUND_ROBOT_POSITION 0x020B (40 B, 1 Hz) ===== */
typedef struct {
  union {
    float heavy_x;
    float hero_x;  // Legacy alias
  };
  union {
    float heavy_y;
    float hero_y;  // Legacy alias
  };
  union {
    float reserved1;
    float engiener_x;  // Legacy alias
  };
  union {
    float reserved2;
    float engineer_y;  // Legacy alias
  };
  union {
    float infantry_3_x;
    float standard_3_x;  // Legacy alias
  };
  union {
    float infantry_3_y;
    float standard_3_y;  // Legacy alias
  };
  union {
    float infantry_4_x;
    float standard_4_x;  // Legacy alias
  };
  union {
    float infantry_4_y;
    float standard_4_y;  // Legacy alias
  };
  union {
    float reserved3;
    float standard_5_x;  // Legacy alias
  };
  union {
    float reserved4;
    float standard_5_y;  // Legacy alias
  };
} __packed ground_robot_position_t;

/* ===== RADAR_MARK_PROGRESS 0x020C (2 B, 1 Hz) ===== */
typedef struct {
  uint16_t mark_progress;
} __packed radar_mark_data_t;

/* ===== SENTRY_INFO 0x020D (14 B, 1 Hz) ===== */
typedef struct {
  uint32_t sentry_info;
  uint16_t sentry_info_2;
  uint64_t sentry_info_3;
} __packed sentry_info_t;

/* ===== RADAR_INFO 0x020E (8 B, 1 Hz) ===== */
typedef struct {
  uint8_t radar_info;
  uint8_t radar_info_2;
  uint16_t fortify_center_x;
  uint16_t fortify_center_y;
  uint16_t fortify_radius;
} __packed radar_info_t;

/* ===== 0x0301 STUDENT_INTERACTIVE SUB-COMMANDS ===== */

typedef struct {
  uint16_t data_cmd_id;
  uint16_t sender_ID;
  uint16_t receiver_ID;
} __packed UI_header_data_t;

typedef struct {
  UI_header_data_t header;
  uint8_t data[112];
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
  uint8_t figure_name[3];
  uint32_t operate_type : 3;
  uint32_t figure_type : 3;
  uint32_t layer : 4;
  uint32_t color : 4;
  uint32_t details_a : 9;
  uint32_t details_b : 9;
  uint32_t width : 10;
  uint32_t start_x : 11;
  uint32_t start_y : 11;
  uint32_t details_c : 10;
  uint32_t details_d : 11;
  uint32_t details_e : 11;
} __packed interaction_figure_t;

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

/* Sub-ID 0x0120 Sentry Decision Command (4 B payload, receiver 0x8080) */
typedef struct {
  UI_header_data_t header;
  uint32_t sentry_cmd;
} __packed sentry_decision_cmd_t;

/* Sub-ID 0x0121 Radar Decision Command (13 B payload, receiver 0x8080) */
typedef struct {
  UI_header_data_t header;
  uint8_t radar_cmd;
  uint8_t password_cmd;
  uint8_t password[6];
  uint8_t armor_fortify_cmd;
  uint16_t fortify_center_x;
  uint16_t fortify_center_y;
} __packed radar_decision_cmd_t;

/* ===== 0x0303 MAP_COMMAND (12 B, Client -> Robot) ===== */
typedef struct {
  float target_position_x;  // m
  float target_position_y;  // m
  uint8_t cmd_keyboard;
  uint8_t target_robot_id;
  uint16_t cmd_source;
} __packed map_command_t;

/* ===== 0x0305 MAP_ROBOT_DATA (48 B, Radar -> Minimap) ===== */
typedef struct {
  uint16_t x;  // cm
  uint16_t y;  // cm
} __packed map_robot_pos_t;

typedef struct {
  map_robot_pos_t enemy_heavy;
  map_robot_pos_t enemy_reserved1;
  map_robot_pos_t enemy_infantry_3;
  map_robot_pos_t enemy_infantry_4;
  map_robot_pos_t enemy_reserved2;
  map_robot_pos_t enemy_sentry;
  map_robot_pos_t ally_heavy;
  map_robot_pos_t ally_reserved1;
  map_robot_pos_t ally_infantry_3;
  map_robot_pos_t ally_infantry_4;
  map_robot_pos_t ally_reserved2;
  map_robot_pos_t ally_sentry;
} __packed map_robot_data_t;

/* ===== 0x0307 MAP_DATA (105 B, Sentry/Semi-auto -> Minimap) ===== */
typedef struct {
  uint8_t intention;          // 1 attack, 2 defend, 3 move
  uint16_t start_position_x;  // dm
  uint16_t start_position_y;  // dm
  int8_t delta_x[49];         // dm
  int8_t delta_y[49];         // dm
  uint16_t sender_id;
} __packed map_data_t;

/* ===== 0x0308 CUSTOM_INFO (34 B, Robot -> Client UTF-16) ===== */
typedef struct {
  uint16_t sender_id;
  uint16_t receiver_id;
  uint8_t user_data[30];      // UTF-16LE text
} __packed custom_info_t;

/* ===== VTM LINK COMMANDS (921600 baud) ===== */

/* 0x0310 Robot -> Custom Terminal (300 B) */
typedef struct {
  uint8_t data[300];
} __packed vtm_custom_terminal_tx_t;

/* 0x0311 Custom Terminal -> Robot (60 B) */
typedef struct {
  uint8_t data[60];
} __packed vtm_custom_terminal_rx_t;

/* ===== RADAR WIRELESS LINK COMMANDS (0x0A01 - 0x0A06) ===== */

/* 0x0A01 Enemy Positions (24 B, cm) */
typedef struct {
  map_robot_pos_t heavy;
  map_robot_pos_t reserved1;
  map_robot_pos_t infantry_3;
  map_robot_pos_t infantry_4;
  map_robot_pos_t reserved2;
  map_robot_pos_t sentry;
} __packed radar_enemy_pos_t;

/* 0x0A02 Enemy HP (12 B) */
typedef struct {
  uint16_t heavy_hp;
  uint16_t reserved1;
  uint16_t infantry_3_hp;
  uint16_t infantry_4_hp;
  uint16_t reserved2;
  uint16_t sentry_hp;
} __packed radar_enemy_hp_t;

/* 0x0A03 Enemy Projectile Allowance (10 B) */
typedef struct {
  uint16_t heavy_allowance;
  uint16_t infantry_3_allowance;
  uint16_t infantry_4_allowance;
  uint16_t reserved;
  uint16_t sentry_allowance;
} __packed radar_enemy_allowance_t;

/* 0x0A04 Enemy Macro Status (12 B) */
typedef struct {
  uint16_t enemy_remaining_gold;
  uint16_t enemy_total_gold;
  uint32_t enemy_buff_status;
  uint32_t enemy_mechanism_status;
} __packed radar_enemy_macro_t;

/* 0x0A05 Enemy Buffs / States (47 B) */
typedef struct {
  uint8_t recovery_buff;
  uint16_t cooling_buff;
  uint8_t defence_buff;
  uint8_t vulnerability_buff;
  uint16_t attack_buff;
} __packed radar_robot_buff_t;

typedef struct {
  radar_robot_buff_t heavy_buff;
  radar_robot_buff_t reserved1_buff;
  radar_robot_buff_t infantry_3_buff;
  radar_robot_buff_t infantry_4_buff;
  radar_robot_buff_t sentry_buff;
  uint8_t enemy_sentry_posture;
  uint8_t heavy_main_state;
  uint8_t reserved1_main_state;
  uint8_t infantry_3_main_state;
  uint8_t infantry_4_main_state;
  uint8_t sentry_main_state;
  uint16_t fortify_center_x;  // mm
  uint16_t fortify_center_y;  // mm
  uint16_t fortify_radius;    // mm
} __packed radar_enemy_buffs_t;

/* 0x0A06 Enemy Jamming-wave Key (6 B, ASCII) */
typedef struct {
  uint8_t password[6];
} __packed radar_enemy_key_t;

/* ===== UI Content Selection Enum ===== */
typedef enum {
  NO_GRAPH,
  DELETE_GRAPH,
  SINGLE_GRAPH,
  DOUBLE_GRAPH,
  FIVE_GRAPH,
  SEVEN_GRAPH,
  CHAR_GRAPH,
  SENTRY_CMD_GRAPH,
  RADAR_CMD_GRAPH,
  CUSTOM_DATA_GRAPH,
} content;

class Referee : public Protocol {
 public:
  // Regular link data
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
  projectile_allowance_t bullet_remaining{};
  rfid_status_t rfid_status{};
  dart_client_cmd_t dart_client_cmd{};
  ground_robot_position_t ground_robot_position{};
  radar_mark_data_t radar_mark_progress{};
  sentry_info_t sentry_info{};
  radar_info_t radar_info{};

  // UI & Student interactive structures
  graphic_delete_t graphic_delete{};
  graphic_single_t graphic_single{};
  graphic_double_t graphic_double{};
  graphic_five_t graphic_five{};
  graphic_seven_t graphic_seven{};
  graphic_character_t graphic_character{};
  sentry_decision_cmd_t sentry_decision_cmd{};
  radar_decision_cmd_t radar_decision_cmd{};
  robot_interactive_data_t robot_interactive_data{};

  // Minimap commands
  map_command_t map_command{};
  map_robot_data_t map_robot_data{};
  map_data_t map_data{};
  custom_info_t custom_info{};

  // VTM Link
  vtm_custom_terminal_tx_t vtm_custom_terminal_tx{};
  vtm_custom_terminal_rx_t vtm_custom_terminal_rx{};

  // Radar wireless link
  radar_enemy_pos_t radar_enemy_pos{};
  radar_enemy_hp_t radar_enemy_hp{};
  radar_enemy_allowance_t radar_enemy_allowance{};
  radar_enemy_macro_t radar_enemy_macro{};
  radar_enemy_buffs_t radar_enemy_buffs{};
  radar_enemy_key_t radar_enemy_key{};

  void PrepareUIContent(content graph_content);

 private:
  bool ProcessDataRx(int cmd_id, const uint8_t* data, int length) final;
  int ProcessDataTx(int cmd_id, uint8_t* data) final;

  content graph_content_ = NO_GRAPH;
};

/* ========================================================================= */
/*                          Command for Host (MiniPC)                        */
/* ========================================================================= */

typedef enum {
  PACK = 0x0401,
  TARGET_ANGLE = 0x0402,
  NO_TARGET_FLAG = 0x0403,
  SHOOT_CMD = 0x0404,
} host_cmd;

typedef struct {
  char chars[256];
} __packed pack_t;

typedef struct {
  float pitch;
  float yaw;
} __packed target_angle_t;

typedef struct {
  char dummy;
} __packed no_target_flag_t;

typedef struct {
  char dummy;
} __packed shoot_cmd_t;

class Host : public Protocol {
 public:
  pack_t pack{};
  target_angle_t target_angle{};
  no_target_flag_t no_target_flag{};
  shoot_cmd_t shoot_cmd{};

 private:
  bool ProcessDataRx(int cmd_id, const uint8_t* data, int length) final;
  int ProcessDataTx(int cmd_id, uint8_t* data) final;
};

} /* namespace communication */
