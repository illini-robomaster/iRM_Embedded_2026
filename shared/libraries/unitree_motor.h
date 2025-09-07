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
#include "protocol.h"

namespace control {

typedef int16_t q15_t;
// general data structure for 4 bytes
typedef union {
  int32_t  L;
  uint8_t  u8[4];
  uint16_t u16[2];
  uint32_t u32;
  float    F;
} COMData32_t; 

// data head
typedef struct {
  unsigned char start[2]; // fixed to be 0xFE and OxEE
  unsigned char motorID;  // motor ID: 0, 1, or 2; 0xBB is for motor broadcasting (reset motor ID)
  unsigned char reserved; // ignored
} __packed COMHead_t;

// send data body
typedef struct {
  uint8_t     mode;       // 0 for stop running; 5 for slowly running with open-loop control; 10 for FOC closed-loop control
  uint8_t     ModifyBit;  // ignored
  uint8_t     ReadBit;    // ignored
  uint8_t     reserved;   // ignored

  COMData32_t Modify;     // ignored

  q15_t       T;          // Expect Torque X 256
  q15_t       W;          // Expect Speed X 128
  int32_t     Pos;        // Expect Position X 16384 / (2 * Pi)

  q15_t       K_P;        // Kp X 2048
  q15_t       K_W;        // Kd X 1024

  uint8_t     LowHzMotorCmdIndex; // ignored
  uint8_t     LowHzMotorCmdByte;  // ignored
	
  COMData32_t Res[1];     // ignored
} __packed MasterComd_t;

// send data package including data head and data body
typedef struct {
  COMHead_t    head;      // data head
  MasterComd_t Mdata;     // data body
  COMData32_t  CRCdata;   // CRC32 verification
} __packed MasterComdData_t;

// recv data body
typedef struct {
  uint8_t     mode;       // current mode
  uint8_t     ReadBit;    // ignored
  int8_t      Temp;       // motor temperature
  uint8_t     MError;     // error message
 
  COMData32_t Read;       // ignored
  int16_t     T;          // Current Torque X 256

  int16_t     W;          // Current Speed X 128
  float       LW;         // filtered current speed, not suggested to use
  int16_t     W2;         // ignored
  float       LW2;        // ignored

  int16_t     Acc;        // Current Acceleration X 1
  int16_t     OutAcc;     // ignored
		 
  int32_t     Pos;        // Current Position X 16384 / (2 * Pi)
  int32_t     Pos2;       // ignored

  int16_t     gyro[3];    // Current Angular Velocity from IMU, times (2000 / 2^15) X (2 * Pi / 360) to translate to rad/s
  int16_t     acc[3];     // Current Linear Acceleration from IMU, times 8 X 9.80665 / 2^15 to translate to m/s^2

  int16_t     Fgyro[3];   // ignored
  int16_t     Facc[3];    // ignored
  int16_t     Fmag[3];    // ignored
  uint8_t     Ftemp;      // ignored
    
  int16_t     Force16;    // ignored
  int8_t      Force8;     // ignored
		
  uint8_t     FError;     // ignored
		
  int8_t      Res[1];     // ignored
} __packed ServoComd_t;

// recv data package including data head and data body
typedef struct {
  COMHead_t   head;       // data head
  ServoComd_t Mdata;      // data body
  COMData32_t CRCdata;    // CRC32 verification
} __packed ServoComdData_t;

// send storage structure
typedef struct{
  MasterComdData_t data;  // send data package
  unsigned short   id;    // motor ID: 0, 1, or 2; 0xBB is for motor broadcasting (reset motor ID)
  unsigned short   mode;  // 0 for stop running; 5 for slowly running with open-loop control; 10 for FOC closed-loop control
  float            T;     // Expect Torque
  float            W;     // Expect Speed
  float            Pos;   // Expect Position
  float            K_P;   // Kp
  float            K_W;   // Kd
  COMData32_t      Res;   // ignored
} motor_send_t;

// recv storage structure
typedef struct {
  ServoComdData_t data;    // recv data package
  bool            correct; // completeness of data package
  int             id;      // motor ID: 0, 1, or 2
  int             mode;    // 0 for stop running; 5 for slowly running with open-loop control; 10 for FOC closed-loop control
  int             Temp;    // motor temperature
  int             MError;  // error message
  float           T;       // Current Torque
  float           W;       // Current Speed
  float           LW;      // filtered current speed, not suggested to use
  int             Acc;     // Current Acceleration
  float           Pos;     // Current Position
  float           gyro[3]; // Current Angular Velocity
  float           acc[3];  // Current Linear Acceleration
} motor_recv_t;

/**
 * @brief general protocol for Unitree motors
 * @note A1 needs 4.8M baud
 */
class UnitreeMotor {
public:
  UnitreeMotor();
  /**
   * @brief zip the send data package
   */
  void ModifyData(const unsigned short motor_id);

  /**
   * @brief unzip the recv data package
   *
   * @param package recv data package including data address and length
   * @return completeness of data package
   */
  bool ExtractData(const communication::package_t package);

  /**
   * @brief stop running
   *
   * @param motor_id motor ID
   */
  void Stop(const unsigned short motor_id);

  /**
   * @brief slowly running with open-loop control for testing motors
   *
   * @param motor_id motor ID
   */
  void Test(const unsigned short motor_id);

  /**
   * @brief FOC closed-loop control
   * @note T = Tff + Kp * (Pdes - P) + Kd * (Wdes - W)
   *
   * @param motor_id motor ID
   * @param torque expect torque
   * @param speed expect speed
   * @param position expect position
   * @param Kp Kp parameter for the PD controller
   * @param Kd Kd parameter for the PD controller
   */
  void Control(const unsigned short motor_id, const float torque, const float speed, const float position, const float Kp, const float Kd);

  volatile bool connection_flag_[3] = {false};

  motor_send_t send[3]; // send storage structure instance
  motor_recv_t recv[3]; // recv storage structure instance

  const int send_length = 34; // send data package length
  const int recv_length = 78; // recv data package length
  const float gear_ratio = 9.07; // A1 Gear Ratio

private:
  /**
   * @brief CRC32 verification function
   *
   * @param ptr data package address
   * @param len data package length
   * @return checksum result
   */
  uint32_t crc32_core(uint32_t* ptr, uint32_t len);

};



}
