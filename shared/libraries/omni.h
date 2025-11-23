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

#include "motor.h"

namespace control {

typedef struct {
  // Chassis
  control::MotorDM3519* motor0;
  control::MotorDM3519* motor1;
  control::MotorDM3519* motor2;
  control::MotorDM3519* motor3;
  // Gimbal
  control::Motor4310* pitch_motor;
  control::Motor6020* yaw_motor;
  float pitch_params[3];
  // Shooter

  // Physical constraints
  float yaw_offset;
  float pitch_min;
  float pitch_max;
  // Other
  control::ConstrainedPID pid;
} omni_t;

class Omni {
 public:
  Omni(omni_t* omni);

  ~Omni();

  void SetIMUAngle(float angle);

  void Init();

  void Update();

  void Transmit();

  // Chassis

  void ChassisMotorsEnable();

  void ChassisMotorsDisable();

  void SetVelocity(float vx, float vy);

  void Spin(float vw);

  // Gimbal
  void GimbalMotorsEnable();

  void GimbalMotorsDisable();

  void SetGimbalVelocity(float w_pitch, float w_yaw);

  float pitch;
  float yaw;

 private:
  float vx_;
  float vy_;
  float vw_;

  float w_pitch_;
  float w_yaw_;

  float p_kp_;
  float p_kd_;
  float p_torque_;

  float pitch_min_;
  float pitch_max_;
  float yaw_offset_;

  control::MotorDM3519* motors_[4];
  control::Motor4310* pitch_motor_;
  control::Motor6020* yaw_motor_;

  control::ConstrainedPID pid_;
  float imu_angle_;

};  // END CLASS OmniChassis

}  // END NAMESPACE control
