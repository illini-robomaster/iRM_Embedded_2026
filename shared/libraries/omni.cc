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

#include "omni.h"

#include <cmath>

#define OMNI_OFFSET -0.785398f      // -PI / 4
#define PITCH_SAFETY_FACTOR 200.0f  // Prevent slamming gimbal into chassis.
#define YAW_VW_FACTOR 400.0f        // Yaw spin modifier.
#define MAX_W 30.0f                 // Max spin rate.

namespace control {

Omni::Omni(omni_t* omni) {
  // Chassis
  motors_[0] = omni->motor0;
  motors_[1] = omni->motor1;
  motors_[2] = omni->motor2;
  motors_[3] = omni->motor3;
  // Gimbal
  pitch_motor_ = omni->pitch_motor;
  yaw_motor_ = omni->yaw_motor;
  p_kp_ = omni->pitch_params[0];
  p_kd_ = omni->pitch_params[1];
  p_torque_ = omni->pitch_params[2];
  // Shooter
  //

  yaw_offset_ = omni->yaw_offset;
  pitch_min_ = omni->pitch_min;
  pitch_max_ = omni->pitch_max;  // Pitch offset is the same as pitch max.

  vx_ = 0.0f;
  vy_ = 0.0f;
  vw_ = 0.0f;

  w_pitch_ = 0.0f;
  w_yaw_ = 0.0f;

  pid_ = omni->pid;
  imu_angle_ = 0.0f;
}

void Omni::SetIMUAngle(float angle) { imu_angle_ = angle; };

void Omni::Init() {
  motors_[0]->SetZeroPos();
  motors_[1]->SetZeroPos();
  motors_[2]->SetZeroPos();
  motors_[3]->SetZeroPos();
  // pitch_motor_->SetZeroPos();
  Update();
}

void Omni::Update() {
  yaw = yaw_motor_->GetTheta() - yaw_offset_;
  pitch = pitch_motor_->GetTheta() + pitch_max_;
}

void Omni::Transmit() {
  control::MotorDM3519::TransmitOutput(motors_, 4);
  control::Motor6020::TransmitOutput((control::MotorCANBase**)(&yaw_motor_), 1);
  control::Motor4310::TransmitOutput((control::Motor4310**)(&pitch_motor_), 1);
}

void Omni::ChassisMotorsEnable() {
  motors_[0]->MotorEnable();
  motors_[1]->MotorEnable();
  motors_[2]->MotorEnable();
  motors_[3]->MotorEnable();
}

void Omni::ChassisMotorsDisable() {
  motors_[0]->MotorDisable();
  motors_[1]->MotorDisable();
  motors_[2]->MotorDisable();
  motors_[3]->MotorDisable();
}

void Omni::SetVelocity(float vx, float vy) {
  float cyaw = cosf(OMNI_OFFSET + yaw);
  float syaw = sinf(OMNI_OFFSET + yaw);
  vx_ = vx * cyaw - vy * syaw;
  vy_ = vx * syaw + vy * cyaw;

  float vw_now = fabs(MAX_W - max(fabsf(vx_), fabsf(vy_)));
  motors_[0]->SetOutput(vw_now - vy_);
  motors_[0]->SetOutput(vw_now + vy_);
  motors_[0]->SetOutput(vw_now - vx_);
  motors_[0]->SetOutput(vw_now + vx_);
}

void Omni::Spin(float vw) {
  vw_ = clip<float>(vw, 0.0f, MAX_W);
}

void Omni::GimbalMotorsEnable() {
  pitch_motor_->MotorEnable();
}

void Omni::GimbalMotorsDisable() {
  pitch_motor_->MotorEnable();
}

void Omni::SetGimbalVelocity(float w_pitch, float w_yaw) {
  float pitch_angle = clip<float>(pitch + w_pitch / PITCH_SAFETY_FACTOR, pitch_min_, pitch_max_);
  float delta_yaw = w_yaw * 0.01f;

  int16_t yaw_val = pid_.ComputeOutput(atan2f(sinf(delta_yaw), cosf(delta_yaw)));  // Wrap arg to [-PI, PI).

  yaw_motor_->SetOutput(yaw_val + vw_ * YAW_VW_FACTOR);
  pitch_motor_->SetOutput(pitch_angle - pitch_max_, w_pitch, p_kp_, p_kd_, p_torque_);
}

}  // END NAMESPACE control
