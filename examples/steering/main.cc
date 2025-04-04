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

#include "main.h"

#include <cmath>

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "steering.h"
#include "motor.h"

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;

constexpr float RUN_SPEED = (4 * PI);
constexpr float ALIGN_SPEED = (PI);
constexpr float ACCELERATION = (100 * PI);
constexpr float WHEEL_SPEED = 200;

constexpr float OFFSET_MOTOR1 = 0;
constexpr float OFFSET_MOTOR2 = 0;
constexpr float OFFSET_MOTOR3 = 0;
constexpr float OFFSET_MOTOR4 = 0;

/* Usage:
 * Working Steering Chassis Example
 * Left stick for translation, right stick for rotation
 * Right switch UP is to start an alignment (calibration)
**/

control::MotorCANBase* trigger_motor = nullptr;
control::MotorCANBase* motor2 = nullptr;
control::MotorCANBase* motor3 = nullptr;
control::MotorCANBase* motor4 = nullptr;

control::SteeringMotor* steering1 = nullptr;
control::SteeringMotor* steering2 = nullptr;
control::SteeringMotor* steering3 = nullptr;
control::SteeringMotor* steering4 = nullptr;

control::MotorCANBase* motor5 = nullptr;
control::MotorCANBase* motor6 = nullptr;
control::MotorCANBase* motor7 = nullptr;
control::MotorCANBase* motor8 = nullptr;

bsp::GPIO* pe1 = nullptr;
bsp::GPIO* pe2 = nullptr;
bsp::GPIO* pe3 = nullptr;
bsp::GPIO* pe4 = nullptr;

bool steering_align_detect1() {
  return pe1->Read() == 0;
}

bool steering_align_detect2() {
  return pe2->Read() == 0;
}

bool steering_align_detect3() {
  return pe3->Read() == 0;
}

bool steering_align_detect4() {
  return pe4->Read() == 0;
}

control::steering_chassis_t* steering_chassis;

control::SteeringChassis* chassis;
remote::DBUS* dbus = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);

  pe1 = new bsp::GPIO(IN1_GPIO_Port, IN1_Pin);
  pe2 = new bsp::GPIO(IN2_GPIO_Port, IN2_Pin);
  pe3 = new bsp::GPIO(IN3_GPIO_Port, IN3_Pin);
  pe4 = new bsp::GPIO(IN4_GPIO_Port, IN4_Pin);

  can1 = new bsp::CAN(&hcan1, true);
  can2 = new bsp::CAN(&hcan2, false);

  // init steerings
  trigger_motor = new control::Motor3508(can1, 0x201);
  motor2 = new control::Motor3508(can1, 0x202);
  motor3 = new control::Motor3508(can1, 0x203);
  motor4 = new control::Motor3508(can1, 0x204);

  control::steering_t steering_data;
  steering_data.motor = trigger_motor;
  steering_data.max_speed = RUN_SPEED;
  steering_data.max_acceleration = ACCELERATION;
  steering_data.transmission_ratio = 8;
  steering_data.omega_pid_param = new float[3]{140, 1.2, 25};
  steering_data.max_iout = 1000;
  steering_data.max_out = 13000;
  steering_data.calibrate_offset = OFFSET_MOTOR1;
  steering_data.align_detect_func = steering_align_detect1;
  steering1 = new control::SteeringMotor(steering_data);

  steering_data.calibrate_offset = OFFSET_MOTOR2;
  steering_data.motor = motor2;
  steering_data.align_detect_func = steering_align_detect2;
  steering2 = new control::SteeringMotor(steering_data);

  steering_data.calibrate_offset = OFFSET_MOTOR3;
  steering_data.motor = motor3;
  steering_data.align_detect_func = steering_align_detect3;
  steering3 = new control::SteeringMotor(steering_data);

  steering_data.motor = motor4;
  steering_data.calibrate_offset = OFFSET_MOTOR4;
  steering_data.align_detect_func = steering_align_detect4;
  steering4 = new control::SteeringMotor(steering_data);

  steering_chassis = new control::steering_chassis_t();

  steering_chassis->fl_steer_motor = steering4;
  steering_chassis->fr_steer_motor = steering3;
  steering_chassis->bl_steer_motor = steering1;
  steering_chassis->br_steer_motor = steering2;

  // init wheels
  motor5 = new control::Motor3508(can2, 0x205);
  motor6 = new control::Motor3508(can2, 0x206);
  motor7 = new control::Motor3508(can2, 0x207);
  motor8 = new control::Motor3508(can2, 0x208);

  steering_chassis->fl_wheel_motor = motor8;
  steering_chassis->fr_wheel_motor = motor7;
  steering_chassis->bl_wheel_motor = motor5;
  steering_chassis->br_wheel_motor = motor6;

  chassis = new control::SteeringChassis(steering_chassis);

  dbus = new remote::DBUS(&huart3);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  control::MotorCANBase* steer_motors[] = {trigger_motor, motor2, motor3, motor4};

  control::MotorCANBase* wheel_motors[] = {motor5, motor6, motor7, motor8};
  control::PIDController pid5(120, 15, 30);
  control::PIDController pid6(120, 15, 30);
  control::PIDController pid7(120, 15, 30);
  control::PIDController pid8(120, 15, 30);

  osDelay(500);  // DBUS initialization needs time

  double vx = 0.0;
  double vy = 0.0;
  double vw = 0.0;

  float v5 = 0;
  float v6 = 0;
  float v7 = 0;
  float v8 = 0;

  bool alignment = false;

  while (true) {
    vx = -static_cast<double>(dbus->ch3) / 660;
    vy = static_cast<double>(dbus->ch2) / 660;
    vw = static_cast<double>(dbus->ch0) / 660;

    // Kill switch
    if (dbus->swl == remote::UP || dbus->swl == remote::DOWN) {
      RM_ASSERT_TRUE(false, "Operation killed");
    }

    // Only Calibrate once per Switch change
    if (dbus->swr != remote::UP) {
      alignment = false;
    }
    // Right Switch Up to start a calibration
    if (dbus->swr == remote::UP && alignment == false) {
      chassis->SteerSetMaxSpeed(ALIGN_SPEED);
      bool alignment_complete = false;
      while (!alignment_complete) {
        chassis->SteerCalcOutput();
        control::MotorCANBase::TransmitOutput(steer_motors, 4);
        alignment_complete = chassis->Calibrate();
        osDelay(1);
      }
      chassis->ReAlign();
      alignment = true;
      chassis->SteerSetMaxSpeed(RUN_SPEED);
      chassis->SteerThetaReset();

      v5 = 0;
      v6 = 0;
      v7 = 0;
      v8 = 0;
      chassis->SetWheelSpeed(0,0,0,0);
    } else {
      chassis->SetSpeed(vx, vy, vw);
      chassis->SteerUpdateTarget();
      chassis->WheelUpdateSpeed(WHEEL_SPEED);
      v5 = chassis->v_bl_;
      v6 = chassis->v_br_;
      v7 = chassis->v_fr_;
      v8 = chassis->v_fl_;
    }

    chassis->SteerCalcOutput();

    motor5->SetOutput(pid5.ComputeConstrainedOutput(motor5->GetOmegaDelta(v5)));
    motor6->SetOutput(pid6.ComputeConstrainedOutput(motor6->GetOmegaDelta(v6)));
    motor7->SetOutput(pid7.ComputeConstrainedOutput(motor7->GetOmegaDelta(v7)));
    motor8->SetOutput(pid8.ComputeConstrainedOutput(motor8->GetOmegaDelta(v8)));

    control::MotorCANBase::TransmitOutput(steer_motors, 4);
    control::MotorCANBase::TransmitOutput(wheel_motors, 4);
    osDelay(2);
  }
}
