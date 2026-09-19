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

/**
 * CV target spinner.
 *
 * Setup: dbus receiver -> chassis TypeC board -> steering chassis
 * (4 steering motors on CAN1 0x201-0x204, 4 wheel motors on CAN2 0x205-0x208).
 *
 * On boot the steering motors align against the photoelectric sensors
 * (IN1-IN4). Flip the left switch (swl) UP to spin in place; any other
 * position stops the wheels (steering motors keep holding their tangent).
 */

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "steering.h"

// speed for steering motors (rad/s)
constexpr float RUN_SPEED = (4 * PI);
constexpr float ALIGN_SPEED = (PI);
constexpr float ACCELERATION = (100 * PI);

// chassis rotation speed (no unit, see chassis.cc)
constexpr float SPIN_SPEED = 8;
constexpr float WHEEL_SPEED_FACTOR = 4;

// fake referee values: no power limiting, PID output passes through
constexpr float POWER_LIMIT = 80;
constexpr float CHASSIS_POWER = 0;
constexpr float CHASSIS_POWER_BUFFER = 0;

static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;
static remote::DBUS* dbus = nullptr;

static control::MotorCANBase* motor1 = nullptr;
static control::MotorCANBase* motor2 = nullptr;
static control::MotorCANBase* motor3 = nullptr;
static control::MotorCANBase* motor4 = nullptr;
static control::MotorCANBase* motor5 = nullptr;
static control::MotorCANBase* motor6 = nullptr;
static control::MotorCANBase* motor7 = nullptr;
static control::MotorCANBase* motor8 = nullptr;

static control::SteeringMotor* steering_motor1 = nullptr;
static control::SteeringMotor* steering_motor2 = nullptr;
static control::SteeringMotor* steering_motor3 = nullptr;
static control::SteeringMotor* steering_motor4 = nullptr;

static bsp::GPIO* pe1 = nullptr;
static bsp::GPIO* pe2 = nullptr;
static bsp::GPIO* pe3 = nullptr;
static bsp::GPIO* pe4 = nullptr;

static control::steering_chassis_t* chassis_data = nullptr;
static control::SteeringChassis* chassis = nullptr;

bool steering_align_detect1() { return pe1->Read() == 0; }

bool steering_align_detect2() { return pe2->Read() == 0; }

bool steering_align_detect3() { return pe3->Read() == 0; }

bool steering_align_detect4() { return pe4->Read() == 0; }

void RM_RTOS_Init() {
  print_use_uart(&huart1);

  can1 = new bsp::CAN(&hcan1, true);
  can2 = new bsp::CAN(&hcan2, false);
  dbus = new remote::DBUS(&huart3);

  motor1 = new control::Motor3508(can1, 0x201);
  motor2 = new control::Motor3508(can1, 0x202);
  motor3 = new control::Motor3508(can1, 0x203);
  motor4 = new control::Motor3508(can1, 0x204);

  motor5 = new control::Motor3508(can2, 0x205);
  motor6 = new control::Motor3508(can2, 0x206);
  motor7 = new control::Motor3508(can2, 0x207);
  motor8 = new control::Motor3508(can2, 0x208);

  pe1 = new bsp::GPIO(IN1_GPIO_Port, IN1_Pin);
  pe2 = new bsp::GPIO(IN2_GPIO_Port, IN2_Pin);
  pe3 = new bsp::GPIO(IN3_GPIO_Port, IN3_Pin);
  pe4 = new bsp::GPIO(IN4_GPIO_Port, IN4_Pin);

  control::steering_t steering_motor_data;
  steering_motor_data.motor = motor1;
  steering_motor_data.max_speed = RUN_SPEED;
  steering_motor_data.max_acceleration = ACCELERATION;
  steering_motor_data.transmission_ratio = 8;
  steering_motor_data.omega_pid_param = new float[3]{200, 7, 1};
  steering_motor_data.max_iout = 1000;
  steering_motor_data.max_out = 13000;
  steering_motor_data.calibrate_offset = 0;

  steering_motor_data.align_detect_func = steering_align_detect1;
  steering_motor1 = new control::SteeringMotor(steering_motor_data);

  steering_motor_data.motor = motor2;
  steering_motor_data.align_detect_func = steering_align_detect2;
  steering_motor2 = new control::SteeringMotor(steering_motor_data);

  steering_motor_data.motor = motor3;
  steering_motor_data.align_detect_func = steering_align_detect3;
  steering_motor3 = new control::SteeringMotor(steering_motor_data);

  steering_motor_data.motor = motor4;
  steering_motor_data.align_detect_func = steering_align_detect4;
  steering_motor4 = new control::SteeringMotor(steering_motor_data);

  chassis_data = new control::steering_chassis_t();

  chassis_data->fl_steer_motor = steering_motor4;
  chassis_data->fr_steer_motor = steering_motor3;
  chassis_data->bl_steer_motor = steering_motor1;
  chassis_data->br_steer_motor = steering_motor2;

  chassis_data->fl_wheel_motor = motor8;
  chassis_data->fr_wheel_motor = motor7;
  chassis_data->bl_wheel_motor = motor5;
  chassis_data->br_wheel_motor = motor6;

  chassis = new control::SteeringChassis(chassis_data);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  control::MotorCANBase* steer_motors[] = {motor1, motor2, motor3, motor4};
  control::MotorCANBase* wheel_motors[] = {motor5, motor6, motor7, motor8};

  osDelay(1000);

  // align steering motors against the photoelectric sensors
  chassis->SteerSetMaxSpeed(ALIGN_SPEED);
  bool alignment_complete = false;
  while (!alignment_complete) {
    chassis->SteerCalcOutput();
    control::MotorCANBase::TransmitOutput(steer_motors, 4);
    alignment_complete = chassis->Calibrate();
    osDelay(1);
  }
  int align_failed = chassis->ReAlign();
  print("Alignment complete, align_failed: %d\r\n", align_failed);
  chassis->SteerCalcOutput();
  control::MotorCANBase::TransmitOutput(steer_motors, 4);

  chassis->SteerSetMaxSpeed(RUN_SPEED);
  chassis->SteerThetaReset();
  chassis->SetWheelSpeed(0, 0, 0, 0);

  while (true) {
    // TODO: spin disabled until alignment is verified on hardware
    // bool spin = dbus->connection_flag_ && dbus->swl == remote::UP;

    // vx = vy = 0, wz only -> all wheels tangent, pure rotation in place
    // chassis->SetSpeed(0, 0, spin ? SPIN_SPEED : 0);
    chassis->SetSpeed(0, 0, 0);
    chassis->SteerUpdateTarget();
    chassis->WheelUpdateSpeed(WHEEL_SPEED_FACTOR);
    chassis->SteerCalcOutput();
    chassis->Update(POWER_LIMIT, CHASSIS_POWER, CHASSIS_POWER_BUFFER);

    control::MotorCANBase::TransmitOutput(wheel_motors, 4);
    control::MotorCANBase::TransmitOutput(steer_motors, 4);

    osDelay(2);
  }
}
