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

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "motor.h"
#include "steering.h"
#include "utils.h"

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;

constexpr float RUN_SPEED = (10 * PI) / 32;
constexpr float ALIGN_SPEED = (PI);
constexpr float ACCELERATION = (100 * PI);

constexpr float OFFSET_MOTOR1 = 0;
constexpr float OFFSET_MOTOR2 = 0;
constexpr float OFFSET_MOTOR3 = 0;
constexpr float OFFSET_MOTOR4 = 0;

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

bsp::GPIO* key1 = nullptr;
BoolEdgeDetector key_detector(false);

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

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);

  // button on typeC. 1 if not pressed 0 otherwise
  key1 = new bsp::GPIO(GPIOA, GPIO_PIN_0);

  /* Usage:
   *   The 'key' is the button on TypeC board.
   *   The goal of this example is to measure the offset (position of photoelectric sensors to real alignment position)
   *   Press key to start alignment, you should see four wheels turning to align.
   *   When alignment finishes, press key again to turn off motors' power.
   *   Now you can turn the motor manually. Align motors by hand to measure the offset of each motor.
  **/
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
  // Need to init because steering chassis rejects nullptr
  motor5 = new control::Motor3508(can2, 0x205);
  motor6 = new control::Motor3508(can2, 0x206);
  motor7 = new control::Motor3508(can2, 0x207);
  motor8 = new control::Motor3508(can2, 0x208);

  steering_chassis->fl_wheel_motor = motor8;
  steering_chassis->fr_wheel_motor = motor7;
  steering_chassis->bl_wheel_motor = motor5;
  steering_chassis->br_wheel_motor = motor6;

  chassis = new control::SteeringChassis(steering_chassis);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* steer_motors[] = {trigger_motor, motor2, motor3, motor4};

  osDelay(500);

  // Press Key to start aligning. Else sudden current change when power is switched on might break
  // the board.
  while(key1->Read());

  // wait for release because align_detect also is key press here
  while(!key1->Read());

  print("Alignment Begin\r\n");

  chassis->SteerSetMaxSpeed(ALIGN_SPEED);

  bool alignment_complete = false;
  while (!alignment_complete) {
    chassis->SteerCalcOutput();
    control::MotorCANBase::TransmitOutput(steer_motors, 4);
    alignment_complete = chassis->Calibrate();
    osDelay(1);
  }

  chassis->ReAlign();
  chassis->PrintData();

  print("\r\nAlignment End\r\n");

  while(1) {
    chassis->SteerCalcOutput();
    control::MotorCANBase::TransmitOutput(steer_motors, 4);
    osDelay(2);
    if (!key1->Read()) {
      break;
    }
  }

  while(1) {
    chassis->PrintData();
    osDelay(1000);
  }

}
