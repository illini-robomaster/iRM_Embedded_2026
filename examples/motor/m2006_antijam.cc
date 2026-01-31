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

// #define WITH_CONTROLLER

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "motor.h"
#include "utils.h"
#include "dbus.h"


#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

static const float NOTCH = (2 * PI / 8);
static const float LOAD_ANGLE = (2 * PI / 8);
static const float SPEED = (6 * PI);
static const float ACCELERATION = (200 * PI);

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motor = nullptr;
control::ServoMotor* servo = nullptr;
BoolEdgeDetector key_detector(false);

remote::DBUS* dbus = nullptr;

void jam_callback(control::ServoMotor* servo, const control::servo_jam_t data) {
  UNUSED(data);
  float servo_target = servo->GetTarget();
  if (servo_target < servo->GetTheta()) {
    print("Antijam in operation\r\n");
  } else {
    servo->SetTarget(servo->GetTheta(), true);
    float prev_target = servo->GetTarget() - NOTCH;
    servo->SetTarget(prev_target, true);
    print("Antijam engage\r\n");
  }
}

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);

  can1 = new bsp::CAN(&hcan1, true);
  motor = new control::Motor2006(can1, 0x205);

  control::servo_t servo_data;
  servo_data.motor = motor;
  servo_data.max_speed = SPEED;
  servo_data.max_acceleration = ACCELERATION;
  servo_data.transmission_ratio = M2006P36_RATIO;
  servo_data.omega_pid_param = new float[3]{150, 4, 0};
  servo_data.max_iout = 2000;
  servo_data.max_out = 10000;
  servo = new control::ServoMotor(servo_data);
  servo->RegisterJamCallback(jam_callback, 0.305);

  dbus = new remote::DBUS(&huart1);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  control::MotorCANBase* motors[] = {motor};
  // bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
  bsp::GPIO *key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);
  while (true) {
    if (!key->Read()) {
      servo->SetMaxSpeed(0);
    } else {
      servo->SetTarget(servo->GetTarget() + LOAD_ANGLE, false);
      servo->SetMaxSpeed(SPEED);
    }
    servo->CalcOutput();
    control::MotorCANBase::TransmitOutput(motors, 1);
    osDelay(2);
  }
}
