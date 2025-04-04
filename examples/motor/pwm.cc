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
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"

#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

// Refer to typeA datasheet for channel detail
#define LEFT_MOTOR_PWM_CHANNEL 1
#define RIGHT_MOTOR_PWM_CHANNEL 4
#define TIM_CLOCK_FREQ 1000000
#define MOTOR_OUT_FREQ 1500
#define SNAIL_IDLE_THROTTLE 500

control::MotorPWMBase* trigger_motor;
control::MotorPWMBase* motor2;

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  trigger_motor = new control::MotorPWMBase(&htim1, LEFT_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ,
                                     SNAIL_IDLE_THROTTLE);
  motor2 = new control::MotorPWMBase(&htim1, RIGHT_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ,
                                     MOTOR_OUT_FREQ, SNAIL_IDLE_THROTTLE);
  trigger_motor->SetOutput(0);
  motor2->SetOutput(0);
  // Snail need to be run at idle throttle for some
  osDelay(3000);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  bsp::GPIO key(KEY_GPIO_GROUP, GPIO_PIN_2);

  while (true) {
    if (key.Read()) {
      trigger_motor->SetOutput(300);
      motor2->SetOutput(300);
    } else {
      trigger_motor->SetOutput(0);
      motor2->SetOutput(0);
    }
    osDelay(1000);
  }
}
