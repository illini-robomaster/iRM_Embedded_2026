/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2025 RoboMaster.                                          *
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

#include "bsp_buzzer.h"
#include "bsp_gpio.h"
#include "bsp_laser.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"

#define LEFT_MOTOR_PWM_CHANNEL 1
#define TIM_CLOCK_FREQ 1000000
#define MOTOR_OUT_FREQ 50
#define IDLE_THROTTLE 1500
#define DEFAULT_TASK_DELAY 100

#define KEY_GPIO_GROUP K1_GPIO_Port
#define KEY_GPIO_PIN K1_Pin

#define MAX_IOUT 16384
#define MAX_OUT 60000

#define MOTOR_TEMP_HIGH_THRESHOLD 80
#define MOTOR_TEMP_LOW_THRESHOLD 40
#define ALARM_INTERVAL 100

#define MAP_RANGE(x, in_min, in_max, out_min, out_max) \
  (((float)(x) - (float)(in_min)) * ((float)(out_max) - (float)(out_min)) / \
  ((float)(in_max) - (float)(in_min)) + (float)(out_min))

// Peripherals
bsp::GPIO* key = nullptr;
bsp::Buzzer* buzzer = nullptr;
bsp::Laser* laser = nullptr;

// Alarm sound for overheating
using Note = bsp::BuzzerNote;
static bsp::BuzzerNoteDelayed AlarmSound[] = {
    {Note::Do1H, 100}, {Note::Silent, 50}, {Note::Do1H, 100}, {Note::Silent, 50},
    {Note::Do1H, 100}, {Note::Silent, 200}, {Note::Silent, 0}, {Note::Finish, 0}};

// Motors
control::MotorPWMBase* trigger_motor = nullptr;
control::MotorCANBase* load_motor_1 = nullptr;
control::MotorCANBase* load_motor_2 = nullptr;
control::MotorCANBase* force_motor = nullptr;
control::MotorCANBase* yaw_motor = nullptr;

// Communication
static remote::DBUS* dbus = nullptr;
static bsp::CAN* can1 = nullptr;

// State variables
static uint16_t load_motor_temperature = 0;
static bool motor_cooling_state = false;
static uint32_t alarm_counter = 0;

// PID parameters
float Kp_load = 50;
float Ki_load = 15;
float Kd_load = 65;

osThreadId_t dartLoadTaskHandle;
const osThreadAttr_t dartLoadTaskAttribute = {.name = "dartLoadTask",
                                              .attr_bits = osThreadDetached,
                                              .cb_mem = nullptr,
                                              .cb_size = 0,
                                              .stack_mem = nullptr,
                                              .stack_size = 256 * 4,
                                              .priority = (osPriority_t)osPriorityNormal,
                                              .tz_module = 0,
                                              .reserved = 0};

void dartLoadTask(void* arg) {
  UNUSED(arg);
  float param[] = {Kp_load, Ki_load, Kd_load};
  control::PIDController pid_yaw(50, 5, 10);
  control::ConstrainedPID pid_left(param, MAX_IOUT, MAX_OUT);
  control::ConstrainedPID pid_right(param, MAX_IOUT, MAX_OUT);
  control::ConstrainedPID pid_force(param, MAX_IOUT, MAX_OUT);

  control::MotorCANBase* motors_can1_load[] = {load_motor_1, load_motor_2, force_motor};
  control::MotorCANBase* yaw_motors[] = {yaw_motor};

  float load_target_speed = 0;
  float force_target_speed = 0;
  float yaw_target_speed = 0;

  while (true) {
    // Temperature protection
    load_motor_temperature = load_motor_1->GetTemp();
    if (load_motor_temperature > MOTOR_TEMP_HIGH_THRESHOLD && !motor_cooling_state) {
      motor_cooling_state = true;
      print("Motor overheated! Temperature: %d C\r\n", load_motor_temperature);
    } else if (load_motor_temperature < MOTOR_TEMP_LOW_THRESHOLD && motor_cooling_state) {
      motor_cooling_state = false;
      print("Motor cooled down! Temperature: %d C\r\n", load_motor_temperature);
    }

    if (motor_cooling_state) {
      alarm_counter++;
      if (alarm_counter >= ALARM_INTERVAL) {
        buzzer->SingSong(AlarmSound);
        alarm_counter = 0;
      }
    } else {
      alarm_counter = 0;
    }

    // Trigger motor control
    if (dbus->swr == remote::UP) {
      trigger_motor->SetOutput(0);
    } else if (dbus->swr == remote::DOWN) {
      trigger_motor->SetOutput(600);
    } else {
      trigger_motor->SetOutput(300);
    }

    // Load motor control
    if (dbus->swl == remote::UP) {
      load_target_speed = 300;
    } else if (dbus->swl == remote::DOWN) {
      load_target_speed = -270;
    } else {
      load_target_speed = 0;
    }

    // Yaw motor control
    if (dbus->ch0 > 300) {
      yaw_target_speed = 100;
    } else if (dbus->ch0 < -300) {
      yaw_target_speed = -100;
    } else {
      yaw_target_speed = 0;
    }

    // Force motor control
    force_target_speed = MAP_RANGE(dbus->ch3, -660, 660, -500, 500);

    // PID control for load motors
    float diff_load_1 = load_motor_1->GetOmegaDelta(-load_target_speed);
    float diff_load_2 = load_motor_2->GetOmegaDelta(load_target_speed);
    float diff_force = force_motor->GetOmegaDelta(force_target_speed);

    int16_t load_motor_1_output = pid_left.ComputeConstrainedOutput(diff_load_1);
    int16_t load_motor_2_output = pid_right.ComputeConstrainedOutput(diff_load_2);
    int16_t force_motor_output = pid_force.ComputeConstrainedOutput(diff_force);

    load_motor_1->SetOutput(load_motor_1_output);
    load_motor_2->SetOutput(load_motor_2_output);
    force_motor->SetOutput(force_motor_output);
    control::MotorCANBase::TransmitOutput(motors_can1_load, 3);

    // PID control for yaw motor
    float diff_yaw = yaw_motor->GetOmegaDelta(yaw_target_speed);
    int16_t yaw_output = pid_yaw.ComputeConstrainedOutput(diff_yaw);
    yaw_motor->SetOutput(yaw_output);
    control::MotorCANBase::TransmitOutput(yaw_motors, 1);

    osDelay(5);
  }
}

void RM_RTOS_Init() {
  print_use_uart(&huart8);

  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);
  buzzer = new bsp::Buzzer(&htim12, 1, 1000000);
  laser = new bsp::Laser(LASER_GPIO_Port, LASER_Pin);

  trigger_motor = new control::MotorPWMBase(&htim4, LEFT_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ,
                                            MOTOR_OUT_FREQ, IDLE_THROTTLE);

  can1 = new bsp::CAN(&hcan1);
  load_motor_1 = new control::Motor3508(can1, 0x201);
  load_motor_2 = new control::Motor3508(can1, 0x202);
  force_motor = new control::Motor2006(can1, 0x204);
  yaw_motor = new control::Motor3508(can1, 0x205);

  dbus = new remote::DBUS(&huart1);
  laser->On();
}

void RM_RTOS_Threads_Init(void) {
  dartLoadTaskHandle = osThreadNew(dartLoadTask, NULL, &dartLoadTaskAttribute);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  while (true) {
    osDelay(DEFAULT_TASK_DELAY);
  }
}
