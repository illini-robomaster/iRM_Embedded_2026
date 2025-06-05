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

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "math.h"
#include "motor.h"
#include "utils.h"

#define LEFT_MOTOR_PWM_CHANNEL 1
#define LOADER_SLIDE_MOTOR_PWM_CHANNEL 2
#define LOADER_FEED_MOTOR_PWM_CHANNEL 3
#define TIM_CLOCK_FREQ 1000000
#define MOTOR_OUT_FREQ 50

#define KEY_GPIO_GROUP GPIOA
#define KEY_GPIO_PIN GPIO_PIN_0

#define DEFAULT_TASK_DELAY 100

#define MAX_IOUT 16384
#define MAX_OUT 60000

#define ABS(x) ((x) > 0 ? (x) : -(x))

#define MAP_RANGE(x, in_min, in_max, out_min, out_max) (((float)(x) - (float)(in_min)) * ((float)(out_max) - (float)(out_min)) / ((float)(in_max) - (float)(in_min)) + (float)(out_min))

bsp::GPIO* key = nullptr;

control::MotorPWMBase* trigger_motor;

control::MotorPWMBase* loader_slide_motor;
control::MotorPWMBase* loader_feed_motor;

control::MotorCANBase* load_motor_1;
control::MotorCANBase* load_motor_2;
control::MotorCANBase* force_motor;

static remote::DBUS* dbus;
static bsp::CAN* can1 = nullptr;  // for load motor

// variables:
static int trigger_motor_output = 1500;

static int16_t load_motor_1_output = 0;
static int16_t load_motor_2_output = 0;
static int16_t force_motor_output = 0;

static uint16_t load_motor_temperature = 0;
static int16_t load_motor_current = 0;

float Kp_load = 50;
float Ki_load = 15;
float Kd_load = 65;
// thread attributes

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
  // TODO: adjust the PID parameters based on different motor characteristics
  control::ConstrainedPID pid_left(param, MAX_IOUT, MAX_OUT);
  control::ConstrainedPID pid_right(param, MAX_IOUT, MAX_OUT);
  control::ConstrainedPID pid_force(param, MAX_IOUT, MAX_OUT);
  control::MotorCANBase* motors_can1_load[] = {load_motor_1, load_motor_2, force_motor};  // load motor
  float diff_load_1 = 0;
  float diff_load_2 = 0;
  float diff_force = 0;
  float load_target_speed = 0;   // target speed for load motor, can be adjusted
  float force_target_speed = 0;  // target speed for force motor, can be adjusted

  int slide_motor_output = 0;
  int feed_motor_output = 0;
  while (true) {
    // This is for the trigger motor
    if (dbus->swr == remote::UP) {  // when SWR is up, increase motor output
      trigger_motor->SetOutput(600);
      // print("trigger on\r\n");
    } else {
      trigger_motor->SetOutput(0);
    }

    // Load motor control logic
    if (dbus->swl == remote::UP) {  // when SWL is up, run the load motor
      // Set target speed for load motor
      load_target_speed = 300;  // adjust target speed based on dbus ch3 input
    } else if (dbus->swl == remote::DOWN) {
      load_target_speed = -170;
    } else {
      load_target_speed = 0;  // stop the load motor when SWL is at mid
    }
    slide_motor_output += MAP_RANGE(dbus->ch0, -660, 660, -10, 10);
    feed_motor_output += MAP_RANGE(dbus->ch1, -660, 660, -10, 10);

    loader_feed_motor->SetOutput(feed_motor_output);
    loader_slide_motor->SetOutput(slide_motor_output);

    force_target_speed = MAP_RANGE(dbus->ch3, -660, 660, -300, 300);  // map ch0 to target speed for force motor

    // Compute the omega delta for PID control
    diff_load_1 = load_motor_1->GetOmegaDelta(-load_target_speed);  // Get the current speed difference
    diff_load_2 = load_motor_2->GetOmegaDelta(load_target_speed);   // Get the current speed difference for second motor if needed
    diff_force = force_motor->GetOmegaDelta(force_target_speed);    // Get the current speed difference for force motor

    load_motor_1_output = pid_left.ComputeConstrainedOutput(diff_load_1);
    load_motor_2_output = pid_right.ComputeConstrainedOutput(diff_load_2);
    force_motor_output = pid_force.ComputeConstrainedOutput(diff_force);

    load_motor_1->SetOutput(load_motor_1_output);
    load_motor_2->SetOutput(load_motor_2_output);
    force_motor->SetOutput(0);
    load_motor_temperature = load_motor_1->GetTemp();
    load_motor_current = load_motor_1->GetCurr();
    // print("Load Motor 1 Output: %d, Load Motor 2 Output: %d, Load Motor Temperature: %d, Load Motor Current: %d\r\n",
    //       load_motor_1_output, load_motor_2_output, load_motor_temperature, load_motor_current);
    // print("Force Motor Speed: %d, Load Motor 1 Speed: %d, Load Motor 2 Speed: %d, Load Motor Temperature: %d, Load Motor Current: %d\r\n",
    //       force_target_speed, load_motor_1->GetOmega(), load_motor_2->GetOmega(), load_motor_temperature, load_motor_current);
    print("loader_slide_motor output: %d\r\n", dbus->ch1);
    print("loader_feed_motor output: %d\r\n", dbus->ch2);
    control::MotorCANBase::TransmitOutput(motors_can1_load, 3);  // Transmit the output to the load motor

    osDelay(10);

    // Load motor control
  }
}
void reload_task(void* arg) {
  UNUSED(arg);
  // When called, this task will reload the dart, which is a fixed action

  return;
}

void RM_RTOS_Init(){
    print_use_uart(&huart1);
    key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);
    trigger_motor = new control::MotorPWMBase(&htim1, LEFT_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, trigger_motor_output);
    loader_slide_motor = new control::MotorPWMBase(&htim1, LOADER_SLIDE_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, 0);
    loader_feed_motor = new control::MotorPWMBase(&htim1, LOADER_FEED_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, 0);
    trigger_motor->SetOutput(0);
    can1 = new bsp::CAN(&hcan1); // can1 for load motor, make sure to initialize can before motor
    load_motor_1 = new control::Motor3508(can1, 0x201);
    load_motor_2 = new control::Motor3508(can1, 0x202);
    force_motor = new control::Motor2006(can1, 0x203);  // force motor, can be used for other purposes
    dbus = new remote::DBUS(&huart3);
}



void RM_RTOS_Threads_Init(void) {
    dartLoadTaskHandle = osThreadNew(dartLoadTask, NULL, &dartLoadTaskAttribute);
    if (dartLoadTaskHandle == NULL) {
        print("Failed to create dart load task\r\n");
    }
}

void RM_RTOS_Default_Task(const void* args){
    UNUSED(args);
    while(true){
      osDelay(DEFAULT_TASK_DELAY);
    }
}


