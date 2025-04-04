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
#include "main.h"
#include "motor.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "utils.h"
#include "controller.h"

#define LEFT_MOTOR_PWM_CHANNEL 1
#define TIM_CLOCK_FREQ 1000000
#define MOTOR_OUT_FREQ 50

#define KEY_GPIO_GROUP GPIOA
#define KEY_GPIO_PIN GPIO_PIN_0

#define DEFAULT_TASK_DELAY 100

#define MAX_IOUT 16384
#define MAX_OUT 60000

bsp::GPIO* key = nullptr;
control::MotorPWMBase* trigger_motor;
control::MotorCANBase *load_motor;
static remote::DBUS *dbus;
static bsp::CAN *can1 = nullptr;  // for load motor

// variables:
static int motor1_output = 1500;
static int load_output = 0;

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


void dartLoadTask(void *arg) {
  UNUSED(arg);
  float param[] = {Kp_load,Ki_load,Kd_load};
  control::ConstrainedPID pid(param, MAX_IOUT, MAX_OUT); 
  control::MotorCANBase* motors_can1_load[] = {load_motor};  // load motor
  float diff_load = 0;
  float load_target_speed = 0; // target speed for load motor, can be adjusted
//  int ch3 = 0;

  while (true) {
//    ch3 = abs(dbus->ch3) >=50 ? dbus->ch3 : 0;
    // This is for the trigger motor
    if (dbus->swr == remote::UP) {  // when SWR is up, increase motor output
      trigger_motor->SetOutput(600);
      print("trigger on\r\n");
    } else {
      trigger_motor->SetOutput(0);
    }

    // Load motor control logic
    if (dbus->swl == remote::UP) {  // when SWL is up, run the load motor
      // Set target speed for load motor
      load_target_speed = 300; // adjust target speed based on dbus ch3 input
    } 
    else if (dbus->swl == remote::DOWN){
      load_target_speed = -180;
    }
    else {
      load_target_speed = 0; // stop the load motor when SWL is down
    }

    // Compute the omega delta for PID control
    diff_load = load_motor->GetOmegaDelta(load_target_speed); // Get the current speed difference
    load_output = pid.ComputeConstrainedOutput(diff_load);
    load_motor->SetOutput(load_output);

    control::MotorCANBase::TransmitOutput(motors_can1_load, 1); // Transmit the output to the load motor

    osDelay(10);

    // Load motor control
//    diff = motors_can1_load->GetOmegaDelta(TARGET_SPEED);
//    int16_t out = pid.ComputeConstrainedOutput();

  }
}


void RM_RTOS_Init(){
    print_use_uart(&huart1);
    key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);
    trigger_motor = new control::MotorPWMBase(&htim1, LEFT_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, motor1_output);
    trigger_motor->SetOutput(0);
    can1 = new bsp::CAN(&hcan1); // can1 for load motor, make sure to initialize can before motor
    load_motor = new control::Motor3508(can1, 0x201);
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


