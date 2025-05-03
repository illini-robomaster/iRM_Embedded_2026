
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

#define KEY_GPIO_GROUP GPIOA
#define KEY_GPIO_PIN GPIO_PIN_0


#define DEFAULT_TASK_DELAY 100

#define JOINT1_PWM_CHANNEL 2
#define JOINT2_PWM_CHANNEL 3
#define CLAW_MOTOR_PWM_CHANNEL 1

#define TIM_CLOCK_FREQ 1000000
#define MOTOR_OUT_FREQ 50

#define MAX_IOUT 16384
#define MAX_OUT 60000

#define MAP_RANGE(x, in_min, in_max, out_min, out_max) (((float)(x) - (float)(in_min)) * ((float)(out_max) - (float)(out_min)) / ((float)(in_max) - (float)(in_min)) + (float)(out_min))

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



bsp::GPIO* key = nullptr;
control::MotorPWMBase* claw_motor = nullptr;
control::MotorPWMBase* loader_joint1 = nullptr;
control::MotorPWMBase* loader_joint2 = nullptr;

control::MotorCANBase* slide_motor = nullptr;

static remote::DBUS *dbus = nullptr;

static bsp::CAN* can1 = nullptr;

float Kp_slide = 50;
float Ki_slide = 15;
float Kd_slide = 65;
float diff_output = 0;

void dartLoadTask(void*arg){
  UNUSED(arg);
  int joint1_output = 900;
  int joint2_output = 0;
  float slide_speed = 0;

  control::MotorCANBase* slide_[] = {slide_motor};
  float diff_slide = 0;
  float param[] = {Kp_slide, Ki_slide, Kd_slide};
  control::ConstrainedPID pid(param, MAX_IOUT, MAX_OUT);

  while(1){
    if (dbus->swl == remote::UP) {
      claw_motor->SetOutput(1000);
    } else {
      claw_motor->SetOutput(500);
    }
    // joint1_output = MAP_RANGE(dbus->ch3, -660, 660, 500, 2500);
    joint2_output = MAP_RANGE(dbus->ch1, -660, 660, 500, 2500);
    slide_speed = MAP_RANGE(dbus->ch2, -660, 660, -50, 50);
    // loader_joint1->SetOutput(joint1_output);
    loader_joint2->SetOutput(joint2_output);

    diff_slide = slide_motor->GetOmegaDelta(slide_speed);
    diff_output = pid.ComputeConstrainedOutput(diff_slide);
    slide_motor->SetOutput(diff_output);
    control::MotorCANBase::TransmitOutput(slide_, 1);

    print("joint1: %d\r\n", joint1_output);
    print("joint2: %d\r\n", joint2_output);
    print("slide speed: %d , diff_slide: %.2f, diff_output: %f \r\n", slide_speed, diff_slide, diff_output);
    osDelay(10);
  }
}


void RM_RTOS_Init(){
  print_use_uart(&huart1);

  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);
  claw_motor = new control::MotorPWMBase(&htim1, CLAW_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, 0);
  loader_joint1 = new control::MotorPWMBase(&htim1, JOINT1_PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, 0);
  can1 = new bsp::CAN(&hcan1);
  loader_joint2 = new control::MotorPWMBase(&htim1, JOINT2_PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, 0);
  slide_motor = new control::Motor3508(can1, 0x202);
  dbus = new remote::DBUS(&huart3);
}


void RM_RTOS_Threads_Init(void) {
    dartLoadTaskHandle = osThreadNew(dartLoadTask, NULL, &dartLoadTaskAttribute);
    if (dartLoadTaskHandle == NULL) {
        print("Failed to create dart load task\r\n");
        Error_Handler();
    }
}

void RM_RTOS_Default_Task(const void* args){
    UNUSED(args);
    while(true){
      osDelay(DEFAULT_TASK_DELAY);
    }
}