
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


#define MAP_RANGE(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

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

static remote::DBUS *dbus = nullptr;


void dartLoadTask(void*arg){
  UNUSED(arg);
  int joint1_output = 900;
  int joint2_output = 0;

  while(1){
    if (dbus->swl == remote::UP){
      claw_motor->SetOutput(1000);
    } else {
      claw_motor->SetOutput(500);
    }
    // joint1_output = MAP_RANGE(dbus->ch3, -660, 660, 500, 2500);
    joint2_output = MAP_RANGE(dbus->ch1, -660, 660, 500, 2500);
    loader_joint1->SetOutput(joint1_output);
    loader_joint2->SetOutput(joint2_output);
    print("joint1: %d\r\n", joint1_output);
    print("joint2: %d\r\n", joint2_output);
    osDelay(20);
  }
}


void RM_RTOS_Init(){
  print_use_uart(&huart1);
  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);
  claw_motor = new control::MotorPWMBase(&htim1, CLAW_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, 0);
  loader_joint1 = new control::MotorPWMBase(&htim1, JOINT1_PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, 0);
  
  loader_joint2 = new control::MotorPWMBase(&htim1, JOINT2_PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, 0);
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