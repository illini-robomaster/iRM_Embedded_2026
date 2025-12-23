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

#include <memory>
#include <cstdlib>


#include "bsp_laser.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"

#include "main.h"

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "math.h"
#include "motor.h"
#include "utils.h"

control::MotorCANBase* yaw_motor;

static bsp::CAN* can1 = nullptr; 
float yaw_target_speed = 20;
float target_diff_threshold = 5;

static bsp::Laser* laser = nullptr;


#define RX_SIGNAL (1 << 0)

extern osThreadId_t defaultTaskHandle;

class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() override final { osThreadFlagsSet(defaultTaskHandle, RX_SIGNAL); }
};

void RM_RTOS_Init(){
    print_use_uart(&huart6);
    can1 = new bsp::CAN(&hcan1, true);
    yaw_motor = new control::Motor3508(can1, 0x205);
    laser = new bsp::Laser(LASER_GPIO_Port, LASER_Pin);
}
void RM_RTOS_Default_Task(const void* argument) {
  UNUSED(argument);

  control::MotorCANBase* yaw_motors[] = {yaw_motor};
  control::PIDController pid(20, 15, 30);
  uint8_t* data;

  auto uart = std::make_unique<CustomUART>(&huart1);  // see cmake for which uart
  uart->SetupRx(50);
  uart->SetupTx(50);
  laser->On();

  while (true) {
    /* wait until rx data is available */
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {
      /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
      uint32_t length = uart->Read(&data);
      
      // Check if data starts with '!' and has at least one more character
      if (length >= 2 && data[0] == '!') {
        // Create a null-terminated buffer for safe parsing
        char buffer[16];
        uint32_t num_len = length - 1;  // Length excluding '!'
        if (num_len > 15) num_len = 15;  // Limit to buffer size
        
        // Copy the number part (skip '!') and null-terminate
        for (uint32_t i = 0; i < num_len; i++) {
          buffer[i] = data[i + 1];
        }
        buffer[num_len] = '\0';
        
        // Parse the number
        int number = atoi(buffer);
        
        // Print only the number
        print("%d\r\n", number);
        if (number > target_diff_threshold && number < 999) {
          yaw_target_speed = 40;
        } else if (number < -target_diff_threshold) {
          yaw_target_speed = -40;
        } else {
          yaw_target_speed = 0;
        }
      }
    }
    float diff = yaw_motor->GetOmegaDelta(yaw_target_speed);  
    int16_t out = pid.ComputeConstrainedOutput(diff);
    yaw_motor->SetOutput(out);
    control::MotorCANBase::TransmitOutput(yaw_motors, 1);
    yaw_motor->PrintData();
  }
  
}
