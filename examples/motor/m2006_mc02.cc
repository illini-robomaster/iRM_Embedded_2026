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

/**
 * @brief Motor 2006 velocity PID control example for DM_MC_02 board
 * 
 * Hardware configuration:
 * - Board: DM_MC_02 (STM32H723, uses FDCAN)
 * - Motor: M2006 connected to FDCAN1
 * - Control: USB Virtual COM Port
 * 
 * Control:
 * - 'w' or 'W': Spin forward at target velocity (rad/s)
 * - 's' or 'S': Spin backward at target velocity (rad/s)
 * - Any other key: Stop motor
 * 
 * Connect via USB and use a serial terminal (e.g., screen, minicom, or Python serial)
 */

#include "bsp_print.h"
#include "bsp_usb.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "motor.h"

#define RX_SIGNAL (1 << 0)
#define TARGET_VELOCITY 40.0f  // Target velocity in rad/s (slow speed)

extern osThreadId_t defaultTaskHandle;

// For DM_MC_02, bsp::CAN is aliased to bsp::FDCAN via bsp_can_bridge.h
static bsp::CAN* can = nullptr;
static control::MotorCANBase* motor = nullptr;
static bsp::VirtualUSB* usb = nullptr;

// Target velocity (updated by USB input)
static volatile float target_velocity = 0.0f;

/**
 * @brief Custom USB callback class
 */
class CustomUSB : public bsp::VirtualUSB {
 protected:
  void RxCompleteCallback() override final { 
    osThreadFlagsSet(defaultTaskHandle, RX_SIGNAL); 
  }
};

void RM_RTOS_Init() {
  // Initialize USB
  usb = new CustomUSB();
  usb->SetupTx(512);
  usb->SetupRx(512);
  
  // DM_MC_02 uses FDCAN instead of CAN
  can = new bsp::CAN(&hfdcan1, 0);
  
  // Motor 2006 with CAN ID 0x206
  motor = new control::Motor2006(can, 0x206);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  
  control::MotorCANBase* motors[] = {motor};
  char msg[256];
  int len;
  
  // Velocity PID controller
  // Parameters: Kp, Ki, Kd
  control::PIDController velocity_pid(150.0f, 5.0f, 0.0f);
  
  osDelay(1000);  // Wait for USB to initialize
  
  len = snprintf(msg, sizeof(msg), 
    "\r\n========================================\r\n"
    "  M2006 Velocity PID Control via USB\r\n"
    "  Press 'w' = forward %.1f rad/s\r\n"
    "  Press 's' = backward %.1f rad/s\r\n"
    "  Any other key = stop\r\n"
    "========================================\r\n\r\n", 
    TARGET_VELOCITY, TARGET_VELOCITY);
  usb->Write((uint8_t*)msg, len);

  int print_counter = 0;
  
  while (true) {
    // Check for USB input with short timeout
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, 10);
    
    if (flags & RX_SIGNAL) {
      uint8_t* data;
      uint32_t length = usb->Read(&data);
      
      if (length > 0) {
        char key = data[0];
        
        if (key == 'w' || key == 'W') {
          target_velocity = TARGET_VELOCITY;
          len = snprintf(msg, sizeof(msg), "Target: %.2f rad/s\r\n", target_velocity);
          usb->Write((uint8_t*)msg, len);
        } else if (key == 's' || key == 'S') {
          target_velocity = -TARGET_VELOCITY;
          len = snprintf(msg, sizeof(msg), "Target: %.2f rad/s\r\n", target_velocity);
          usb->Write((uint8_t*)msg, len);
        } else {
          target_velocity = 0.0f;
          len = snprintf(msg, sizeof(msg), "Stop\r\n");
          usb->Write((uint8_t*)msg, len);
        }
      }
    }
    
    // Velocity PID control
    float current_velocity = motor->GetOmega();
    float velocity_error = target_velocity - current_velocity;
    int16_t output = static_cast<int16_t>(velocity_pid.ComputeConstrainedOutput(velocity_error));
    
    motor->SetOutput(output);
    control::MotorCANBase::TransmitOutput(motors, 1);
    
    // Print status every 500ms
    if (++print_counter >= 250) {  // 250 * 2ms = 500ms
      print_counter = 0;
      len = snprintf(msg, sizeof(msg), "Target: %6.2f | Actual: %6.2f | Output: %5d\r\n",
                     target_velocity, current_velocity, output);
      usb->Write((uint8_t*)msg, len);
    }
    
    osDelay(2);  // Motor control at 500Hz
  }
}
