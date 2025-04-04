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

#include "main.h"

#include <cstring>
#include <memory>

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "minipc_protocol.h"
#include "controller.h"
#include "motor.h"

#include "bsp_gpio.h"

#define RX_SIGNAL (1 << 0)

// They don't need to be different. RX_SIGNAL is thread flag, VX_READY_SIGNAL is event flag
#define VX_READY_SIGNAL (1 << 1)

osEventFlagsId_t vx_flag_id;

static bsp::GPIO *gpio_red;

static float vx = 0;

/* init new task START */
static osThreadId_t MotorTaskHandle;

const osThreadAttr_t MotorTaskAttributes = {.name = "MotorTask",
                                           .attr_bits = osThreadDetached,
                                           .cb_mem = nullptr,
                                           .cb_size = 0,
                                           .stack_mem = nullptr,
                                           .stack_size = 128 * 4,
                                           .priority = (osPriority_t)osPriorityNormal,
                                           .tz_module = 0,
                                           .reserved = 0};


bsp::CAN* can = nullptr;
control::MotorCANBase* trigger_motor = nullptr;

void MotorTask(void* argument) {
  UNUSED(argument);
  uint32_t flags;
  control::PIDController pid1(20, 0, 0);
  control::MotorCANBase* motors[] = {trigger_motor};

  while (true) {
    // Wait time = 50 ticks, 50ms?
    flags = 0;
    flags = osEventFlagsWait(vx_flag_id, VX_READY_SIGNAL, osFlagsWaitAny, 50);
    // When timeout it returns -2 so we need extra checks here
    if (flags != osFlagsErrorTimeout && flags & VX_READY_SIGNAL) {
      // if receives packet, drive the motor and toggle RED LED
      float diff = 0;
      int16_t out = 0;
      diff = trigger_motor->GetOmegaDelta(vx);
      out = pid1.ComputeConstrainedOutput(diff);
      trigger_motor->SetOutput(out);
      gpio_red->Toggle();
    } else {
      // if timeout (no packet, stop the motor)
      float diff = 0;
      int16_t out = 0;
      diff = trigger_motor->GetOmegaDelta(0);
      out = pid1.ComputeConstrainedOutput(diff);
      trigger_motor->SetOutput(out);
    }
    control::MotorCANBase::TransmitOutput(motors, 1);
    osDelay(10);
  }
}

extern osThreadId_t defaultTaskHandle;

class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() override final { osThreadFlagsSet(defaultTaskHandle, RX_SIGNAL); }
};


void RM_RTOS_Threads_Init(void) {
  MotorTaskHandle = osThreadNew(MotorTask, nullptr, &MotorTaskAttributes);
}

void RM_RTOS_Init(void) {
  can = new bsp::CAN(&hcan1, true);
  trigger_motor = new control::Motor3508(can, 0x201);
  gpio_red = new bsp::GPIO(LED_RED_GPIO_Port, LED_RED_Pin);
  gpio_red->High();
  vx_flag_id = osEventFlagsNew(nullptr);
}

void RM_RTOS_Default_Task(const void* argument) {
  UNUSED(argument);

  auto uart = std::make_unique<CustomUART>(&huart8); 
  uart->SetupRx(50);
  uart->SetupTx(50);

  auto minipc_session = communication::MinipcPort();

  const communication::status_data_t* status_data;

  uint8_t *data;
  int32_t length;

  // When packet arrives, raise a eventflag
  while (true) {
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {
      length = uart->Read(&data);
      minipc_session.ParseUartBuffer(data, length);
      status_data = minipc_session.GetStatus();
      vx = status_data->vx;
      osEventFlagsSet(vx_flag_id, VX_READY_SIGNAL);
    }
    osDelay(10);
  }
}

