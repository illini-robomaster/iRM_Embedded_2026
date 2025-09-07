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

#include "unitree_motor.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"

static control::UnitreeMotor* A1 = nullptr;

#define RX_SIGNAL (1 << 0)

const osThreadAttr_t A1TaskAttribute = {.name = "A1Task",
                                        .attr_bits = osThreadDetached,
                                        .cb_mem = nullptr,
                                        .cb_size = 0,
                                        .stack_mem = nullptr,
                                        .stack_size = 128 * 4,
                                        .priority = (osPriority_t)osPriorityNormal,
                                        .tz_module = 0,
                                        .reserved = 0};

osThreadId_t A1TaskHandle;

class CustomUART: public bsp::UART {
 public:
  using bsp::UART::UART;
 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() override final { osThreadFlagsSet(A1TaskHandle, RX_SIGNAL); }
};

static CustomUART* A1_uart = nullptr;

void A1Task(void* arg) {
  UNUSED(arg);
  uint32_t length;
  uint8_t* data;

  while (true) {
    /* wait until rx data is available */
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {  // unnecessary check
      /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
      length = A1_uart->Read(&data);
      if ((int)length == A1->recv_length)
        A1->ExtractData(communication::package_t{data, (int)length});
    }
  }
}

void RM_RTOS_Init(void) {
  print_use_uart(&huart6);

  A1_uart = new CustomUART(&huart1);
  A1_uart->SetupRx(300);
  A1_uart->SetupTx(300);

  A1 = new control::UnitreeMotor();
}

void RM_RTOS_Threads_Init(void) {
  A1TaskHandle = osThreadNew(A1Task, nullptr, &A1TaskAttribute);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  A1->Test(0);
  A1_uart->Write((uint8_t*)(&(A1->send[0].data)), A1->send_length);
  osDelay(3000);

  A1->Stop(0);
  A1_uart->Write((uint8_t*)(&(A1->send[0].data)), A1->send_length);
  osDelay(3000);

  A1->Control(0, 0.0, -1.0, 0.0, 0.0, 3.0); // constant speed mode
  for (int i = 0; i < 30; ++i) {
    A1_uart->Write((uint8_t*)(&(A1->send[0].data)), A1->send_length);
    set_cursor(0, 0);
    clear_screen();
    print("Motor ID: %d\r\n", A1->recv[0].id);
    print("Mode    : %d\r\n", A1->recv[0].mode);
    print("Temp    : %d\r\n", A1->recv[0].Temp);
    print("MError  : %d\r\n", A1->recv[0].MError);
    print("Torque  : %.3f\r\n", A1->recv[0].T);
    print("Speed   : %.3f\r\n", A1->recv[0].W);
    print("Accel   : %d\r\n", A1->recv[0].Acc);
    print("Position: %.3f\r\n", A1->recv[0].Pos);
    osDelay(100);
  }

  A1->Control(0, 0.0, 0.0, 0.0, 0.0, 0.0); // zero torque mode
  while (true) {
    A1_uart->Write((uint8_t*)(&(A1->send[0].data)), A1->send_length);
    set_cursor(0, 0);
    clear_screen();
    print("Motor ID: %d\r\n", A1->recv[0].id);
    print("Mode    : %d\r\n", A1->recv[0].mode);
    print("Temp    : %d\r\n", A1->recv[0].Temp);
    print("MError  : %d\r\n", A1->recv[0].MError);
    print("Torque  : %.3f\r\n", A1->recv[0].T);
    print("Speed   : %.3f\r\n", A1->recv[0].W);
    print("Accel   : %d\r\n", A1->recv[0].Acc);
    print("Position: %.3f\r\n", A1->recv[0].Pos);
    osDelay(100);
  }
}
