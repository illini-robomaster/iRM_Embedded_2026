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

#include <memory>

#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"

#define RX_SIGNAL (1 << 0)

extern osThreadId_t defaultTaskHandle;

class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  void RxCompleteCallback() override final {
    osThreadFlagsSet(defaultTaskHandle, RX_SIGNAL);
  }
};

void RM_RTOS_Init(void) {
  print_use_usb();
}

void RM_RTOS_Default_Task(const void* argument) {
  UNUSED(argument);
  // Wait for USB CDC to enumerate with host PC
  osDelay(3000);

  uint32_t length;
  uint8_t* data;

  print("USB Print Test Started\r\n");

  auto uart = std::make_unique<CustomUART>(&UART_HANDLE);
  uart->SetupTx(50);
  uart->SetupRx(50);
  print("UART initialized - ready to receive data\r\n");

  while (true) {
    // Wait for RX signal with 1 second timeout
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, 1000);

    if (flags & RX_SIGNAL) {
      // Data received on UART
      length = uart->Read(&data);
      print("Received %lu bytes from UART\r\n", length);

      // Echo back 3 times
      uart->Write(data, length);
      uart->Write(data, length);
      uart->Write(data, length);
      print("Echoed data 3 times\r\n");
    } else {
      // Timeout - print status
      print("Waiting for UART data...\r\n");
    }
  }
}
