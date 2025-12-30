/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2024 RoboMaster.                                          *
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
 * @file usb_mc02.cc
 * @brief USB CDC Example for DM_MC_02
 *
 * This example demonstrates USB Virtual COM Port (VCP) communication
 * on the DM_MC_02 board using USB High-Speed (HS) mode.
 *
 * Hardware Setup:
 *   - Connect DM_MC_02 to PC via USB cable
 *   - On macOS: device appears as /dev/cu.usbmodemXXXX
 *   - On Linux: device appears as /dev/ttyACMX
 *   - On Windows: COM port (may need driver)
 *
 * Test with Python:
 * ```python
 * import serial
 * import time
 *
 * # Find the correct port for your system
 * ser = serial.Serial('/dev/cu.usbmodem1234', baudrate=115200, timeout=1)
 * time.sleep(2)  # Wait for connection
 *
 * # Read printed messages
 * while True:
 *     if ser.in_waiting:
 *         print(ser.readline().decode(), end='')
 *
 * # Or send data and receive echo
 * ser.write(b'Hello MC02!')
 * response = ser.read(3 * len(b'Hello MC02!'))
 * print(response)
 * ```
 */

#include "main.h"

#include "bsp_usb.h"
#include "cmsis_os.h"
#include "printf.h"

#define RX_SIGNAL (1 << 0)
#define MAX_MSG_LEN 256

extern osThreadId_t defaultTaskHandle;

static bsp::VirtualUSB* usb = nullptr;
static char msg_buffer[MAX_MSG_LEN];

// Helper function to print via USB
static void usb_print(const char* format, ...) {
  va_list args;
  va_start(args, format);
  int len = vsnprintf(msg_buffer, MAX_MSG_LEN, format, args);
  va_end(args);
  if (len > 0 && usb) {
    usb->Write((uint8_t*)msg_buffer, len);
  }
}

/**
 * @brief Custom USB callback class
 * 
 * Override RxCompleteCallback to notify the main task when data is received.
 */
class CustomUSBCallback : public bsp::VirtualUSB {
 protected:
  void RxCompleteCallback() override final { 
    osThreadFlagsSet(defaultTaskHandle, RX_SIGNAL); 
  }
};

void RM_RTOS_Init(void) {
  // Initialize USB with TX and RX buffers
  // Note: Only one VirtualUSB instance can exist at a time
  usb = new CustomUSBCallback();
  usb->SetupTx(2048);
  usb->SetupRx(2048);

  // Use the same USB instance for print (do NOT call print_use_usb() as it creates another instance)
  // We'll use usb->Write() directly for output
}

void RM_RTOS_Default_Task(const void* argument) {
  UNUSED(argument);

  uint32_t length;
  uint8_t* data;
  uint32_t counter = 0;

  usb_print("\r\n");
  usb_print("========================================\r\n");
  usb_print("  DM_MC_02 USB CDC Example\r\n");
  usb_print("  USB High-Speed Mode\r\n");
  usb_print("========================================\r\n");
  usb_print("\r\n");
  usb_print("Send any data to receive 3x echo.\r\n");
  usb_print("Counter updates every second.\r\n");
  usb_print("\r\n");

  while (true) {
    // Wait for RX data with timeout (1 second)
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, 1000);

    if (flags & RX_SIGNAL) {
      // Data received - echo it back 3 times
      length = usb->Read(&data);
      if (length > 0) {
        usb_print("Received %lu bytes: ", length);
        
        // Print received data (if printable)
        for (uint32_t i = 0; i < length && i < 64; i++) {
          if (data[i] >= 32 && data[i] < 127) {
            usb_print("%c", data[i]);
          } else {
            usb_print(".");
          }
        }
        usb_print("\r\n");

        // Echo back 3 times
        usb->Write(data, length);
        usb->Write(data, length);
        usb->Write(data, length);
      }
    } else {
      // Timeout - print periodic status
      usb_print("[%lu] USB CDC running...\r\n", counter++);
    }
  }
}
