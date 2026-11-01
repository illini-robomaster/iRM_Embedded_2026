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

#include "bsp_print.h"

#include "FreeRTOS.h"
#include "bsp_uart.h"
#include "bsp_usb.h"
#include "main.h"
#include "printf.h"  // third party tiny-printf implemnetations
#include "semphr.h"
#include "task.h"

#define MAX_PRINT_LEN 512  // Increased from 256 for larger IMU output

static bsp::UART* print_uart = NULL;
static bsp::VirtualUSB* print_usb = NULL;
static char print_buffer[MAX_PRINT_LEN];
// print_buffer is shared by every task that prints, so formatting and queuing are
// serialized. Interrupts and pre-scheduler code print unlocked.
static StaticSemaphore_t print_mutex_storage;
static SemaphoreHandle_t print_mutex = NULL;

static void print_mutex_init() {
  if (!print_mutex) print_mutex = xSemaphoreCreateMutexStatic(&print_mutex_storage);
}

void print_use_uart(UART_HandleTypeDef* huart) {
  print_mutex_init();
  if (print_uart) delete print_uart;

  print_uart = new bsp::UART(huart);
  print_uart->SetupTx(MAX_PRINT_LEN * 2);  // burst transfer size up to 2x max buffer size
  print_usb = NULL;
}

void print_use_usb() {
  print_mutex_init();
  if (!print_usb) print_usb = new bsp::VirtualUSB();

  print_usb->SetupTx(MAX_PRINT_LEN * 2);  // burst transfer size up to 2x max buffer size
  print_usb->SetupRx(MAX_PRINT_LEN * 2);  // also setup RX buffer for proper USB CDC operation
  print_uart = NULL;
}

int32_t print(const char* format, ...) {
#ifdef NDEBUG
  UNUSED(format);
  UNUSED(print_buffer);
  return 0;
#else   // == #ifdef DEBUG
  va_list args;
  int length;
  int32_t written = 0;
  const bool locked = print_mutex && __get_IPSR() == 0 &&
                      xTaskGetSchedulerState() == taskSCHEDULER_RUNNING &&
                      xSemaphoreTake(print_mutex, portMAX_DELAY) == pdTRUE;

  va_start(args, format);
  length = vsnprintf(print_buffer, MAX_PRINT_LEN, format, args);
  va_end(args);

  if (print_uart)
    written = print_uart->Write((uint8_t*)print_buffer, length);
  else if (print_usb)
    written = print_usb->Write((uint8_t*)print_buffer, length);

  if (locked) xSemaphoreGive(print_mutex);
  return written;
#endif  // #ifdef NDEBUG
}

void set_cursor(int row, int col) { print("\033[%d;%dH", row, col); }

void clear_screen(void) { print("\033[2J"); }
