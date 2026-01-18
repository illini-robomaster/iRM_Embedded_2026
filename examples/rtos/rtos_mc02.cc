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
 * @file rtos_mc02.cc
 * @brief FreeRTOS Multi-Thread Example for DM_MC_02
 *
 * This example demonstrates:
 *   1. Creating multiple threads with osThreadNew
 *   2. Thread communication using Event Flags
 *   3. Thread communication using Message Queue
 *   4. Mutex for shared resource protection
 *   5. Different thread priorities
 *
 * Hardware:
 *   - Buzzer (PB15, TIM12_CH2): Audio feedback
 *   - USB: Debug output
 */

#include "main.h"
#include "tim.h"

#include <cstdarg>
#include <cstdio>

#include "bsp_print.h"
#include "bsp_buzzer.h"
#include "cmsis_os.h"

// ========================= Hardware Definitions =========================
#define BUZZER_CLOCK_FREQ (80000000 / 24)

// ========================= RTOS Objects =========================
// Event flags for inter-thread signaling
static osEventFlagsId_t event_flags;
#define EVENT_TASK2_TRIGGER (1 << 0)
#define EVENT_TASK3_TRIGGER (1 << 1)

// Message queue for data passing
typedef struct {
  uint32_t counter;
  uint32_t timestamp;
} Message_t;

static osMessageQueueId_t message_queue;

// Mutex for protecting shared resource (print)
static osMutexId_t print_mutex;

// ========================= Hardware Objects =========================
static bsp::Buzzer* buzzer = nullptr;

// ========================= Thread Handles =========================
static osThreadId_t task1_handle;
static osThreadId_t task2_handle;
static osThreadId_t task3_handle;

// ========================= Thread Attributes =========================
const osThreadAttr_t task1_attr = {
    .name = "Task1_Counter",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 256 * 4,
    .priority = osPriorityNormal,
    .tz_module = 0,
    .reserved = 0
};

const osThreadAttr_t task2_attr = {
    .name = "Task2_EventHandler",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 256 * 4,
    .priority = osPriorityAboveNormal,  // Higher priority
    .tz_module = 0,
    .reserved = 0
};

const osThreadAttr_t task3_attr = {
    .name = "Task3_Buzzer",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 256 * 4,
    .priority = osPriorityBelowNormal,  // Lower priority
    .tz_module = 0,
    .reserved = 0
};

// ========================= Thread-safe print =========================
static void safe_print(const char* format, ...) {
  osMutexAcquire(print_mutex, osWaitForever);
  
  va_list args;
  va_start(args, format);
  char buffer[128];
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  print("%s", buffer);
  
  osMutexRelease(print_mutex);
}

// ========================= Task 1: Counter & Producer =========================
/**
 * Task 1: Periodic counter that:
 *   - Prints status every 500ms
 *   - Sends message to queue every 2 seconds
 *   - Triggers Task 2 via event flag every 5 counts
 */
void Task1_Counter(void* argument) {
  UNUSED(argument);
  
  uint32_t counter = 0;
  Message_t msg;
  
  safe_print("[Task1] Started - Counter & Producer\r\n");
  
  while (true) {
    counter++;
    
    // Print heartbeat
    safe_print("[Task1] Counter: %lu\r\n", counter);
    
    // Send message to queue every 4 cycles (2 seconds)
    if (counter % 4 == 0) {
      msg.counter = counter;
      msg.timestamp = osKernelGetTickCount();
      
      if (osMessageQueuePut(message_queue, &msg, 0, 0) == osOK) {
        safe_print("[Task1] Sent message to queue: counter=%lu\r\n", msg.counter);
      }
    }
    
    // Trigger Task 2 every 10 cycles (5 seconds)
    if (counter % 10 == 0) {
      safe_print("[Task1] >>> Triggering Task2 via event flag\r\n");
      osEventFlagsSet(event_flags, EVENT_TASK2_TRIGGER);
    }
    
    // Trigger Task 3 (buzzer) every 20 cycles (10 seconds)
    if (counter % 20 == 0) {
      safe_print("[Task1] >>> Triggering Task3 (buzzer)\r\n");
      osEventFlagsSet(event_flags, EVENT_TASK3_TRIGGER);
    }
    
    osDelay(500);  // 500ms period
  }
}

// ========================= Task 2: Event Handler & Consumer =========================
/**
 * Task 2: Event-driven task that:
 *   - Waits for event flag from Task 1
 *   - Reads messages from queue
 */
void Task2_EventHandler(void* argument) {
  UNUSED(argument);
  
  Message_t msg;
  uint32_t flags;
  
  safe_print("[Task2] Started - Event Handler & Consumer\r\n");
  
  while (true) {
    // Wait for event flag (with timeout for queue check)
    flags = osEventFlagsWait(event_flags, EVENT_TASK2_TRIGGER, osFlagsWaitAny, 1000);
    
    if ((flags != osFlagsErrorTimeout) && (flags & EVENT_TASK2_TRIGGER)) {
      safe_print("[Task2] <<< Event received!\r\n");
      
      // Short beep to indicate event
      buzzer->SingTone(bsp::BuzzerNote::La6M);
      osDelay(50);
      buzzer->SingTone(bsp::BuzzerNote::Silent);
    }
    
    // Check message queue (non-blocking)
    while (osMessageQueueGet(message_queue, &msg, nullptr, 0) == osOK) {
      uint32_t latency = osKernelGetTickCount() - msg.timestamp;
      safe_print("[Task2] Received from queue: counter=%lu, latency=%lu ms\r\n",
                 msg.counter, latency);
    }
  }
}

// ========================= Task 3: Buzzer =========================
/**
 * Task 3: Low priority buzzer task that:
 *   - Waits for event flag
 *   - Plays a short melody
 */
void Task3_Buzzer(void* argument) {
  UNUSED(argument);
  
  uint32_t flags;
  
  safe_print("[Task3] Started - Buzzer\r\n");
  
  // Startup beep
  buzzer->SingTone(bsp::BuzzerNote::Do1H);
  osDelay(100);
  buzzer->SingTone(bsp::BuzzerNote::Silent);
  
  while (true) {
    // Wait for buzzer event
    flags = osEventFlagsWait(event_flags, EVENT_TASK3_TRIGGER, osFlagsWaitAny, osWaitForever);
    
    if (flags & EVENT_TASK3_TRIGGER) {
      safe_print("[Task3] <<< Playing melody\r\n");
      
      // Play ascending melody
      buzzer->SingTone(bsp::BuzzerNote::Do1M);
      osDelay(100);
      buzzer->SingTone(bsp::BuzzerNote::Mi3M);
      osDelay(100);
      buzzer->SingTone(bsp::BuzzerNote::So5M);
      osDelay(100);
      buzzer->SingTone(bsp::BuzzerNote::Do1H);
      osDelay(200);
      buzzer->SingTone(bsp::BuzzerNote::Silent);
    }
  }
}

// ========================= RTOS Initialization =========================
void RM_RTOS_Init(void) {
  print_use_usb();
  
  // Initialize hardware
  buzzer = new bsp::Buzzer(&htim12, 2, BUZZER_CLOCK_FREQ);
  
  // Create RTOS objects
  event_flags = osEventFlagsNew(nullptr);
  message_queue = osMessageQueueNew(8, sizeof(Message_t), nullptr);
  print_mutex = osMutexNew(nullptr);
}

void RM_RTOS_Threads_Init(void) {
  // Create all threads
  task1_handle = osThreadNew(Task1_Counter, nullptr, &task1_attr);
  task2_handle = osThreadNew(Task2_EventHandler, nullptr, &task2_attr);
  task3_handle = osThreadNew(Task3_Buzzer, nullptr, &task3_attr);
}

void RM_RTOS_Default_Task(const void* argument) {
  UNUSED(argument);
  
  // Wait a bit for USB to connect
  osDelay(1000);
  
  safe_print("\r\n");
  safe_print("========================================\r\n");
  safe_print("  DM_MC_02 FreeRTOS Multi-Thread Demo\r\n");
  safe_print("========================================\r\n");
  safe_print("\r\n");
  safe_print("Tasks:\r\n");
  safe_print("  Task1: Counter (Normal priority)\r\n");
  safe_print("  Task2: Event Handler (Above Normal)\r\n");
  safe_print("  Task3: Buzzer (Below Normal)\r\n");
  safe_print("\r\n");
  safe_print("Schedule:\r\n");
  safe_print("  - Task1 prints every 0.5s\r\n");
  safe_print("  - Task1 sends to queue every 2s\r\n");
  safe_print("  - Task1 triggers Task2 every 5s\r\n");
  safe_print("  - Task1 triggers Task3 every 10s\r\n");
  safe_print("\r\n");
  
  // Default task monitors system uptime
  uint32_t tick = 0;
  while (true) {
    osDelay(30000);  // Print status every 30 seconds
    tick += 30;
    safe_print("[Main] === Uptime: %lu seconds ===\r\n", tick);
  }
}
