// Standalone low-current test for DM3519 with V41 DJI-compatible firmware.
#include <cmath>
#include "bsp_fdcan.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "main.h"
#include "usart.h"

namespace {
bsp::FDCAN* bus;
remote::DBUS* rc;
volatile uint32_t feedback_ms = 0;
volatile bool received = false;
volatile int16_t rpm = 0;
volatile int16_t current = 0;
volatile uint8_t status = 0;
volatile uint32_t clear_ack_count = 0;
volatile uint32_t clear_ack_id = 0;
void ClearReply(const uint8_t data[], void* arg) {
  if (data[0] == 1 && data[1] == 0 && data[2] == 0x55 && data[3] == 0x3C) {
    clear_ack_id = reinterpret_cast<uintptr_t>(arg);
    ++clear_ack_count;
  }
}
void Feedback(const uint8_t data[], void*) {
  rpm = static_cast<int16_t>((data[2] << 8) | data[3]);
  current = static_cast<int16_t>((data[4] << 8) | data[5]);
  status = data[7];
  feedback_ms = HAL_GetTick();
  received = true;
}
}

void RM_RTOS_Init() {
  print_use_uart(&huart7);
  bus = new bsp::FDCAN(&hfdcan1, 0);
  bus->RegisterRxCallback(0x201, Feedback, nullptr);
  bus->RegisterRxCallback(0x18, ClearReply, reinterpret_cast<void*>(0x18));
  bus->RegisterRxCallback(0x16, ClearReply, reinterpret_cast<void*>(0x16));
  rc = new remote::DBUS(&huart5);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  const uint8_t zero[8] = {};
  const uint8_t disable[8] = {255,255,255,255,255,255,255,253};
  // Clear existing DJI commands and disable the native DM pitch motor.
  for (int i = 0; i < 3; ++i) {
    bus->Transmit(0x200, zero, 8);
    bus->Transmit(0x1FF, zero, 8);
    bus->Transmit(0x1D, disable, 8);
    bus->Transmit(0x11D, disable, 8);
    osDelay(10);
  }
  bool armed = false;
  bool saw_disabled = false;
  bool clear_attempted = false;
  uint32_t report_ms = 0;
  print("[201 TEST] UP+neutral then MID+neutral to arm; right stick vertical; max +/-1000 counts\r\n");
  while (true) {
    const uint32_t now = HAL_GetTick();
    const bool link = rc->connection_flag_ && now - rc->timestamp <= 100;
    const bool fresh = received && now - feedback_ms <= 100;
    const int stick = rc->ch1;
    const bool neutral = stick >= -30 && stick <= 30;
    if (!link || !fresh) {
      armed = false;
      saw_disabled = false;
    } else if (rc->swl == remote::UP) {
      armed = false;
      saw_disabled = neutral;
    } else if (rc->swl == remote::MID && saw_disabled && neutral) {
      armed = true;
    } else if (rc->swl != remote::MID) {
      armed = false;
      saw_disabled = false;
    }
    int16_t command = 0;
    if (armed && link && fresh && rc->swl == remote::MID && !neutral) {
      const int bounded = stick > 660 ? 660 : (stick < -660 ? -660 : stick);
      command = static_cast<int16_t>(bounded * 1000 / 660);
    }
    // One diagnostic clear only, with explicit remote disable and neutral stick.
    // Official V41 command can also save the present position as zero.
    if (!clear_attempted && link && fresh && rc->swl == remote::UP && neutral) {
      armed = false;
      saw_disabled = false;
      command = 0;
      bus->Transmit(0x200, zero, 8);
      const uint8_t clear_error[4] = {0x01, 0x00, 0x55, 0x3C};
      const int clear_tx = bus->Transmit(0x7FF, clear_error, 4);
      clear_attempted = true;
      print("[201 CLEAR] tx=%d D7_before=0x%02X; inspect subsequent D7\r\n", clear_tx, status);
    }
    uint8_t frame[8] = {};
    frame[0] = static_cast<uint16_t>(command) >> 8;
    frame[1] = static_cast<uint16_t>(command) & 255;
    const int tx = bus->Transmit(0x200, frame, 8);
    bus->Transmit(0x1FF, zero, 8);
    if (now - report_ms >= 200) {
      report_ms = now;
      print("[201 TEST] arm=%d link=%d fresh=%d stick=%d cmd=%d tx=%d rpm=%d fbI=%d D7=0x%02X age=%lums\r\n",
            armed, link, fresh, stick, command, tx, rpm, current, status,
            static_cast<unsigned long>(now - feedback_ms));
      print("[201 ACK] count=%lu id=0x%02lX PSR=0x%08lX TXFQS=0x%08lX\r\n",
            (unsigned long)clear_ack_count, (unsigned long)clear_ack_id,
            (unsigned long)hfdcan1.Instance->PSR, (unsigned long)hfdcan1.Instance->TXFQS);
    }
    osDelay(5);
  }
}
