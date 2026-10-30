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
 * @file dbus_keymouse_mc02.cc
 * @brief Standalone DR16 / DBUS keyboard and mouse diagnostic for DM_MC_02.
 *
 * Signal chain:
 *   PC keyboard/mouse -> RoboMaster Client -> USB -> DT7 -> (2.4 GHz) -> DR16
 *   -> MC02 UART5 (DBUS, 100000 8E1) -> this monitor.
 *
 * What it does:
 * - Decodes every DBUS frame and prints WASD, Shift, mouse dx/dy/dz and the
 *   left/right mouse buttons.
 * - Prints the number of valid frames, the age of the last valid frame, the
 *   observed frame rate and the link state, throttled to 10 Hz.
 * - Judges the link only from frames that pass real validation (frame length,
 *   UART error flags, all-zero frame, stick range, switch range) and from frame
 *   freshness. `remote::DBUS::connection_flag_` is NOT used here because the
 *   shared driver sets it on any idle interrupt, even for a torn frame.
 *
 * What it does NOT do:
 * - No chassis / gimbal / feeder / shooter output, no CAN, no IMU, no autoaim.
 *   Nothing is enabled, so the robot cannot move while this target runs.
 *
 * @note `print()` compiles to a no-op when NDEBUG is defined, so build this
 *       target with CMAKE_BUILD_TYPE=Debug (or RelWithDebInfo) to see output.
 * @note The keyboard and mouse fields of a DBUS frame are only filled while the
 *       RoboMaster Client is connected and syncing to the DT7. Sticks keep
 *       working without it, so "sticks alive but WASD/mouse stuck at 0" means
 *       the client link, not the DBUS wiring.
 */

#include <cstddef>

#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "main.h"
#include "task.h"

namespace {

/* ---------------------------------------------------------------- protocol */

/* A DBUS burst is always 18 bytes: 4 x 11 bit sticks + 2 x 2 bit switches,
 * 3 x 16 bit mouse axes + 2 x 8 bit mouse buttons, 16 bit keyboard, 11 bit
 * wheel (padded to 16 bit). */
constexpr uint32_t kFrameBytes = 18;
constexpr uint32_t kChannelCount = 4;

/* DR16 raw stick range from the DJI specification (mid position is 1024). */
constexpr uint16_t kRockerRawMin = 364;
constexpr uint16_t kRockerRawMax = 1684;

/* UART5 is hard-configured to these values by CubeMX (100000 8E1). */
constexpr uint32_t kDbusBaudRate = 100000;

/* Log throttle and link supervision. */
constexpr uint32_t kReportPeriodMs = 100; /* 10 Hz */
constexpr uint32_t kLinkTimeoutMs = 100;  /* 100 ms without a valid frame -> lost */
constexpr uint32_t kTaskTickMs = 10;

/**
 * @brief DBUS frame layout, identical to the one used by shared/libraries/dbus.cc.
 *
 * The mouse / keyboard / wheel payloads reuse the public typedefs of dbus.h so
 * the key bitmap is defined in exactly one place.
 */
typedef struct {
  uint16_t ch0 : 11;
  uint16_t ch1 : 11;
  uint16_t ch2 : 11;
  uint16_t ch3 : 11;
  uint8_t swr : 2;
  uint8_t swl : 2;
  remote::mouse_t mouse;
  remote::keyboard_t keyboard;
  remote::wheel_t wheel;
} __packed dbus_frame_t;

/* Compile-time proof of the layout: a build error here means the frame format
 * no longer matches the DR16 payload and every decoded value would be garbage. */
static_assert(sizeof(dbus_frame_t) == static_cast<size_t>(kFrameBytes),
              "DBUS frame must be 18 bytes");
static_assert(offsetof(dbus_frame_t, mouse) == static_cast<size_t>(6),
              "mouse payload must start at byte 6");
static_assert(offsetof(dbus_frame_t, keyboard) == static_cast<size_t>(14),
              "keyboard payload must start at byte 14");
static_assert(offsetof(dbus_frame_t, wheel) == static_cast<size_t>(16),
              "wheel payload must start at byte 16");

/** @brief Link classification reported by this monitor. */
enum class LinkState {
  kNoSignal,      /* no UART activity at all inside the supervision window */
  kInvalidFrames, /* bytes are arriving but none of them is a usable frame */
  kConnected,     /* at least one valid frame inside the supervision window */
};

const char* StateName(LinkState state) {
  switch (state) {
    case LinkState::kConnected:
      return "CONNECTED";
    case LinkState::kInvalidFrames:
      return "INVALID_FRAMES";
    default:
      return "NO_SIGNAL";
  }
}

const char* SwitchName(uint8_t value) {
  switch (value) {
    case 1:
      return "UP";
    case 2:
      return "DOWN";
    case 3:
      return "MID";
    default:
      return "??";
  }
}

/* ----------------------------------------------------------------- monitor */

/**
 * @brief DBUS receiver that validates raw frames before trusting them.
 *
 * Decoding runs in the UART idle-line interrupt, the same context used by
 * remote::DBUS. The main task only takes snapshots, so nothing is printed from
 * interrupt context and the interrupt stays short.
 */
class DBUSKeyMouseMonitor : public bsp::UART {
 public:
  explicit DBUSKeyMouseMonitor(UART_HandleTypeDef* huart)
      : UART(huart) {
    SetupRx(kFrameBytes + 1);
  }

  /** @brief Consistent copy of the monitor state, taken in a critical section. */
  struct Snapshot {
    LinkState state;
    uint32_t rx_events;
    uint32_t frames;
    uint32_t valid_frames;
    uint32_t short_frames;
    uint32_t partial_frames;
    uint32_t invalid_frames;
    uint32_t zero_frames;
    uint32_t uart_errors;
    int32_t age_ms;      /* time since the last valid frame, -1 if none yet */
    int32_t interval_ms; /* gap between the last two valid frames */
    uint16_t ch[kChannelCount];
    remote::mouse_t mouse;
    remote::keyboard_t keyboard;
    uint8_t swl;
    uint8_t swr;
    uint16_t wheel;
  };

  Snapshot TakeSnapshot(uint32_t now_ms);

 protected:
  void RxCompleteCallback() override;

 private:
  static bool IsPlausible(const dbus_frame_t& frame, bool* all_zero);
  void StoreFrame(const dbus_frame_t& frame, uint32_t now_ms);

  volatile uint32_t rx_events_ = 0;
  volatile uint32_t frames_ = 0;
  volatile uint32_t valid_frames_ = 0;
  volatile uint32_t short_frames_ = 0;
  volatile uint32_t partial_frames_ = 0;
  volatile uint32_t invalid_frames_ = 0;
  volatile uint32_t zero_frames_ = 0;
  volatile uint32_t uart_errors_ = 0;
  volatile uint32_t last_rx_ms_ = 0;
  volatile uint32_t last_valid_ms_ = 0;
  volatile int32_t interval_ms_ = -1;
  volatile bool have_valid_frame_ = false;
  volatile uint16_t ch_[kChannelCount] = {0, 0, 0, 0};
  remote::mouse_t mouse_ = {};
  remote::keyboard_t keyboard_ = {};
  remote::wheel_t wheel_ = {};
  volatile uint8_t swl_ = 0;
  volatile uint8_t swr_ = 0;
};

/**
 * @brief Structural and semantic check of one decoded frame.
 *
 * A frame is only trusted when every stick is inside the calibrated DR16 range,
 * the payload is not the all-zero pattern the receiver keeps sending while the
 * transmitter is off or unpaired, and both switches are inside 1..3.
 *
 * @param frame    candidate frame
 * @param all_zero set to true when all four sticks are zero
 * @return true if the frame can be used as a control sample
 */
bool DBUSKeyMouseMonitor::IsPlausible(const dbus_frame_t& frame, bool* all_zero) {
  const uint16_t channels[kChannelCount] = {frame.ch0, frame.ch1, frame.ch2, frame.ch3};
  bool zero = true;
  bool in_range = true;
  for (uint32_t i = 0; i < kChannelCount; ++i) {
    if (channels[i] != 0) zero = false;
    if (channels[i] < kRockerRawMin || channels[i] > kRockerRawMax) in_range = false;
  }
  *all_zero = zero;
  if (zero) return false;
  if (!in_range) return false;
  if (frame.swl < 1 || frame.swl > 3) return false;
  if (frame.swr < 1 || frame.swr > 3) return false;
  return true;
}

/** @brief Publish a validated frame as the newest control sample. */
void DBUSKeyMouseMonitor::StoreFrame(const dbus_frame_t& frame, uint32_t now_ms) {
  ch_[0] = frame.ch0;
  ch_[1] = frame.ch1;
  ch_[2] = frame.ch2;
  ch_[3] = frame.ch3;
  mouse_ = frame.mouse;
  keyboard_ = frame.keyboard;
  wheel_ = frame.wheel;
  swl_ = frame.swl;
  swr_ = frame.swr;
  interval_ms_ = have_valid_frame_ ? static_cast<int32_t>(now_ms - last_valid_ms_) : -1;
  have_valid_frame_ = true;
  last_valid_ms_ = now_ms;
  ++valid_frames_;
}

/**
 * @brief Idle-line interrupt entry point: read the burst and validate it.
 *
 * Rejection reasons are counted separately so the log tells wiring problems
 * (no bytes / UART errors) apart from protocol problems (torn frame, remote
 * powered off, sticks out of calibration).
 */
void DBUSKeyMouseMonitor::RxCompleteCallback() {
  uint8_t* data = nullptr;
  const int32_t length = Read<true>(&data); /* also re-arms the RX DMA */
  const uint32_t now_ms = HAL_GetTick();

  ++rx_events_;
  last_rx_ms_ = now_ms;

  /* Parity / framing / noise / overrun: this burst is not trustworthy even if
   * its length happens to match. */
  if ((huart_->Instance->ISR & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) != 0u) {
    ++uart_errors_;
    return;
  }

  if (length <= 0) return; /* idle interrupt without data */
  if (length < static_cast<int32_t>(kFrameBytes)) {
    ++short_frames_;
    return;
  }

  const uint32_t complete_frames = static_cast<uint32_t>(length) / kFrameBytes;
  if ((static_cast<uint32_t>(length) % kFrameBytes) != 0u) ++partial_frames_;
  frames_ += complete_frames;

  /* Use the newest complete frame of the burst: two frames back to back are
   * possible when the idle gap between them is too short to trigger the ISR. */
  const dbus_frame_t* frame =
      reinterpret_cast<const dbus_frame_t*>(data + (complete_frames - 1u) * kFrameBytes);

  bool all_zero = false;
  if (!IsPlausible(*frame, &all_zero)) {
    if (all_zero) {
      ++zero_frames_;
    } else {
      ++invalid_frames_;
    }
    return;
  }
  StoreFrame(*frame, now_ms);
}

/** @brief Copy the interrupt-side state so the task can print it. */
DBUSKeyMouseMonitor::Snapshot DBUSKeyMouseMonitor::TakeSnapshot(uint32_t now_ms) {
  Snapshot snapshot = {};
  uint32_t last_rx_ms = 0;

  taskENTER_CRITICAL();
  snapshot.rx_events = rx_events_;
  snapshot.frames = frames_;
  snapshot.valid_frames = valid_frames_;
  snapshot.short_frames = short_frames_;
  snapshot.partial_frames = partial_frames_;
  snapshot.invalid_frames = invalid_frames_;
  snapshot.zero_frames = zero_frames_;
  snapshot.uart_errors = uart_errors_;
  snapshot.interval_ms = interval_ms_;
  for (uint32_t i = 0; i < kChannelCount; ++i) snapshot.ch[i] = ch_[i];
  snapshot.mouse = mouse_;
  snapshot.keyboard = keyboard_;
  snapshot.wheel = wheel_.wheel;
  snapshot.swl = swl_;
  snapshot.swr = swr_;
  snapshot.age_ms = have_valid_frame_ ? static_cast<int32_t>(now_ms - last_valid_ms_) : -1;
  last_rx_ms = last_rx_ms_;
  taskEXIT_CRITICAL();

  /* Freshness decides the link state, never the bare presence of interrupts. */
  const bool recent_valid =
      snapshot.age_ms >= 0 && snapshot.age_ms <= static_cast<int32_t>(kLinkTimeoutMs);
  const bool recent_rx = last_rx_ms != 0u && (now_ms - last_rx_ms) <= kLinkTimeoutMs;
  if (recent_valid) {
    snapshot.state = LinkState::kConnected;
  } else if (recent_rx) {
    snapshot.state = LinkState::kInvalidFrames;
  } else {
    snapshot.state = LinkState::kNoSignal;
  }
  return snapshot;
}

/* -------------------------------------------------------------- app wiring */

DBUSKeyMouseMonitor* monitor = nullptr;

/**
 * @brief One-shot check of the CubeMX UART5 configuration.
 *
 * A wrong baud rate, word length, parity or an unarmed RX DMA explains "no data
 * at all" much more often than the remote does, so it is reported at boot.
 */
void PrintDbusUartConfig() {
  UART_HandleTypeDef* huart = &huart5;
  const bool config_ok = huart->Init.BaudRate == kDbusBaudRate &&
                         huart->Init.WordLength == UART_WORDLENGTH_9B &&
                         huart->Init.Parity == UART_PARITY_EVEN &&
                         huart->Init.StopBits == UART_STOPBITS_1 &&
                         huart->Init.Mode == UART_MODE_TX_RX;
  const bool dma_ok = huart->hdmarx != nullptr;
  print("[DBUS] UART5 cfg: baud=%u word_len=%u parity=%u stop=%u mode=%u dma=%u rx_state=%u -> %s\r\n",
        static_cast<unsigned>(huart->Init.BaudRate), static_cast<unsigned>(huart->Init.WordLength),
        static_cast<unsigned>(huart->Init.Parity), static_cast<unsigned>(huart->Init.StopBits),
        static_cast<unsigned>(huart->Init.Mode), dma_ok ? 1u : 0u,
        static_cast<unsigned>(huart->RxState), (config_ok && dma_ok) ? "OK" : "MISMATCH");
}

}  // namespace

void RM_RTOS_Init() {
  print_use_uart(&huart7); /* PE7/PE8, 115200 8N1, same debug port as eoh_infant */
  monitor = new DBUSKeyMouseMonitor(&huart5);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  print("\r\n[DBUS] keyboard/mouse diagnostic for DM_MC_02 (read only, no motors)\r\n");
  print("[DBUS] chain: PC -> RoboMaster Client -> USB -> DT7 -> DR16 -> UART5\r\n");
  PrintDbusUartConfig();

  LinkState last_state = LinkState::kNoSignal;
  uint32_t last_report_ms = HAL_GetTick();
  uint32_t last_report_valid = 0;

  while (true) {
    const uint32_t now_ms = HAL_GetTick();
    if (now_ms - last_report_ms >= kReportPeriodMs) {
      const DBUSKeyMouseMonitor::Snapshot s = monitor->TakeSnapshot(now_ms);
      const uint32_t window_ms = now_ms - last_report_ms;
      const uint32_t rate = (s.valid_frames - last_report_valid) * 1000u / window_ms;
      last_report_valid = s.valid_frames;
      last_report_ms = now_ms;

      if (s.state != last_state) {
        print("[DBUS] link -> %s\r\n", StateName(s.state));
        last_state = s.state;
      }

      print("[DBUS] link=%s age=%dms frames=%u valid=%u rate=%u/s short=%u partial=%u bad=%u zero=%u uart_err=%u\r\n",
            StateName(s.state), static_cast<int>(s.age_ms), static_cast<unsigned>(s.frames),
            static_cast<unsigned>(s.valid_frames), static_cast<unsigned>(rate),
            static_cast<unsigned>(s.short_frames), static_cast<unsigned>(s.partial_frames),
            static_cast<unsigned>(s.invalid_frames), static_cast<unsigned>(s.zero_frames),
            static_cast<unsigned>(s.uart_errors));

      print("[DBUS] key W=%d A=%d S=%d D=%d SHIFT=%d CTRL=%d | mouse dx=%d dy=%d dz=%d L=%u R=%u | wheel=%u swl=%s swr=%s ch=%u,%u,%u,%u\r\n",
            static_cast<int>(s.keyboard.bit.W), static_cast<int>(s.keyboard.bit.A),
            static_cast<int>(s.keyboard.bit.S), static_cast<int>(s.keyboard.bit.D),
            static_cast<int>(s.keyboard.bit.SHIFT), static_cast<int>(s.keyboard.bit.CTRL),
            static_cast<int>(s.mouse.x), static_cast<int>(s.mouse.y), static_cast<int>(s.mouse.z),
            static_cast<unsigned>(s.mouse.l), static_cast<unsigned>(s.mouse.r),
            static_cast<unsigned>(s.wheel), SwitchName(s.swl), SwitchName(s.swr),
            static_cast<unsigned>(s.ch[0]), static_cast<unsigned>(s.ch[1]),
            static_cast<unsigned>(s.ch[2]), static_cast<unsigned>(s.ch[3]));
    }
    osDelay(kTaskTickMs);
  }
}
