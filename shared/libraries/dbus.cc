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

#include "dbus.h"

#include <cmath>
#include <cstring>

#include "bsp_error_handler.h"
#include "cmsis_os.h"
#include "task.h"

/* rocker range and dead-zones */
#define RC_ROCKER_MID 1024
#define RC_ROCKER_ZERO_DRIFT 10  // Range of possible drift around initial position
// Range of possible drift around min or max position
#define RC_ROCKER_MIN_MAX_DRIFT (RC_ROCKER_MAX - RC_ROCKER_MID + 10)

namespace remote {

// helper struct for decoding raw bytes
typedef struct {
  /* rocker channel information */
  uint16_t ch0 : 11;  // S1*             *S2
  uint16_t ch1 : 11;  //   C3-^       ^-C1
  uint16_t ch2 : 11;  // C2-<   >+ -<   >+C0
  uint16_t ch3 : 11;  //     +v       v+

  /* left and right switch information */
  uint8_t swr : 2;
  uint8_t swl : 2;
  /* mouse movement and button information */
  remote::mouse_t mouse;

  /* keyboard key information */
  remote::keyboard_t keyboard;
  remote::wheel_t wheel;  // Analog range: Mid(steady state): 1024, Upmax: around 6960 Downmax: around 0
                          // Digital range: Down: 1684 (below analog signal range)
} __packed dbus_t;

DBUS::DBUS(UART_HandleTypeDef* huart) : bsp::UART(huart) { SetupRx(kDbusBurstBufferBytes); }

void DBUS::RxCompleteCallback() {
  uint8_t* data = nullptr;
  /* also re-arms the RX DMA and reports -1 when the burst was polluted by a UART
   * error; such a burst has neither a trustworthy length nor trustworthy content */
  const int32_t length = this->Read<true>(&data);
  if (length < 0) {
    ++rejected_bursts_;
    return;
  }

  const uint32_t now_ms = HAL_GetTick();
  const DbusBurst burst = ParseDbusBurst(data, length, rx_size_);

  if (burst.status == DbusBurstStatus::kEmpty) return; /* idle interrupt without data */
  if (burst.status != DbusBurstStatus::kAccepted) {
    /* A torn or truncated burst shifts every following bit field, so it neither
     * updates the state nor reports a connection: the previous frame stays the
     * newest valid sample. */
    ++rejected_bursts_;
    return;
  }

  connection_flag_ = true;

  for (uint32_t i = 0; i < burst.frame_count; ++i) {
    /* re-interpret the data buffer and decode into class properties */
    const dbus_t* repr =
        reinterpret_cast<const dbus_t*>(burst.data + i * static_cast<uint32_t>(kDbusFrameBytes));

    /* every frame carries a displacement, so all of them are accumulated; a poll
     * between two bursts must not lose the frames it did not see */
    mouse_delta_.Accumulate(repr->mouse.x, repr->mouse.y, now_ms);

    this->ch0 = repr->ch0 - RC_ROCKER_MID;
    this->ch1 = repr->ch1 - RC_ROCKER_MID;
    this->ch2 = repr->ch2 - RC_ROCKER_MID;
    this->ch3 = repr->ch3 - RC_ROCKER_MID;
    this->ch0 = abs(this->ch0) <= RC_ROCKER_ZERO_DRIFT ? 0 : this->ch0;
    this->ch1 = abs(this->ch1) <= RC_ROCKER_ZERO_DRIFT ? 0 : this->ch1;
    this->ch2 = abs(this->ch2) <= RC_ROCKER_ZERO_DRIFT ? 0 : this->ch2;
    this->ch3 = abs(this->ch3) <= RC_ROCKER_ZERO_DRIFT ? 0 : this->ch3;

    this->swl = static_cast<switch_t>(repr->swl);
    this->swr = static_cast<switch_t>(repr->swr);

    memcpy(&this->mouse, &repr->mouse, sizeof(mouse_t));
    memcpy(&this->keyboard, &repr->keyboard, sizeof(keyboard_t));
    memcpy(&this->wheel, &repr->wheel, sizeof(wheel_t));

    this->timestamp = now_ms;
  }
}

bool DBUS::TakeMouseDelta(mouse_t* delta) {
  int16_t dx = 0;
  int16_t dy = 0;
  bool fresh;

  /* the receive callback runs in the UART interrupt, the consumer in a task: the
   * take and clear has to be atomic with respect to the accumulation */
  taskENTER_CRITICAL();
  fresh = mouse_delta_.Take(&dx, &dy, HAL_GetTick());
  taskEXIT_CRITICAL();

  if (delta) {
    /* only x / y carry per-frame displacement; z, l and r are per-frame state */
    delta->x = dx;
    delta->y = dy;
    delta->z = 0;
    delta->l = 0;
    delta->r = 0;
  }
  return fresh;
}

void DBUS::DiscardMouseDelta() {
  taskENTER_CRITICAL();
  mouse_delta_.Discard();
  taskEXIT_CRITICAL();
}

} /* namespace remote */
