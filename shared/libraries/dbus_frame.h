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
 * @file dbus_frame.h
 * @brief Framing rules for one DBUS burst (the bytes between two UART idle events).
 *
 * A DR16 frame is 18 bytes, 8E1 = 11 bits per byte, so at 100 kbaud one frame
 * takes 198 bit times: 100000 / 198 = 505 frames/s is the theoretical maximum
 * when frames are sent back to back without an idle gap.
 *
 * The idle line interrupt normally fires once per frame. When the receiver is
 * late (a longer critical section, a higher priority interrupt) two frames can
 * end up in one burst. Either way the burst is only usable when its length is an
 * exact multiple of the frame size: a ragged burst means the burst boundary is
 * polluted (parity error, dropped byte, late read) and its frames cannot be
 * trusted, because a single dropped byte shifts every following bit field.
 *
 * This header deliberately has no HAL dependency so the rule can be exercised by
 * the host regression test in tests/dbus_input.
 */

#pragma once

#include <cstdint>

namespace remote {

constexpr uint32_t kDbusFrameBytes = 18;

/* Largest number of frames accepted inside one burst. */
constexpr uint32_t kDbusMaxBurstFrames = 2;

/* RX buffer size for one burst: the accepted burst plus one byte, so a burst that
 * fills the buffer completely can be recognised as truncated instead of being
 * decoded as a valid frame. */
constexpr uint32_t kDbusBurstBufferBytes = kDbusMaxBurstFrames * kDbusFrameBytes + 1;

/** @brief Verdict for one burst. */
enum class DbusBurstStatus {
  kEmpty,     /* no bytes: idle interrupt without data, or a discarded burst */
  kAccepted,  /* complete, aligned frames inside the accepted burst size */
  kTooShort,  /* fewer bytes than one frame */
  kRagged,    /* length is not a multiple of the frame size -> torn burst */
  kTruncated, /* the RX buffer filled up: the burst is longer than the buffer */
};

/** @brief One burst split into whole frames. */
struct DbusBurst {
  const uint8_t* data;  /* burst start, nullptr when nothing was read */
  uint32_t length;      /* bytes in the burst, 0 when nothing was read */
  uint32_t frame_count; /* whole frames, 0 unless status is kAccepted */
  DbusBurstStatus status;
};

/**
 * @brief Split one burst into whole frames and report whether it is usable.
 *
 * @param data      burst start as returned by bsp::UART::Read
 * @param length    byte count returned by bsp::UART::Read (negative on failure)
 * @param capacity  capacity of the RX buffer that produced the burst
 * @return burst description; only kAccepted carries usable frames
 */
inline DbusBurst ParseDbusBurst(const uint8_t* data, int32_t length, uint32_t capacity) {
  DbusBurst burst = {data, 0u, 0u, DbusBurstStatus::kEmpty};
  if (length <= 0 || data == nullptr) return burst;

  burst.length = static_cast<uint32_t>(length);
  if (burst.length >= capacity) {
    burst.status = DbusBurstStatus::kTruncated;
    return burst;
  }
  if (burst.length < kDbusFrameBytes) {
    burst.status = DbusBurstStatus::kTooShort;
    return burst;
  }
  if ((burst.length % kDbusFrameBytes) != 0u) {
    burst.status = DbusBurstStatus::kRagged;
    return burst;
  }

  burst.frame_count = burst.length / kDbusFrameBytes;
  if (burst.frame_count > kDbusMaxBurstFrames) {
    /* Cannot happen while the buffer is sized from kDbusMaxBurstFrames, kept as a
     * guard so a larger buffer cannot silently hand out more frames than the
     * consumers account for. */
    burst.frame_count = 0u;
    burst.status = DbusBurstStatus::kTruncated;
    return burst;
  }

  burst.status = DbusBurstStatus::kAccepted;
  return burst;
}

}  // namespace remote
