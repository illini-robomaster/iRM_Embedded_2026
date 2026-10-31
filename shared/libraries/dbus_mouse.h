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
 * @file dbus_mouse.h
 * @brief Accumulation of DBUS mouse displacement between two control-loop polls.
 *
 * A DBUS frame carries the mouse displacement since the previous frame, not a
 * position. A control loop that reads the newest frame on every iteration
 * therefore integrates the same displacement once per iteration (too much
 * movement while the frame rate is below the loop rate), silently drops every
 * frame that arrives between two iterations (too little movement), and keeps
 * integrating the last non-zero displacement forever once the link is lost.
 *
 * The receiver side accumulates displacement (interrupt context, single writer)
 * and the control side takes and clears it inside a critical section, so every
 * received frame is applied exactly once and no displacement is lost between
 * polls.
 *
 * This header deliberately has no HAL dependency so the rule can be exercised by
 * the host regression test in tests/dbus_input; the caller owns the locking.
 */

#pragma once

#include <cstdint>

namespace remote {

/* Pending displacement older than this is dropped instead of applied: a delta
 * that waited longer than this arrived while the robot was not listening (link
 * loss, disabled outputs) and replaying it would move the gimbal without a
 * matching operator action. */
constexpr uint32_t kMouseDeltaStaleMs = 100;

/* Per-axis saturation. The DR16 reports int16 counts per frame, so a sum that
 * leaves the int16 range means many frames piled up; saturating keeps the unit
 * consistent instead of wrapping the displacement around. */
constexpr int32_t kMouseDeltaAxisLimit = 32767;

/**
 * @brief Accumulates DBUS mouse displacement between two control-loop polls.
 *
 * @note Not internally synchronized. The writer must be the only interrupt-side
 *       caller and the reader must run inside a critical section, which is how
 *       remote::DBUS drives it.
 */
class MouseDeltaAccumulator {
 public:
  /**
   * @brief Add the displacement of one received frame.
   * @param dx      displacement counts, x axis
   * @param dy      displacement counts, y axis
   * @param now_ms  reception timestamp
   */
  void Accumulate(int16_t dx, int16_t dy, uint32_t now_ms) {
    pending_x_ = Saturate(pending_x_ + dx);
    pending_y_ = Saturate(pending_y_ + dy);
    last_ms_ = now_ms;
    pending_ = true;
  }

  /**
   * @brief Take the accumulated displacement and clear it.
   *
   * The pending displacement is cleared in every case, so a stale delta is
   * dropped rather than kept for the next poll.
   *
   * @param dx,dy  accumulated displacement since the previous take, zeroed when
   *               nothing is pending or the accumulation went stale
   * @param now_ms current tick
   * @return true when fresh displacement was available
   */
  bool Take(int16_t* dx, int16_t* dy, uint32_t now_ms) {
    const bool available = pending_ && (now_ms - last_ms_) <= kMouseDeltaStaleMs;
    if (dx) *dx = available ? static_cast<int16_t>(pending_x_) : 0;
    if (dy) *dy = available ? static_cast<int16_t>(pending_y_) : 0;
    pending_ = false;
    pending_x_ = 0;
    pending_y_ = 0;
    return available;
  }

  /** @brief Drop the pending displacement (outputs disabled, mode switch). */
  void Discard() {
    pending_ = false;
    pending_x_ = 0;
    pending_y_ = 0;
  }

  /** @brief True while displacement is waiting to be taken. */
  bool HasPending() const { return pending_; }

  /** @brief Number of axes whose accumulated displacement had to be saturated. */
  uint32_t overflow_count() const { return overflow_count_; }

 private:
  int32_t Saturate(int32_t value) {
    if (value > kMouseDeltaAxisLimit) {
      ++overflow_count_;
      return kMouseDeltaAxisLimit;
    }
    if (value < -kMouseDeltaAxisLimit) {
      ++overflow_count_;
      return -kMouseDeltaAxisLimit;
    }
    return value;
  }

  int32_t pending_x_ = 0;
  int32_t pending_y_ = 0;
  uint32_t last_ms_ = 0;
  bool pending_ = false;
  uint32_t overflow_count_ = 0;
};

}  // namespace remote
