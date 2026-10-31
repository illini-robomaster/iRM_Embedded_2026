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
 * @file mouse_gimbal_step.h
 * @brief Omni gimbal reaction to one control iteration worth of mouse movement.
 *
 * DBUS mouse x/y are per-frame displacement counts, so they are integrated as
 * angle increments (no dt) and still pass through the existing soft limits:
 * +/-45 deg on pitch and the YAW_MAX_LEAD clamp on yaw.
 *
 * The displacement is consumed from the receiver before this function runs, so a
 * step that is not applied here is gone for good. That is intended: movement that
 * arrived while the outputs were disabled, or while autoaim owned a locked
 * gimbal, must not be replayed when the gimbal starts listening again.
 *
 * This header has no HAL dependency so the policy can be exercised by the host
 * regression test in tests/dbus_input.
 */

#pragma once

#include <cstdint>

namespace omni {

constexpr float kMouseYawGain = 0.0005f;   /* rad per count, left/right */
constexpr float kMousePitchGain = 0.0015f; /* rad per count, up/down (increased for responsive pitch) */
constexpr float kMouseStepLimit = 0.15f;   /* rad, per-loop anti-jump clamp */

/** @brief Gimbal angle increments for one control iteration. */
struct MouseGimbalStep {
  float yaw;
  float pitch;
};

/**
 * @brief Convert the mouse displacement of one iteration into gimbal steps.
 *
 * @param dx              consumed x displacement (counts), sign as the DR16 reports it
 * @param dy              consumed y displacement (counts), sign as the DR16 reports it
 * @param outputs_enabled false while the remote keeps the outputs disabled
 * @param mouse_active    false while autoaim owns a locked gimbal
 * @return angle increments in rad, zero when the movement must not be applied
 */
inline MouseGimbalStep ComputeMouseGimbalStep(int16_t dx, int16_t dy, bool outputs_enabled,
                                              bool mouse_active) {
  MouseGimbalStep step = {0.0f, 0.0f};
  if (!outputs_enabled || !mouse_active) return step;

  const float yaw = -static_cast<float>(dx) * kMouseYawGain;
  const float pitch = -static_cast<float>(dy) * kMousePitchGain;
  step.yaw = yaw > kMouseStepLimit ? kMouseStepLimit : (yaw < -kMouseStepLimit ? -kMouseStepLimit : yaw);
  step.pitch =
      pitch > kMouseStepLimit ? kMouseStepLimit : (pitch < -kMouseStepLimit ? -kMouseStepLimit : pitch);
  return step;
}

}  // namespace omni
