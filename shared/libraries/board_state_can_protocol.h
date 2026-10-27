/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2026 RoboMaster.                                          *
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

// Jetson <-> CBoard board-state CAN protocol v1.  The authoritative wire
// specification is iRM_Vision_2026/documentation/board_state_can_protocol.md.
//
// This file contains only the pure wire codec and the CBoard-side session state
// machine.  It does not touch HAL, RTOS, or any global hardware state, so the
// same code can be checked independently of the CAN driver.

#pragma once

#include <cstddef>
#include <cstdint>

#include "minipc_protocol.h"

namespace communication {
namespace board_state_can {

// Default standard IDs verified on the Type C CAN1 <-> Jetson can0 test bus.
// Recheck the allocation before using them on a different robot or bus.
constexpr uint16_t JETSON_TO_BOARD_ID = 0x320;      // START and final ACK
constexpr uint16_t BOARD_TO_JETSON_ACK_ID = 0x321;  // ACK + status + color
constexpr uint16_t BOARD_TO_JETSON_DATA_ID = 0x322; // board-state fragments

constexpr uint8_t PROTOCOL_VERSION = 0x01;

constexpr uint8_t HANDSHAKE_DLC = 8;
constexpr uint8_t FRAGMENT_DLC = 8;

constexpr uint8_t FRAGMENT_COUNT = 4;
constexpr uint8_t FRAGMENT_BYTES = 6;
constexpr uint8_t FRAGMENT_INDEX_SHIFT = 5;
constexpr uint8_t SAMPLE_SEQ_MASK = 0x1F;
constexpr uint8_t BOARD_STATE_PAYLOAD_SIZE = 21;

// CBoard repeats ACK+color every 100 ms until it receives the final ACK.
constexpr uint32_t HANDSHAKE_RETRY_MS = 100;
// Nominal board-state rate.  Fragments are paced by the comm task.
constexpr uint32_t TELEMETRY_PERIOD_MS = 10;

// byte 6 of ACK+color
enum class Status : uint8_t {
  READY = 0,
  NOT_READY = 1,
  VERSION_MISMATCH = 2,
};

// byte 7 of ACK+color.  These are protocol values, NOT auto_aim::Color values.
enum class Color : uint8_t {
  UNSPECIFIED = 0,
  RED = 1,
  BLUE = 2,
};

enum class MessageType : uint8_t {
  START = 0x01,
  ACK = 0x02,
  FINAL_ACK = 0x03,
};

// Decoded START.  A version other than PROTOCOL_VERSION is still decoded so the
// board can echo the session in a VERSION_MISMATCH response.
struct StartMessage {
  uint32_t session_id;
  uint8_t version;
};

struct FinalAckMessage {
  uint32_t session_id;
  uint8_t version;
};

/**
 * @brief decode a START frame
 * @return true only for DLC=8, type=START, zero padding in bytes 6..7
 */
bool DecodeStart(const uint8_t data[HANDSHAKE_DLC], uint32_t dlc, StartMessage* message);

/**
 * @brief decode a final ACK frame
 * @return true only for DLC=8, type=FINAL_ACK, zero padding in bytes 6..7
 */
bool DecodeFinalAck(const uint8_t data[HANDSHAKE_DLC], uint32_t dlc, FinalAckMessage* message);

/**
 * @brief encode ACK+color
 *
 * Layout: 02 01 session_BE status color.
 */
void EncodeAck(uint8_t data[HANDSHAKE_DLC], uint32_t session_id, Status status, Color color);

/**
 * @brief encode one board-state fragment
 *
 * Layout: fragment_index<<5 | sample_seq&0x1F, session_id&0xFF, six payload
 * bytes.  Fragment 3 carries three payload bytes and bytes 5..7 are zero padding.
 *
 * @return false for a fragment index outside 0..3
 */
bool EncodeFragment(uint8_t data[FRAGMENT_DLC], const board_state_t& state, uint32_t session_id, uint8_t sample_seq, uint8_t fragment_index);

/**
 * @brief CBoard side of a single telemetry session
 *
 * The server owns a snapshot for the whole four-fragment burst so a sample
 * cannot be torn by the gimbal loop updating state midway.  The caller
 * drives all hardware: it polls received frames, asks whether a handshake
 * response is due, and only advances a fragment after bsp::CAN::TryTransmit()
 * reports QUEUED.
 */
class BoardStateCanServer {
 public:
  explicit BoardStateCanServer(Color color);

  // READY is DataReady() && CaliDone() on the board.
  void SetReady(bool ready);
  bool IsReady() const { return ready_; }
  Status GetStatus() const;
  bool IsStreaming() const;
  uint32_t GetSessionId() const { return session_id_; }
  Color GetColor() const { return color_; }

  /**
   * @brief process one received frame; safe to call for unrelated/malformed data
   * @param now_ms monotonic millisecond tick (wraparound handled)
   */
  void HandleFrame(const uint8_t data[HANDSHAKE_DLC], uint32_t dlc, uint32_t now_ms);

  // Handshake response is due immediately after START and every 100 ms while
  // waiting for the final ACK.  It has priority over telemetry.
  bool IsResponseDue(uint32_t now_ms) const;
  void EncodeResponse(uint8_t data[HANDSHAKE_DLC]) const;
  void OnResponseQueued(uint32_t now_ms);

  // Telemetry.  StartSample() captures one snapshot; only one sample is active
  // at a time, and a new sample cannot start until the fourth fragment of the
  // previous one has been queued.
  bool StartSample(const board_state_t& state, uint32_t now_ms);
  bool HasFragment() const { return sample_active_; }
  bool EncodeFragment(uint8_t data[FRAGMENT_DLC]) const;
  void OnFragmentQueued();

 private:
  void BeginSession(uint32_t session_id, uint32_t now_ms);
  void DropSample();

  Color color_;
  bool ready_ = false;

  bool has_session_ = false;
  uint32_t session_id_ = 0;
  bool version_mismatch_ = false;
  bool final_ack_received_ = false;

  bool response_pending_ = false;
  uint32_t response_due_ms_ = 0;

  bool sample_active_ = false;
  uint8_t sample_seq_ = 0;
  uint8_t fragment_index_ = 0;
  uint32_t next_sample_ms_ = 0;
  board_state_t snapshot_ = {};
};

// Wire layout is part of the protocol.  Keep these checks next to the codec so a
// layout change cannot silently break the fragmented image.
static_assert(PROTOCOL_VERSION == 0x01, "protocol version changed");
static_assert(HANDSHAKE_DLC == 8, "handshake frames must be DLC 8");
static_assert(FRAGMENT_DLC == 8, "fragment frames must be DLC 8");
static_assert(FRAGMENT_COUNT == 4, "board-state payload must use four fragments");
static_assert(SAMPLE_SEQ_MASK == 0x1F, "sample sequence must be five bits");

// The payload is a raw board_state_t image, so both ends must be little-endian.
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "board-state CAN payload assumes little-endian");
#endif

static_assert(sizeof(board_state_t) == BOARD_STATE_PAYLOAD_SIZE, "board_state_t must stay 21 bytes");
static_assert(offsetof(board_state_t, q) == 0, "board_state_t.q offset changed");
static_assert(offsetof(board_state_t, bullet_speed) == 16, "board_state_t.bullet_speed offset changed");
static_assert(offsetof(board_state_t, mode) == 20, "board_state_t.mode offset changed");

static_assert(FRAGMENT_COUNT * FRAGMENT_BYTES == BOARD_STATE_PAYLOAD_SIZE + 3, "four fragments must cover 21 bytes with three pad bytes");
static_assert(BOARD_STATE_PAYLOAD_SIZE - (FRAGMENT_COUNT - 1) * FRAGMENT_BYTES == 3, "fragment 3 must carry three real bytes");
static_assert((1u << FRAGMENT_INDEX_SHIFT) == 32u, "fragment index must start at bit 5");
static_assert(FRAGMENT_COUNT - 1 <= 0x07, "fragment index must fit in bits 7..5");

static_assert(JETSON_TO_BOARD_ID != BOARD_TO_JETSON_ACK_ID, "CAN ids must be distinct");
static_assert(JETSON_TO_BOARD_ID != BOARD_TO_JETSON_DATA_ID, "CAN ids must be distinct");
static_assert(BOARD_TO_JETSON_ACK_ID != BOARD_TO_JETSON_DATA_ID, "CAN ids must be distinct");
static_assert(JETSON_TO_BOARD_ID <= 0x7FF, "classical CAN standard id limit");
static_assert(BOARD_TO_JETSON_ACK_ID <= 0x7FF, "classical CAN standard id limit");
static_assert(BOARD_TO_JETSON_DATA_ID <= 0x7FF, "classical CAN standard id limit");

static_assert(static_cast<uint8_t>(Status::READY) == 0, "READY wire value changed");
static_assert(static_cast<uint8_t>(Status::NOT_READY) == 1, "NOT_READY wire value changed");
static_assert(static_cast<uint8_t>(Status::VERSION_MISMATCH) == 2, "VERSION_MISMATCH wire value changed");
static_assert(static_cast<uint8_t>(Color::UNSPECIFIED) == 0, "color wire value changed");
static_assert(static_cast<uint8_t>(Color::RED) == 1, "color wire value changed");
static_assert(static_cast<uint8_t>(Color::BLUE) == 2, "color wire value changed");

}  // namespace board_state_can
}  // namespace communication
