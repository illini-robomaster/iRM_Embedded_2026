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

#include "board_state_can_protocol.h"

namespace communication {
namespace board_state_can {

namespace {

uint32_t ReadBe32(const uint8_t* data) {
  return (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) | static_cast<uint32_t>(data[3]);
}

void WriteBe32(uint8_t* data, uint32_t value) {
  data[0] = static_cast<uint8_t>(value >> 24);
  data[1] = static_cast<uint8_t>(value >> 16);
  data[2] = static_cast<uint8_t>(value >> 8);
  data[3] = static_cast<uint8_t>(value);
}

// Wraparound-safe comparison of a free-running millisecond tick.
bool TimeReached(uint32_t now_ms, uint32_t deadline_ms) {
  return static_cast<int32_t>(now_ms - deadline_ms) >= 0;
}

}  // namespace

bool DecodeStart(const uint8_t data[HANDSHAKE_DLC], uint32_t dlc, StartMessage* message) {
  if (data == nullptr || message == nullptr || dlc != HANDSHAKE_DLC) return false;
  if (data[0] != static_cast<uint8_t>(MessageType::START)) return false;
  if (data[6] != 0 || data[7] != 0) return false;

  message->version = data[1];
  message->session_id = ReadBe32(&data[2]);
  return true;
}

bool DecodeFinalAck(const uint8_t data[HANDSHAKE_DLC], uint32_t dlc, FinalAckMessage* message) {
  if (data == nullptr || message == nullptr || dlc != HANDSHAKE_DLC) return false;
  if (data[0] != static_cast<uint8_t>(MessageType::FINAL_ACK)) return false;
  if (data[6] != 0 || data[7] != 0) return false;

  message->version = data[1];
  message->session_id = ReadBe32(&data[2]);
  return true;
}

void EncodeAck(uint8_t data[HANDSHAKE_DLC], uint32_t session_id, Status status, Color color) {
  data[0] = static_cast<uint8_t>(MessageType::ACK);
  data[1] = PROTOCOL_VERSION;
  WriteBe32(&data[2], session_id);
  data[6] = static_cast<uint8_t>(status);
  data[7] = static_cast<uint8_t>(color);
}

bool EncodeFragment(uint8_t data[FRAGMENT_DLC], const board_state_t& state, uint32_t session_id, uint8_t sample_seq, uint8_t fragment_index) {
  if (data == nullptr || fragment_index >= FRAGMENT_COUNT) return false;

  // Zero the frame first: this also supplies the mandatory zero padding in
  // fragment 4, byte 7.
  for (uint8_t i = 0; i < FRAGMENT_DLC; ++i) data[i] = 0;

  data[0] = static_cast<uint8_t>((fragment_index << FRAGMENT_INDEX_SHIFT) | (sample_seq & SAMPLE_SEQ_MASK));
  data[1] = static_cast<uint8_t>(session_id & 0xFF);

  const uint8_t offset = static_cast<uint8_t>(fragment_index * FRAGMENT_BYTES);
  const uint8_t count = fragment_index == FRAGMENT_COUNT - 1 ? BOARD_STATE_PAYLOAD_SIZE - offset : FRAGMENT_BYTES;
  // Both targets are little-endian; the payload is a raw board_state_t image.
  const uint8_t* payload = reinterpret_cast<const uint8_t*>(&state);
  for (uint8_t i = 0; i < count; ++i) data[2 + i] = payload[offset + i];

  return true;
}

BoardStateCanServer::BoardStateCanServer(Color color) : color_(color) {}

void BoardStateCanServer::SetReady(bool ready) {
  if (ready_ == ready) return;
  ready_ = ready;
  // Readiness loss invalidates any partially sent sample; never publish a
  // snapshot that was captured before readiness was lost.
  if (!ready_) DropSample();
}

Status BoardStateCanServer::GetStatus() const {
  if (version_mismatch_) return Status::VERSION_MISMATCH;
  return ready_ ? Status::READY : Status::NOT_READY;
}

bool BoardStateCanServer::IsStreaming() const {
  return has_session_ && final_ack_received_ && !version_mismatch_ && ready_;
}

void BoardStateCanServer::BeginSession(uint32_t session_id, uint32_t now_ms) {
  has_session_ = true;
  session_id_ = session_id;
  version_mismatch_ = false;
  final_ack_received_ = false;
  response_pending_ = true;
  response_due_ms_ = now_ms;
  sample_active_ = false;
  sample_seq_ = 0;
  fragment_index_ = 0;
  next_sample_ms_ = now_ms;
}

void BoardStateCanServer::DropSample() {
  if (!sample_active_) return;
  sample_active_ = false;
  fragment_index_ = 0;
  // Never resume an interrupted sample with the same sequence number: the
  // receiver could otherwise combine fragments from two different snapshots.
  sample_seq_ = (sample_seq_ + 1) & SAMPLE_SEQ_MASK;
}

void BoardStateCanServer::HandleFrame(const uint8_t data[HANDSHAKE_DLC], uint32_t dlc, uint32_t now_ms) {
  if (data == nullptr || dlc != HANDSHAKE_DLC) return;

  switch (static_cast<MessageType>(data[0])) {
    case MessageType::START: {
      StartMessage start;
      if (!DecodeStart(data, dlc, &start)) return;

      if (start.version != PROTOCOL_VERSION) {
        // A version mismatch is still a session: echo its id so the Jetson can
        // correlate the response, but never stream for it.
        if (!has_session_ || start.session_id != session_id_) BeginSession(start.session_id, now_ms);
        version_mismatch_ = true;
        final_ack_received_ = false;
        DropSample();
        response_pending_ = true;
        response_due_ms_ = now_ms;
        return;
      }

      if (!has_session_ || start.session_id != session_id_) {
        // New session: stop the old stream and invalidate unsent fragments.
        BeginSession(start.session_id, now_ms);
      } else {
        // Duplicate START for the active session: request another response but
        // do not reset an already streaming session.
        version_mismatch_ = false;
        response_pending_ = true;
        response_due_ms_ = now_ms;
      }
      return;
    }
    case MessageType::FINAL_ACK: {
      FinalAckMessage final_ack;
      if (!DecodeFinalAck(data, dlc, &final_ack)) return;
      if (!has_session_ || final_ack.session_id != session_id_) return;
      if (final_ack.version != PROTOCOL_VERSION || version_mismatch_) return;
      // A final ACK only starts the stream when the board is READY.
      if (!ready_) return;

      if (final_ack_received_) {
        // Duplicate final ACK is idempotent.
        response_pending_ = false;
        return;
      }
      final_ack_received_ = true;
      response_pending_ = false;
      next_sample_ms_ = now_ms;
      return;
    }
    default:
      return;
  }
}

bool BoardStateCanServer::IsResponseDue(uint32_t now_ms) const {
  return has_session_ && response_pending_ && TimeReached(now_ms, response_due_ms_);
}

void BoardStateCanServer::EncodeResponse(uint8_t data[HANDSHAKE_DLC]) const {
  EncodeAck(data, session_id_, GetStatus(), color_);
}

void BoardStateCanServer::OnResponseQueued(uint32_t now_ms) {
  response_due_ms_ = now_ms + HANDSHAKE_RETRY_MS;
  // Once the final ACK has arrived the handshake is complete; repeated STARTs
  // still get an immediate response but no periodic retry.
  response_pending_ = !final_ack_received_;
}

bool BoardStateCanServer::StartSample(const board_state_t& state, uint32_t now_ms) {
  if (!IsStreaming() || sample_active_ || !TimeReached(now_ms, next_sample_ms_)) return false;

  // Capture once and send all five fragments from this copy.  Motor angles and
  // quaternion therefore cannot change midway through a sample.
  snapshot_ = state;
  sample_active_ = true;
  fragment_index_ = 0;
  next_sample_ms_ = now_ms + TELEMETRY_PERIOD_MS;
  return true;
}

bool BoardStateCanServer::EncodeFragment(uint8_t data[FRAGMENT_DLC]) const {
  if (!sample_active_) return false;
  return board_state_can::EncodeFragment(data, snapshot_, session_id_, sample_seq_, fragment_index_);
}

void BoardStateCanServer::OnFragmentQueued() {
  if (!sample_active_) return;

  ++fragment_index_;
  if (fragment_index_ >= FRAGMENT_COUNT) {
    fragment_index_ = 0;
    sample_active_ = false;
    sample_seq_ = (sample_seq_ + 1) & SAMPLE_SEQ_MASK;
  }
}

}  // namespace board_state_can
}  // namespace communication
