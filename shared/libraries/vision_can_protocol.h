#pragma once

#include <cstdint>
#include <cstring>

namespace communication {
namespace vision_can {

// CAN IDs from vision-embedded-can-protocol.md
constexpr uint16_t HELLO_CAN_ID = 0x310;         // Embedded -> Vision
constexpr uint16_t HELLO_ACK_CAN_ID = 0x311;     // Vision -> Embedded
constexpr uint16_t CONTROL_CAN_ID = 0x312;       // Embedded -> Vision
constexpr uint16_t RELATIVE_OBS_CAN_ID = 0x0FE;  // Vision -> Embedded

constexpr uint8_t PROTOCOL_VERSION = 0x01;
constexpr uint8_t CAN_DLC = 8;

enum class Status : uint8_t {
  READY = 0,
  NOT_READY = 1,
  VERSION_MISMATCH = 2,
};

enum class Color : uint8_t {
  RED = 1,
  BLUE = 2,
};

// Encode HELLO frame (0x310)
// Layout: [0x01, 0x01, session_BE (4 bytes), 0x00, 0x00]
inline void EncodeHello(uint8_t out[CAN_DLC], uint32_t session_id) {
  out[0] = 0x01;
  out[1] = PROTOCOL_VERSION;
  out[2] = static_cast<uint8_t>((session_id >> 24) & 0xFF);
  out[3] = static_cast<uint8_t>((session_id >> 16) & 0xFF);
  out[4] = static_cast<uint8_t>((session_id >> 8) & 0xFF);
  out[5] = static_cast<uint8_t>(session_id & 0xFF);
  out[6] = 0x00;
  out[7] = 0x00;
}

// Decode HELLO_ACK frame (0x311)
// Layout: [0x02, 0x01, session_BE (4 bytes), status, 0x00]
inline bool DecodeHelloAck(const uint8_t in[CAN_DLC], uint32_t* session_id, Status* status) {
  if (in[0] != 0x02 || in[1] != PROTOCOL_VERSION) {
    return false;
  }
  if (session_id) {
    *session_id = (static_cast<uint32_t>(in[2]) << 24) |
                  (static_cast<uint32_t>(in[3]) << 16) |
                  (static_cast<uint32_t>(in[4]) << 8) |
                  static_cast<uint32_t>(in[5]);
  }
  if (status) {
    *status = static_cast<Status>(in[6]);
  }
  return true;
}

// Encode CONTROL frame (0x312)
// Layout: [0x03, 0x01, session_BE (4 bytes), flags, sequence]
// flags: Bit 0: enable; Bits 1-3: target color (1 = RED, 2 = BLUE)
inline void EncodeControl(uint8_t out[CAN_DLC], uint32_t session_id, bool enable, Color target_color, uint8_t sequence) {
  out[0] = 0x03;
  out[1] = PROTOCOL_VERSION;
  out[2] = static_cast<uint8_t>((session_id >> 24) & 0xFF);
  out[3] = static_cast<uint8_t>((session_id >> 16) & 0xFF);
  out[4] = static_cast<uint8_t>((session_id >> 8) & 0xFF);
  out[5] = static_cast<uint8_t>(session_id & 0xFF);
  uint8_t flags = (enable ? 0x01 : 0x00) | ((static_cast<uint8_t>(target_color) & 0x07) << 1);
  out[6] = flags;
  out[7] = sequence;
}

// Relative Observation (0x0FE)
// Layout: [control, 0x00, delta_yaw_BE (2 bytes), delta_pitch_BE (2 bytes), 0x00, 0x00]
struct RelativeObservation {
  bool valid;
  float delta_yaw;    // in radians (positive = left)
  float delta_pitch;  // in radians (negative = up)
};

inline bool DecodeRelativeObservation(const uint8_t in[CAN_DLC], RelativeObservation* obs) {
  if (!obs) return false;
  obs->valid = (in[0] & 0x01) != 0;
  if (!obs->valid) {
    obs->delta_yaw = 0.0f;
    obs->delta_pitch = 0.0f;
    return true;
  }
  int16_t raw_yaw = static_cast<int16_t>((static_cast<uint16_t>(in[2]) << 8) | in[3]);
  int16_t raw_pitch = static_cast<int16_t>((static_cast<uint16_t>(in[4]) << 8) | in[5]);
  obs->delta_yaw = raw_yaw * 0.0001f;
  obs->delta_pitch = raw_pitch * 0.0001f;
  return true;
}

}  // namespace vision_can
}  // namespace communication
