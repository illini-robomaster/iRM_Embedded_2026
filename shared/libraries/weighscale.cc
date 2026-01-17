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

/*
 * This is the implementation file for LT-LJ-S weight scale transmitter library.
 * It provides functions to communicate with the weighing scale device over CAN bus.
 */
#include "weighscale.h"

#include <cstring>

#include "cmsis_os.h"

namespace control {

// Response timeout in milliseconds
constexpr uint32_t RESPONSE_TIMEOUT_MS = 50;

// Extended frame ID prefix
constexpr uint32_t EXT_ID_PREFIX = 0xAA0000;

WeighScale::WeighScale(bsp::CAN* can, uint8_t addr, WeighScaleFrameType frame_type)
    : can_(can), addr_(addr), frame_type_(frame_type) {
  memset(&data_, 0, sizeof(data_));
}

void WeighScale::SetAddress(uint8_t addr) {
  addr_ = addr;
}

void WeighScale::SetFrameType(WeighScaleFrameType frame_type) {
  frame_type_ = frame_type;
}

uint32_t WeighScale::CalculateId(WeighScaleFuncCode func_code) {
  uint8_t func = static_cast<uint8_t>(func_code);
  if (frame_type_ == WeighScaleFrameType::STANDARD) {
    // Standard frame: ID = (func << 8) | addr
    return (static_cast<uint32_t>(func) << 8) | addr_;
  } else {
    // Extended frame: ID = 0xAA0000 | (func << 8) | addr
    return EXT_ID_PREFIX | (static_cast<uint32_t>(func) << 8) | addr_;
  }
}

int WeighScale::TransmitFrame(uint32_t id, const uint8_t data[8]) {
  // Note: Currently bsp::CAN only supports standard frames
  // For extended frame support, the BSP layer would need to be extended
  // For now, we use standard frame transmission which works for 11-bit IDs
  
  // If extended frame is requested, we truncate to 16-bit for standard CAN
  // TODO: Add extended frame support to bsp::CAN if needed
  uint16_t std_id = static_cast<uint16_t>(id & 0xFFFF);
  
  return can_->Transmit(std_id, data, 8);
}

uint32_t WeighScale::ParseU32BE(const uint8_t* data) {
  return (static_cast<uint32_t>(data[0]) << 24) |
         (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) |
         static_cast<uint32_t>(data[3]);
}

void WeighScale::PackU32BE(uint32_t value, uint8_t* data) {
  data[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
  data[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
  data[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[3] = static_cast<uint8_t>(value & 0xFF);
}

float WeighScale::ParseF32BE(const uint8_t* data) {
  uint32_t raw = ParseU32BE(data);
  float result;
  memcpy(&result, &raw, sizeof(float));
  return result;
}

void WeighScale::PackF32BE(float value, uint8_t* data) {
  uint32_t raw;
  memcpy(&raw, &value, sizeof(uint32_t));
  PackU32BE(raw, data);
}

bool WeighScale::Tare(uint8_t channel) {
  uint8_t data[8] = {0};
  data[0] = channel;  // Channel (1-14 or 0x0F for all)
  
  uint32_t id = CalculateId(WeighScaleFuncCode::TARE);
  int ret = TransmitFrame(id, data);
  
  if (ret > 0) {
    connection_flag_ = true;
    // Device echoes the frame as ACK
    // In polling mode, we assume success if transmit succeeded
    return true;
  }
  return false;
}

bool WeighScale::ZeroCalibrate(uint8_t channel) {
  uint8_t data[8] = {0};
  data[0] = channel;  // Channel (1-14 or 0x0F for all)
  
  uint32_t id = CalculateId(WeighScaleFuncCode::ZERO_CALIBRATE);
  int ret = TransmitFrame(id, data);
  
  if (ret > 0) {
    connection_flag_ = true;
    return true;
  }
  return false;
}

uint8_t WeighScale::ReadWeights(WeighScaleData_t* data, uint8_t num_channels) {
  // Clear existing data
  memset(data, 0, sizeof(WeighScaleData_t));
  memset(&data_, 0, sizeof(data_));
  
  // Send read request
  uint8_t tx_data[8] = {0};
  uint32_t id = CalculateId(WeighScaleFuncCode::READ_WEIGHT);
  int ret = TransmitFrame(id, tx_data);
  
  if (ret <= 0) {
    return 0;
  }
  
  connection_flag_ = true;
  
  // Note: Response handling requires callback registration
  // The device responds with multiple frames:
  // - STD: 0x302 (ch1,2), 0x303 (ch3,4), etc.
  // - EXT: 0xAA0302 (ch1,2), 0xAA0303 (ch3,4), etc.
  //
  // For proper implementation, you should:
  // 1. Register callbacks for expected response IDs before calling this
  // 2. Or implement a blocking receive with timeout
  //
  // Current implementation just sends the request.
  // The response parsing should be done in a callback registered with:
  //   can_->RegisterRxCallback(response_id, callback, args);
  //
  // Example response ID calculation:
  //   STD: 0x302 + ((ch_pair-1)/2) where ch_pair starts at 1
  //   EXT: 0xAA0302 + ((ch_pair-1)/2)
  
  // Return expected channel count (actual data comes via callback)
  data->valid_channels = num_channels;
  return num_channels;
}

uint8_t WeighScale::ReadAddress() {
  uint8_t data[8] = {0};
  data[0] = static_cast<uint8_t>(WeighScaleCommCmd::READ_ADDR);
  
  uint32_t id = CalculateId(WeighScaleFuncCode::COMM_PARAM);
  int ret = TransmitFrame(id, data);
  
  if (ret > 0) {
    connection_flag_ = true;
    // Response comes via callback with data[1] = address
    // For polling, return current address
    return addr_;
  }
  return 0;
}

bool WeighScale::WriteAddress(uint8_t new_addr) {
  uint8_t data[8] = {0};
  data[0] = static_cast<uint8_t>(WeighScaleCommCmd::WRITE_ADDR);
  data[1] = new_addr;
  
  uint32_t id = CalculateId(WeighScaleFuncCode::COMM_PARAM);
  int ret = TransmitFrame(id, data);
  
  if (ret > 0) {
    connection_flag_ = true;
    // Note: Requires power cycle (5 seconds) to take effect
    return true;
  }
  return false;
}

WeighScaleBaudCode WeighScale::ReadBaudCode() {
  uint8_t data[8] = {0};
  data[0] = static_cast<uint8_t>(WeighScaleCommCmd::READ_BAUD);
  
  uint32_t id = CalculateId(WeighScaleFuncCode::COMM_PARAM);
  int ret = TransmitFrame(id, data);
  
  if (ret > 0) {
    connection_flag_ = true;
    // Response comes via callback with data[1] = baud_code
    // Default is 1Mbps
    return WeighScaleBaudCode::BAUD_1M;
  }
  return WeighScaleBaudCode::BAUD_1M;
}

bool WeighScale::WriteBaudCode(WeighScaleBaudCode baud_code) {
  uint8_t data[8] = {0};
  data[0] = static_cast<uint8_t>(WeighScaleCommCmd::WRITE_BAUD);
  data[1] = static_cast<uint8_t>(baud_code);
  
  uint32_t id = CalculateId(WeighScaleFuncCode::COMM_PARAM);
  int ret = TransmitFrame(id, data);
  
  if (ret > 0) {
    connection_flag_ = true;
    // Note: Requires power cycle (5 seconds) to take effect
    return true;
  }
  return false;
}

bool WeighScale::SetSampleRate(uint8_t channel, WeighScaleSampleRate rate) {
  uint8_t data[8] = {0};
  data[0] = static_cast<uint8_t>(WeighScaleSampleCmd::SAMPLE_RATE);
  data[1] = channel;  // Channel (1-14)
  data[2] = static_cast<uint8_t>(rate);
  
  uint32_t id = CalculateId(WeighScaleFuncCode::SAMPLE_CONFIG);
  int ret = TransmitFrame(id, data);
  
  if (ret > 0) {
    connection_flag_ = true;
    // Note: Does NOT persist after power cycle
    return true;
  }
  return false;
}

bool WeighScale::SetZeroTracking(uint8_t channel, uint8_t range) {
  // Clamp range to 0-100
  if (range > 100) {
    range = 100;
  }
  
  uint8_t data[8] = {0};
  data[0] = static_cast<uint8_t>(WeighScaleSampleCmd::ZERO_TRACKING);
  data[1] = channel;  // Channel (1-14)
  data[2] = range;
  
  uint32_t id = CalculateId(WeighScaleFuncCode::SAMPLE_CONFIG);
  int ret = TransmitFrame(id, data);
  
  if (ret > 0) {
    connection_flag_ = true;
    // Note: Does NOT persist after power cycle
    return true;
  }
  return false;
}

bool WeighScale::SetDivision(uint8_t channel, uint8_t division) {
  // Valid division values: 1, 2, 5, 10, 20, 50, 100
  uint8_t data[8] = {0};
  data[0] = static_cast<uint8_t>(WeighScaleSampleCmd::DIVISION);
  data[1] = channel;  // Channel (1-14)
  data[2] = division;
  
  uint32_t id = CalculateId(WeighScaleFuncCode::SAMPLE_CONFIG);
  int ret = TransmitFrame(id, data);
  
  if (ret > 0) {
    connection_flag_ = true;
    // Note: Does NOT persist after power cycle
    return true;
  }
  return false;
}

bool WeighScale::SetCalibrationFactor(uint8_t channel, float factor) {
  uint8_t data[8] = {0};
  data[0] = channel;  // Channel (1-14)
  PackF32BE(factor, &data[1]);  // Float32 in bytes 1-4 (big-endian)
  
  uint32_t id = CalculateId(WeighScaleFuncCode::CALIB_FACTOR);
  int ret = TransmitFrame(id, data);
  
  if (ret > 0) {
    connection_flag_ = true;
    // Note: Persists after power cycle, but requires restart to take effect
    return true;
  }
  return false;
}

// Utility functions

uint32_t WeighScaleBaudCodeToBps(WeighScaleBaudCode code) {
  switch (code) {
    case WeighScaleBaudCode::BAUD_20K:  return 20000;
    case WeighScaleBaudCode::BAUD_50K:  return 50000;
    case WeighScaleBaudCode::BAUD_100K: return 100000;
    case WeighScaleBaudCode::BAUD_125K:
      return 125000;  // Fixed: was incorrectly 120000
    case WeighScaleBaudCode::BAUD_200K: return 200000;
    case WeighScaleBaudCode::BAUD_250K: return 250000;
    case WeighScaleBaudCode::BAUD_400K: return 400000;
    case WeighScaleBaudCode::BAUD_500K: return 500000;
    case WeighScaleBaudCode::BAUD_800K: return 800000;
    case WeighScaleBaudCode::BAUD_1M:   return 1000000;
    default: return 500000;
  }
}

WeighScaleBaudCode WeighScaleBpsToBaudCode(uint32_t bps) {
  switch (bps) {
    case 20000:   return WeighScaleBaudCode::BAUD_20K;
    case 50000:   return WeighScaleBaudCode::BAUD_50K;
    case 100000:  return WeighScaleBaudCode::BAUD_100K;
    case 125000:
      return WeighScaleBaudCode::BAUD_125K;  // Fixed: was incorrectly 120000
    case 200000:  return WeighScaleBaudCode::BAUD_200K;
    case 250000:  return WeighScaleBaudCode::BAUD_250K;
    case 400000:  return WeighScaleBaudCode::BAUD_400K;
    case 500000:  return WeighScaleBaudCode::BAUD_500K;
    case 800000:  return WeighScaleBaudCode::BAUD_800K;
    case 1000000: return WeighScaleBaudCode::BAUD_1M;
    default:      return WeighScaleBaudCode::BAUD_500K;
  }
}

}  // namespace control
