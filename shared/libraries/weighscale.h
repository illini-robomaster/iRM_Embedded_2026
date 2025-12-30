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

#pragma once

#include "bsp_can.h"

namespace control {

/**
 * @brief Maximum number of channels supported by the weighing transmitter
 */
constexpr uint8_t WEIGHSCALE_MAX_CHANNELS = 14;

/**
 * @brief Special channel value for "all channels"
 */
constexpr uint8_t WEIGHSCALE_ALL_CHANNELS = 0x0F;

/**
 * @brief CAN frame type selection
 */
enum class WeighScaleFrameType {
  STANDARD,  // 11-bit ID
  EXTENDED   // 29-bit ID
};

/**
 * @brief Baud rate codes for CAN communication
 */
enum class WeighScaleBaudCode : uint8_t {
  BAUD_20K = 0x02,
  BAUD_50K = 0x03,
  BAUD_100K = 0x04,
  BAUD_120K = 0x05,
  BAUD_200K = 0x06,
  BAUD_250K = 0x07,
  BAUD_400K = 0x08,
  BAUD_500K = 0x09,
  BAUD_800K = 0x0A,
  BAUD_1M = 0x0B    // Default (1Mbps)
};

/**
 * @brief Sample rate options
 */
enum class WeighScaleSampleRate : uint8_t {
  RATE_10HZ = 0x01,
  RATE_40HZ = 0x02
};

/**
 * @brief Function codes for CAN protocol
 */
enum class WeighScaleFuncCode : uint8_t {
  TARE = 0x01,             // Execute tare on channel(s)
  ZERO_CALIBRATE = 0x02,   // Zero point calibration
  READ_WEIGHT = 0x03,      // Read weight values
  COMM_PARAM = 0x04,       // Read/write communication parameters
  SAMPLE_CONFIG = 0x05,    // Sample rate, zero tracking, division
  CALIB_FACTOR = 0x06      // Calibration factor (float32)
};

/**
 * @brief Communication parameter sub-commands
 */
enum class WeighScaleCommCmd : uint8_t {
  READ_ADDR = 0xA1,
  READ_BAUD = 0xA2,
  WRITE_ADDR = 0xB1,
  WRITE_BAUD = 0xB2
};

/**
 * @brief Sample config sub-commands
 */
enum class WeighScaleSampleCmd : uint8_t {
  SAMPLE_RATE = 0xA1,
  ZERO_TRACKING = 0xB1,
  DIVISION = 0xC1
};

/**
 * @brief Weight data for all channels
 */
typedef struct {
  uint32_t weight[WEIGHSCALE_MAX_CHANNELS];  // Weight in grams (uint32)
  uint8_t valid_channels;                     // Number of valid channels received
} __packed WeighScaleData_t;

/**
 * @brief Multi-channel CAN weighing transmitter driver
 *
 * Supports:
 * - Standard frame (11-bit ID) and Extended frame (29-bit ID)
 * - Up to 14 channels
 * - Tare, zero calibration, weight reading
 * - Communication parameter configuration
 * - Sample rate, zero tracking, division settings
 * - Calibration factor configuration
 *
 * Protocol:
 * - Data bytes are always 8 bytes, unused bytes filled with 0x00
 * - Weight values are uint32 in grams, big-endian
 * - Calibration factors are IEEE754 float32, big-endian
 */
class WeighScale {
 public:
  /**
   * @brief Constructor
   * @param can Pointer to CAN instance
   * @param addr Device address (1-255, default 1)
   * @param frame_type Frame type (STANDARD or EXTENDED)
   */
  WeighScale(bsp::CAN* can, uint8_t addr = 1,
             WeighScaleFrameType frame_type = WeighScaleFrameType::STANDARD);

  /**
   * @brief Set the device address
   * @param addr Device address (1-255)
   */
  void SetAddress(uint8_t addr);

  /**
   * @brief Set the frame type
   * @param frame_type STANDARD (11-bit) or EXTENDED (29-bit)
   */
  void SetFrameType(WeighScaleFrameType frame_type);

  /**
   * @brief Execute tare on specified channel
   * @param channel Channel number (1-14) or WEIGHSCALE_ALL_CHANNELS (0x0F)
   * @return true if ACK received
   */
  bool Tare(uint8_t channel);

  /**
   * @brief Execute zero point calibration on specified channel
   * @param channel Channel number (1-14) or WEIGHSCALE_ALL_CHANNELS (0x0F)
   * @return true if ACK received
   */
  bool ZeroCalibrate(uint8_t channel);

  /**
   * @brief Read weight from all channels
   * @param data Pointer to WeighScaleData_t structure to store results
   * @param num_channels Expected number of channels (for timeout calculation)
   * @return Number of channels successfully read
   */
  uint8_t ReadWeights(WeighScaleData_t* data, uint8_t num_channels = 4);

  /**
   * @brief Read the current device address
   * @return Device address, or 0 on failure
   */
  uint8_t ReadAddress();

  /**
   * @brief Write a new device address
   * @note Requires power cycle (5 seconds) to take effect
   * @param new_addr New address (1-255)
   * @return true if ACK received
   */
  bool WriteAddress(uint8_t new_addr);

  /**
   * @brief Read the current baud rate code
   * @return Baud rate code, or 0 on failure
   */
  WeighScaleBaudCode ReadBaudCode();

  /**
   * @brief Write a new baud rate code
   * @note Requires power cycle (5 seconds) to take effect
   * @param baud_code New baud rate code
   * @return true if ACK received
   */
  bool WriteBaudCode(WeighScaleBaudCode baud_code);

  /**
   * @brief Set sample rate for a channel
   * @note Does NOT persist after power cycle
   * @param channel Channel number (1-14)
   * @param rate Sample rate (10Hz or 40Hz)
   * @return true if ACK received
   */
  bool SetSampleRate(uint8_t channel, WeighScaleSampleRate rate);

  /**
   * @brief Set zero tracking range for a channel
   * @note Does NOT persist after power cycle
   * @param channel Channel number (1-14)
   * @param range Range value (0-100)
   * @return true if ACK received
   */
  bool SetZeroTracking(uint8_t channel, uint8_t range);

  /**
   * @brief Set division value for a channel
   * @note Does NOT persist after power cycle
   * @param channel Channel number (1-14)
   * @param division Division value (1, 2, 5, 10, 20, 50, or 100)
   * @return true if ACK received
   */
  bool SetDivision(uint8_t channel, uint8_t division);

  /**
   * @brief Set calibration factor for a channel
   * @note Persists after power cycle, but requires restart to take effect
   * @param channel Channel number (1-14)
   * @param factor Calibration factor (float32)
   * @return true if ACK received
   */
  bool SetCalibrationFactor(uint8_t channel, float factor);

  /**
   * @brief Get the latest weight data
   * @return Reference to internal weight data structure
   */
  const WeighScaleData_t& GetData() const { return data_; }

  /**
   * @brief Connection status flag (set after successful communication)
   */
  volatile bool connection_flag_ = false;

 private:
  // CAN instance
  bsp::CAN* can_;

  // Device address
  uint8_t addr_;

  // Frame type
  WeighScaleFrameType frame_type_;

  // Internal weight data storage
  WeighScaleData_t data_;

  /**
   * @brief Calculate CAN ID based on function code and current settings
   * @param func_code Function code
   * @return CAN ID (11-bit or 29-bit depending on frame_type_)
   */
  uint32_t CalculateId(WeighScaleFuncCode func_code);

  /**
   * @brief Transmit a CAN frame (standard or extended based on settings)
   * @param id CAN ID
   * @param data Data bytes (8 bytes)
   * @return Number of bytes transmitted, -1 on failure
   */
  int TransmitFrame(uint32_t id, const uint8_t data[8]);

  /**
   * @brief Parse uint32 from big-endian bytes
   * @param data Pointer to 4 bytes (MSB first)
   * @return Parsed uint32 value
   */
  static uint32_t ParseU32BE(const uint8_t* data);

  /**
   * @brief Pack uint32 to big-endian bytes
   * @param value Value to pack
   * @param data Pointer to 4 bytes output buffer
   */
  static void PackU32BE(uint32_t value, uint8_t* data);

  /**
   * @brief Parse float32 from big-endian bytes (IEEE754)
   * @param data Pointer to 4 bytes (MSB first)
   * @return Parsed float value
   */
  static float ParseF32BE(const uint8_t* data);

  /**
   * @brief Pack float32 to big-endian bytes (IEEE754)
   * @param value Value to pack
   * @param data Pointer to 4 bytes output buffer
   */
  static void PackF32BE(float value, uint8_t* data);
};

/**
 * @brief Convert baud code to actual baud rate in bps
 * @param code Baud rate code
 * @return Baud rate in bps
 */
uint32_t WeighScaleBaudCodeToBps(WeighScaleBaudCode code);

/**
 * @brief Convert baud rate to baud code
 * @param bps Baud rate in bps
 * @return Baud code, or BAUD_500K if not found
 */
WeighScaleBaudCode WeighScaleBpsToBaudCode(uint32_t bps);

}  // namespace control
