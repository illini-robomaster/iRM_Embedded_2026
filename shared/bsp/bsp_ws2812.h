/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2024 RoboMaster.                                          *
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

#include "main.h"
#include <cstdint>

namespace bsp {

/**
 * @brief WS2812 LED Driver using SPI MOSI
 *
 * This driver uses SPI to generate the WS2812 timing protocol.
 * The WS2812 protocol requires:
 *   - 0 bit: ~0.4us high, ~0.85us low
 *   - 1 bit: ~0.8us high, ~0.45us low
 *   - Reset: >50us low
 *
 * By setting SPI clock to ~6.4MHz (or 8MHz), each SPI bit is ~125-156ns.
 * We use 8 SPI bits to represent one WS2812 bit:
 *   - 0 bit: 0b11000000 (0xC0) - 2 high bits, 6 low bits
 *   - 1 bit: 0b11111100 (0xFC) - 6 high bits, 2 low bits
 *
 * For DM_MC_02 (STM32H7):
 *   - SPI6 on PA7 (MOSI)
 *   - SPI clock source: 24MHz HSE
 *   - Prescaler: 4 -> 6MHz SPI clock -> ~166ns per bit
 *
 * Hardware Configuration (CubeMX):
 *   1. Enable SPI6 in Transmit Only Master mode
 *   2. Set PA7 as SPI6_MOSI
 *   3. Configure SPI6:
 *      - Data Size: 8 Bits
 *      - First Bit: MSB First
 *      - Prescaler: 4 (for ~6MHz)
 *      - CPOL: Low, CPHA: 1 Edge
 *   4. Enable SPI6 TX DMA (optional for better performance)
 */
class WS2812 {
 public:
  /**
   * @brief Construct a WS2812 driver
   * @param hspi Pointer to SPI handle
   * @param num_leds Number of LEDs in the strip
   */
  WS2812(SPI_HandleTypeDef* hspi, uint16_t num_leds);

  /**
   * @brief Destructor
   */
  ~WS2812();

  /**
   * @brief Set color for a specific LED
   * @param index LED index (0-based)
   * @param r Red value (0-255)
   * @param g Green value (0-255)
   * @param b Blue value (0-255)
   */
  void SetColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b);

  /**
   * @brief Set color for a specific LED using 32-bit color
   * @param index LED index (0-based)
   * @param color Color in 0x00RRGGBB format
   */
  void SetColor(uint16_t index, uint32_t color);

  /**
   * @brief Set all LEDs to the same color
   * @param r Red value (0-255)
   * @param g Green value (0-255)
   * @param b Blue value (0-255)
   */
  void SetAll(uint8_t r, uint8_t g, uint8_t b);

  /**
   * @brief Set all LEDs to the same color
   * @param color Color in 0x00RRGGBB format
   */
  void SetAll(uint32_t color);

  /**
   * @brief Clear all LEDs (turn off)
   */
  void Clear();

  /**
   * @brief Update the LED strip (send data via SPI)
   */
  void Update();

  /**
   * @brief Get number of LEDs
   */
  uint16_t GetNumLeds() const { return num_leds_; }

  // Predefined colors
  static constexpr uint32_t COLOR_RED     = 0x00FF0000;
  static constexpr uint32_t COLOR_GREEN   = 0x0000FF00;
  static constexpr uint32_t COLOR_BLUE    = 0x000000FF;
  static constexpr uint32_t COLOR_WHITE   = 0x00FFFFFF;
  static constexpr uint32_t COLOR_YELLOW  = 0x00FFFF00;
  static constexpr uint32_t COLOR_CYAN    = 0x0000FFFF;
  static constexpr uint32_t COLOR_MAGENTA = 0x00FF00FF;
  static constexpr uint32_t COLOR_ORANGE  = 0x00FF8000;
  static constexpr uint32_t COLOR_PURPLE  = 0x008000FF;
  static constexpr uint32_t COLOR_OFF     = 0x00000000;

 private:
  SPI_HandleTypeDef* hspi_;
  uint16_t num_leds_;
  uint8_t* buffer_;        // SPI transmit buffer
  uint32_t buffer_size_;   // Size of SPI buffer

  // Convert one byte to 8 SPI bytes (for WS2812 timing)
  void ByteToSPI(uint8_t byte, uint8_t* spi_data);

  // Reset pulse length (in SPI bytes, >50us at 6MHz = >300 bits = 38 bytes)
  static constexpr uint16_t RESET_BYTES = 48;

  // SPI encoding for WS2812 bits
  static constexpr uint8_t WS2812_0 = 0xC0;  // 0b11000000 - short pulse
  static constexpr uint8_t WS2812_1 = 0xFC;  // 0b11111100 - long pulse
};

}  // namespace bsp
