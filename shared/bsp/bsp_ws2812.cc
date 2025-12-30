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

#include "bsp_ws2812.h"
#include <cstring>

namespace bsp {

WS2812::WS2812(SPI_HandleTypeDef* hspi, uint16_t num_leds)
    : hspi_(hspi), num_leds_(num_leds) {
  // Reconfigure SPI for WS2812 timing requirements
  // WS2812 needs ~6MHz SPI clock, 8-bit data, CPOL=Low, CPHA=1Edge
  hspi_->Init.DataSize = SPI_DATASIZE_8BIT;
  hspi_->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;  // 24MHz/4 = 6MHz
  hspi_->Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi_->Init.CLKPhase = SPI_PHASE_1EDGE;
  HAL_SPI_Init(hspi_);

  // Each LED needs 24 bits (8 for each color: G, R, B)
  // Each WS2812 bit needs 8 SPI bits
  // Plus reset pulse at the end
  buffer_size_ = (num_leds_ * 24 * 8) / 8 + RESET_BYTES;
  buffer_ = new uint8_t[buffer_size_];
  Clear();
}

WS2812::~WS2812() {
  delete[] buffer_;
}

void WS2812::ByteToSPI(uint8_t byte, uint8_t* spi_data) {
  for (int i = 0; i < 8; i++) {
    if (byte & (0x80 >> i)) {
      spi_data[i] = WS2812_1;
    } else {
      spi_data[i] = WS2812_0;
    }
  }
}

void WS2812::SetColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
  if (index >= num_leds_) return;

  // WS2812 expects GRB order
  uint8_t* led_data = buffer_ + (index * 24);  // 24 SPI bytes per LED

  ByteToSPI(g, led_data);
  ByteToSPI(r, led_data + 8);
  ByteToSPI(b, led_data + 16);
}

void WS2812::SetColor(uint16_t index, uint32_t color) {
  uint8_t r = (color >> 16) & 0xFF;
  uint8_t g = (color >> 8) & 0xFF;
  uint8_t b = color & 0xFF;
  SetColor(index, r, g, b);
}

void WS2812::SetAll(uint8_t r, uint8_t g, uint8_t b) {
  for (uint16_t i = 0; i < num_leds_; i++) {
    SetColor(i, r, g, b);
  }
}

void WS2812::SetAll(uint32_t color) {
  for (uint16_t i = 0; i < num_leds_; i++) {
    SetColor(i, color);
  }
}

void WS2812::Clear() {
  memset(buffer_, 0, buffer_size_);
  // Initialize all LEDs to off (all WS2812_0 patterns)
  for (uint32_t i = 0; i < buffer_size_ - RESET_BYTES; i++) {
    buffer_[i] = WS2812_0;
  }
  // Reset pulse at the end (all zeros)
  memset(buffer_ + buffer_size_ - RESET_BYTES, 0, RESET_BYTES);
}

void WS2812::Update() {
  // Send data via SPI (blocking mode)
  HAL_SPI_Transmit(hspi_, buffer_, buffer_size_, HAL_MAX_DELAY);
}

}  // namespace bsp
