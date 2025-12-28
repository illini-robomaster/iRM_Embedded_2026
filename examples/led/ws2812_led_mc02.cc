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

/**
 * @file ws2812_mc02.cc
 * @brief WS2812 RGB LED Example for DM_MC_02
 *
 * Hardware Setup:
 *   - WS2812 LED strip connected to PA7 (SPI6_MOSI)
 *   - Power supply: 5V (external) for LEDs, 3.3V logic level from MCU
 *   - SPI6_SCK on PA5 (not connected, just for CubeMX)
 *
 * SPI6 Configuration (already done in CubeMX):
 *   - Mode: Transmit Only Master
 *   - Clock: 24MHz HSE based
 */

#include "main.h"
#include "usart.h"

#include "bsp_print.h"
#include "bsp_ws2812.h"
#include "cmsis_os.h"

// Number of WS2812 LEDs in the strip
#define NUM_LEDS 8

// SPI6 handle for WS2812 (PA7 = MOSI)
extern SPI_HandleTypeDef hspi6;

static bsp::WS2812* led_strip = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart7);

  // Initialize WS2812 using SPI6 (PA7)
  led_strip = new bsp::WS2812(&hspi6, NUM_LEDS);
  
  print("WS2812 LED initialized\r\n");
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  print("WS2812 LED Demo - Using SPI6 (PA7)\r\n");

  uint8_t hue = 0;
  uint8_t effect = 0;

  while (true) {
    switch (effect) {
      case 0:
        // Effect 1: Color cycle - all LEDs same color
        {
          // Simple HSV to RGB (hue only, full saturation and value)
          uint8_t r, g, b;
          uint8_t region = hue / 43;
          uint8_t remainder = (hue - region * 43) * 6;

          switch (region) {
            case 0: r = 255; g = remainder; b = 0; break;
            case 1: r = 255 - remainder; g = 255; b = 0; break;
            case 2: r = 0; g = 255; b = remainder; break;
            case 3: r = 0; g = 255 - remainder; b = 255; break;
            case 4: r = remainder; g = 0; b = 255; break;
            default: r = 255; g = 0; b = 255 - remainder; break;
          }

          led_strip->SetAll(r, g, b);
          led_strip->Update();

          hue += 2;
          if (hue == 0) {
            effect = 1;
            print("Effect: Chase\r\n");
          }
        }
        break;

      case 1:
        // Effect 2: Chase pattern
        {
          static uint8_t pos = 0;
          led_strip->Clear();

          // Set current LED to red
          led_strip->SetColor(pos, bsp::WS2812::COLOR_RED);
          // Set trailing LEDs with fading colors
          if (pos > 0) led_strip->SetColor(pos - 1, 0x00800000);  // Dimmer red
          if (pos > 1) led_strip->SetColor(pos - 2, 0x00400000);  // Even dimmer

          led_strip->Update();

          pos++;
          if (pos >= NUM_LEDS) {
            pos = 0;
            hue++;
            if (hue >= 5) {
              effect = 2;
              hue = 0;
              print("Effect: Rainbow\r\n");
            }
          }
        }
        break;

      case 2:
        // Effect 3: Rainbow spread across strip
        {
          for (uint16_t i = 0; i < NUM_LEDS; i++) {
            uint8_t led_hue = hue + (i * 256 / NUM_LEDS);
            uint8_t r, g, b;
            uint8_t region = led_hue / 43;
            uint8_t remainder = (led_hue - region * 43) * 6;

            switch (region) {
              case 0: r = 255; g = remainder; b = 0; break;
              case 1: r = 255 - remainder; g = 255; b = 0; break;
              case 2: r = 0; g = 255; b = remainder; break;
              case 3: r = 0; g = 255 - remainder; b = 255; break;
              case 4: r = remainder; g = 0; b = 255; break;
              default: r = 255; g = 0; b = 255 - remainder; break;
            }

            led_strip->SetColor(i, r, g, b);
          }
          led_strip->Update();

          hue += 2;
          static uint8_t count = 0;
          if (++count >= 128) {
            count = 0;
            effect = 3;
            print("Effect: Blink\r\n");
          }
        }
        break;

      case 3:
        // Effect 4: Blink different colors
        {
          static uint8_t blink_state = 0;
          static const uint32_t colors[] = {
            bsp::WS2812::COLOR_RED,
            bsp::WS2812::COLOR_GREEN,
            bsp::WS2812::COLOR_BLUE,
            bsp::WS2812::COLOR_YELLOW,
            bsp::WS2812::COLOR_CYAN,
            bsp::WS2812::COLOR_MAGENTA,
            bsp::WS2812::COLOR_WHITE,
            bsp::WS2812::COLOR_OFF
          };

          led_strip->SetAll(colors[blink_state % 8]);
          led_strip->Update();

          blink_state++;
          if (blink_state >= 16) {
            blink_state = 0;
            effect = 0;
            print("Effect: Color Cycle\r\n");
          }
          osDelay(300);  // Slower for blink effect
          continue;
        }
        break;
    }

    osDelay(20);  // ~50 FPS update rate
  }
}
