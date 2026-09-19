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

// Pin finder for the 4-button OLED module on DJI_Board_TypeA, revision 3.
//
// The module's buttons are plain switches to ground with a pull-up, so they
// show up as a pin that idles high and drops low while held.  Two of them are
// already known from measurement:
//
//   PC0 / ADC_CHANNEL_10 - "up",   4095 idle -> ~0 pressed
//   PC1 / ADC_CHANNEL_11 - "down", 4095 idle -> ~0 pressed
//
// Both of those sit in GPIO_MODE_ANALOG (CubeMX put them in the ADC scan), so
// their digital input buffer is off and IDR always reads 0 for them.  The
// remaining two buttons are on unknown pins, so this pass watches every GPIO
// port at once.
//
// It only ever READS the IDR registers.  No pin is reconfigured and nothing is
// driven, which matters on this board: PH2-PH5 are MOS_CTL1-4, PG13 is LASER
// and PI9/PF10 are Q1/Q2, so blindly driving pins to hunt for a button could
// switch real hardware.
//
// Pins that toggle on their own (UART, I2C, SPI, ...) would otherwise show up
// as false hits, so the first LEARN_MS are spent recording which bits move
// while nothing is touched; those are masked out afterwards.

#include "main.h"

#include <cstdio>

#include "bsp_print.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "oled.h"

// This module's controller is an SH1106: 132 GDDRAM columns with the panel
// wired to SEG2..SEG129, so everything has to be shifted right by two.
#define OLED_COL_OFFSET 2

#define NUM_PORTS 9
#define LEARN_MS 3000
#define MAX_SHOWN 9

static display::OLED* OLED = nullptr;

static GPIO_TypeDef* const kPorts[NUM_PORTS] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE,
                                                GPIOF, GPIOG, GPIOH, GPIOI};
static const char kPortName[NUM_PORTS] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'};

static uint16_t baseline[NUM_PORTS];  // level while idle
static uint16_t noisy[NUM_PORTS];     // bits that move on their own
static uint16_t hits[NUM_PORTS];      // bits that moved after the learn phase

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  OLED = new display::OLED(&hi2c2, 0x3C, OLED_COL_OFFSET);

  // Clocks only: enabling a GPIO clock does not change any pin's mode, but a
  // port whose clock is off would read back as zeros.
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
}

void RM_RTOS_Default_Task(const void* arg) {
  UNUSED(arg);

  for (int p = 0; p < NUM_PORTS; ++p) {
    baseline[p] = (uint16_t)kPorts[p]->IDR;
    noisy[p] = 0;
    hits[p] = 0;
  }

  const uint32_t start = HAL_GetTick();
  bool learning = true;

  print("=== TypeA pin finder: learning idle state, keep hands off ===\r\n");

  while (true) {
    const uint32_t now = HAL_GetTick();

    for (int p = 0; p < NUM_PORTS; ++p) {
      const uint16_t moved = (uint16_t)kPorts[p]->IDR ^ baseline[p];
      if (learning) {
        noisy[p] |= moved;
      } else {
        const uint16_t real = moved & ~noisy[p];
        if (real & ~hits[p]) {
          for (int b = 0; b < 16; ++b) {
            if ((real & ~hits[p]) & (1u << b)) print("HIT P%c%d\r\n", kPortName[p], b);
          }
          hits[p] |= real;
        }
      }
    }

    if (learning && now - start >= LEARN_MS) {
      learning = false;
      print("=== learn done, press the buttons now ===\r\n");
    }

    OLED->OperateGram(display::PEN_CLEAR);
    if (learning) {
      OLED->Printf(0, 1, "PIN FINDER");
      OLED->Printf(2, 1, "LEARNING %lu", (unsigned long)((LEARN_MS - (now - start)) / 100));
      OLED->Printf(3, 1, "hands off!");
    } else {
      OLED->Printf(0, 1, "PRESS A BUTTON");
      // List every pin that has dropped at least once, three per row.
      int shown = 0;
      char line[24];
      int len = 0;
      int row = 1;
      for (int p = 0; p < NUM_PORTS && shown < MAX_SHOWN; ++p) {
        for (int b = 0; b < 16 && shown < MAX_SHOWN; ++b) {
          if (!(hits[p] & (1u << b))) continue;
          len += snprintf(line + len, sizeof(line) - len, "%c%-2d ", kPortName[p], b);
          ++shown;
          if (shown % 3 == 0) {
            OLED->Printf(row++, 1, line);
            len = 0;
            line[0] = '\0';
          }
        }
      }
      if (len > 0) OLED->Printf(row, 1, line);
      if (shown == 0) OLED->Printf(2, 1, "(nothing yet)");
    }
    OLED->RefreshGram();

    osDelay(5);
  }
}
