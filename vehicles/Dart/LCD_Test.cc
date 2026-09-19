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

// Menu demo for the 4-button OLED module on DJI_Board_TypeA.
//
// The four buttons sit on PC0..PC3, but CubeMX splits them across two very
// different modes, so they cannot all be read the same way:
//
//   up    PC0  ADC1_IN10, GPIO_MODE_ANALOG  -> read as an ADC code
//   down  PC1  ADC1_IN11, GPIO_MODE_ANALOG  -> read as an ADC code
//   #     PC2  L1_Pin, GPIO_MODE_INPUT + pull-up -> HAL_GPIO_ReadPin
//   *     PC3  M1_Pin, GPIO_MODE_INPUT + pull-up -> HAL_GPIO_ReadPin
//
// GPIO_MODE_ANALOG disables the digital input buffer, so IDR reads 0 for PC0
// and PC1 no matter what the button does.  They have to come from the ADC.
// Each button is a switch to ground with a pull-up: idle reads ~4095 / high,
// held reads ~0 / low.

#include "main.h"

#include "adc.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "oled.h"

// This module's controller is an SH1106: 132 GDDRAM columns with the panel
// wired to SEG2..SEG129, so everything has to be shifted right by two.
#define OLED_COL_OFFSET 2

#define ADC_RANK_COUNT 4
#define ADC_INDEX_UP 1    // ADC_CHANNEL_10 / PC0 sits at rank 2
#define ADC_INDEX_DOWN 2  // ADC_CHANNEL_11 / PC1 sits at rank 3

// Idle is ~4095 and held is ~0, so anything near mid-scale is a safe split.
#define ADC_PRESS_LEVEL 2048

#define DEBOUNCE_MS 200

// Fills the whole screen white then black on boot.  Anything that does not
// follow along is outside the 128 columns this driver can address, which is
// what a 132-column SH1106 controller looks like when it is driven as an
// SSD1306.  Set to 0 once the panel has been identified.
#define SCREEN_SELFTEST 0

static display::OLED* OLED = nullptr;

// The ADC DMA stream is configured for HALFWORD transfers in HAL_ADC_MspInit,
// so this has to be 16-bit wide even though HAL_ADC_Start_DMA takes uint32_t*.
static uint16_t adc_buf[ADC_RANK_COUNT];

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  OLED = new display::OLED(&hi2c2, 0x3C, OLED_COL_OFFSET);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_RANK_COUNT);
  // PC2 / PC3 are already INPUT with a pull-up from MX_GPIO_Init, and PC0 /
  // PC1 belong to the ADC, so nothing here should touch GPIOC's config.
}

// ── input ─────────────────────────────────────────────────────────────────

static bool KeyUp(void) { return adc_buf[ADC_INDEX_UP] < ADC_PRESS_LEVEL; }
static bool KeyDown(void) { return adc_buf[ADC_INDEX_DOWN] < ADC_PRESS_LEVEL; }
static bool KeyHash(void) { return HAL_GPIO_ReadPin(L1_GPIO_Port, L1_Pin) == GPIO_PIN_RESET; }
static bool KeyStar(void) { return HAL_GPIO_ReadPin(M1_GPIO_Port, M1_Pin) == GPIO_PIN_RESET; }

typedef struct {
  bool prev;
  uint32_t last_ms;
} KeyState;

// Debounced rising edge: true once per press, no more often than DEBOUNCE_MS.
static bool JustPressed(bool now_down, KeyState* s, uint32_t now_ms) {
  bool fired = false;
  if (now_down && !s->prev && (now_ms - s->last_ms >= DEBOUNCE_MS)) {
    s->last_ms = now_ms;
    fired = true;
  }
  s->prev = now_down;
  return fired;
}

// ── content ───────────────────────────────────────────────────────────────

static const uint8_t NUM_ITEMS = 2;
static const char* ITEM_LABEL[] = {"Joey is GOAT", "Daniel is Qu"};
static const char* ITEM_DETAIL[] = {"GOAT", "Qu"};

// ── rendering ─────────────────────────────────────────────────────────────

// Invert all pixels in text row `text_row` (0-based, each row is 12 px tall)
// over the full screen width.  Called *after* Printf so the normal
// white-on-black text is flipped to black-on-white (highlight effect).
static void InvertTextRow(uint8_t text_row) {
  const uint8_t y_start = text_row * 12;
  const uint8_t y_end = y_start + 11;
  for (uint8_t y = y_start; y <= y_end; ++y) {
    OLED->DrawLine(0, y, 127, y, display::PEN_INVERSION);
  }
}

static void DrawMenu(uint8_t cursor) {
  OLED->OperateGram(display::PEN_CLEAR);
  OLED->Printf(0, 2, "--- MENU ---");
  for (uint8_t i = 0; i < NUM_ITEMS; ++i) {
    OLED->Printf(i + 1, 2, ITEM_LABEL[i]);  // items at row 1 and 2
  }
  OLED->Printf(4, 2, "UP/DN  #:OK");
  // Highlight selected row: items live at text rows (cursor+1)
  InvertTextRow(cursor + 1);
  OLED->RefreshGram();
}

static void DrawDetail(uint8_t item) {
  OLED->OperateGram(display::PEN_CLEAR);
  OLED->Printf(0, 2, ITEM_LABEL[item]);
  OLED->Printf(2, 2, ITEM_DETAIL[item]);
  OLED->Printf(4, 2, "*:Back");
  OLED->RefreshGram();
}

// ── task ──────────────────────────────────────────────────────────────────

void RM_RTOS_Default_Task(const void* arg) {
  UNUSED(arg);

#if SCREEN_SELFTEST
  OLED->OperateGram(display::PEN_WRITE);
  OLED->RefreshGram();
  osDelay(2000);
  OLED->OperateGram(display::PEN_CLEAR);
  OLED->RefreshGram();
  osDelay(2000);
#endif

  OLED->ShowRMLOGO();
  osDelay(2000);
  OLED->ShowIlliniRMLOGO();
  osDelay(2000);

  typedef enum { STATE_MENU, STATE_DETAIL } AppState;

  AppState state = STATE_MENU;
  uint8_t cursor = 0;
  bool need_redraw = true;

  KeyState up = {false, 0}, down = {false, 0}, hash = {false, 0}, star = {false, 0};

  while (true) {
    const uint32_t now = HAL_GetTick();

    if (JustPressed(KeyUp(), &up, now) && state == STATE_MENU) {
      cursor = (cursor == 0) ? (NUM_ITEMS - 1) : cursor - 1;
      need_redraw = true;
    }
    if (JustPressed(KeyDown(), &down, now) && state == STATE_MENU) {
      cursor = (cursor + 1) % NUM_ITEMS;
      need_redraw = true;
    }
    if (JustPressed(KeyHash(), &hash, now) && state == STATE_MENU) {
      state = STATE_DETAIL;
      need_redraw = true;
    }
    if (JustPressed(KeyStar(), &star, now) && state == STATE_DETAIL) {
      state = STATE_MENU;
      need_redraw = true;
    }

    if (need_redraw) {
      if (state == STATE_MENU)
        DrawMenu(cursor);
      else
        DrawDetail(cursor);
      need_redraw = false;
    }

    osDelay(10);
  }
}
