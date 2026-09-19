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

#include "i2c.h"

// the resolution of oled   128*64
#define MAX_COLUMN 128
#define MAX_ROW 64

#define X_WIDTH MAX_COLUMN
#define Y_WIDTH MAX_ROW

#define OLED_CMD 0x00
#define OLED_DATA 0x01

#define CHAR_SIZE_WIDTH 6
#define CHAR_SIZE_HIGHT 12

namespace display {

typedef enum {
  PEN_CLEAR = 0x00,
  PEN_WRITE = 0x01,
  PEN_INVERSION = 0x02,
} pen_typedef;

class OLED {
 public:
  /**
   * @param[in]      col_offset: first GDDRAM column that the panel actually
   *                 shows.  0 for a true SSD1306 (128 columns).  SH1106 parts
   *                 have 132 columns and wire the panel to SEG2..SEG129, so
   *                 they need 2 here -- otherwise the leftmost two pixel
   *                 columns fall off the panel and the two columns that are
   *                 never written show up as a noise stripe at the edge.
   */
  OLED(I2C_HandleTypeDef* hi2c, uint16_t OLED_i2c_addr, uint8_t col_offset = 0);

  bool IsReady();

  /**
   * @brief          initialize the oled device
   * @param[in]      none
   * @retval         none
   */
  void Init(void);

  /**
   * @brief          turn on OLED display
   * @param[in]      none
   * @retval         none
   */
  void DisplayOn(void);

  /**
   * @brief          turn off OLED display
   * @param[in]      none
   * @retval         none
   */
  void DisplayOff(void);

  /**
  * @brief          operate the graphic ram(size: 128*8 char)
  * @param[in]      pen: the type of operate.
                    PEN_CLEAR: set ram to 0x00
                    PEN_WRITE: set ram to 0xff
                    PEN_INVERSION: bit inversion
  * @retval         none
  */
  void OperateGram(pen_typedef pen);

  /**
   * @brief          cursor set to (x,y) point
   * @param[in]      x:X-axis, from 0 to 127
   * @param[in]      y:Y-axis, from 0 to 7
   * @retval         none
   */
  void SetPos(uint8_t x, uint8_t y);

  /**
  * @brief          draw one bit of graphic raw, operate one point of screan(128*64)
  * @param[in]      x: x-axis, [0, X_WIDTH-1]
  * @param[in]      y: y-axis, [0, Y_WIDTH-1]
  * @param[in]      pen: type of operation,
                        PEN_CLEAR: set (x,y) to 0
                        PEN_WRITE: set (x,y) to 1
                        PEN_INVERSION: (x,y) value inversion
  * @retval         none
  */
  void DrawPoint(int8_t x, int8_t y, pen_typedef pen);

  /**
   * @brief          draw a line from (x1, y1) to (x2, y2)
   * @param[in]      x1: the start point of line
   * @param[in]      y1: the start point of line
   * @param[in]      x2: the end point of line
   * @param[in]      y2: the end point of line
   * @param[in]      pen: type of operation,PEN_CLEAR,PEN_WRITE,PEN_INVERSION.
   * @retval         none
   */
  void DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, pen_typedef pen);

  /**
   * @brief          show a character
   * @param[in]      row: start row of character
   * @param[in]      col: start column of character
   * @param[in]      chr: the character ready to show
   * @retval         none
   */
  void ShowChar(uint8_t row, uint8_t col, uint8_t chr);

  void ShowBlock(uint8_t row, uint8_t col, bool correct);

  /**
   * @brief          show a character string
   * @param[in]      row: row of character string begin
   * @param[in]      col: column of character string begin
   * @param[in]      chr: the pointer to character string
   * @retval         none
   */
  void ShowString(uint8_t row, uint8_t col, uint8_t* chr);

  /**
   * @brief          formatted output in oled 128*64
   * @param[in]      row: row of character string begin, 0 <= row <= 4;
   * @param[in]      col: column of character string begin, 0 <= col <= 20;
   * @param          *fmt: the pointer to format character string
   * @note           if the character length is more than one row at a time, the extra characters
   * will be truncated
   * @retval         none
   */
  void Printf(uint8_t row, uint8_t col, const char* fmt, ...);

  /**
   * @brief          send the data of gram to oled sreen
   * @param[in]      none
   * @retval         none
   */
  void RefreshGram(void);

  /**
   * @brief          show the logo of RoboMaster
   * @param[in]      none
   * @retval         none
   */
  void ShowRMLOGO(void);
  void ShowIlliniRMLOGO(void);
  void DrawCat(void);

 private:
  /**
   * @brief          write data/command to OLED, if you use spi, please rewrite the function
   * @param[in]      dat: the data ready to write
   * @param[in]      cmd: OLED_CMD means command; OLED_DATA means data
   * @retval         none
   */
  void WriteByte(uint8_t dat, uint8_t cmd);

  void Cat(unsigned char graph[128][8]);

  unsigned long CatCount_ = 0;

  I2C_HandleTypeDef* hi2c_;
  uint16_t OLED_i2c_addr_;
  uint8_t col_offset_;
  uint8_t OLED_GRAM_[128][8];
};

}  // namespace display
