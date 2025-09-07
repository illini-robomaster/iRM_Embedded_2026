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

#include "utils.h"
#include "cmsis_os.h"

/**
 *  Motor naming convention:
 *  {where} + {axis?} + {translate/rotate}
 *  Example: base_vertical_rotate OR base_vertical OR base (if only one motor on base)
 *  Axis can be omitted if no 2 motors sharing the same position
**/

/* M3508 params start */
const int BASE_TRANSLATE_ID = 0x201;
const GPIO_TypeDef* BASE_TRANSLATE_CALI_GPIO_PORT = GPIOC;
const uint16_t BASE_TRANSLATE_CALI_GPIO_PIN = GPIO_PIN_2;

// M3508 steering params. (translate 3508 motor) */
const float BASE_TRANSLATE_MAX = PI;
const float BASE_TRANSLATE_MIN = -PI;
const float RUN_SPEED = (1 * PI);
const float ALIGN_SPEED = (0.5 * PI);
const float ACCELERATION = (100 * PI);

/* M3508 params end */

/* A1 params start */
const int BASE_YAW_ID = 2;
const int BASE_PITCH_ID = 1;
const int ELBOW_PITCH_ID = 0;
const int A1_CONTROL_DELAY = 2;

UART_HandleTypeDef* A1_UART = &huart1;


// base vertical rotation motor params. (rotate the entire arm around an vertical axis)
const float BASE_VERT_ROTATE_MAX = PI/8;
const float BASE_VERT_ROTATE_MIN = -PI/8;

// base horizontal rotation motor params (rotate the entire arm around a horizontal axis)
const float BASE_HOR_ROTATE_MAX = PI/8;
const float BASE_HOR_ROTATE_MIN = -PI/8;

// elbow rotation motor param (rotate the forearm on a horizontal axis)
const float ELBOW_ROTATE_MAX = PI/8;
const float ELBOW_ROTATE_MIN = -PI/8;

/* A1 params end */

/* M4310 params start */
const int FOREARM_ROTATE_RX_ID = 0x02;
const int FOREARM_ROTATE_TX_ID = 0x01;
const int WRIST_ROTATE_RX_ID = 0x04;
const int WRIST_ROTATE_TX_ID = 0x03;
const int HAND_ROTATE_RX_ID = 0x06;
const int HAND_ROTATE_TX_ID = 0x05;

const float M4310_VEL = 4.0; // magic number, see m4310_mit example

// forearm rotation motor params. (rotate the forearm on its axis)
const float FOREARM_ROTATE_MIN = -PI;
const float FOREARM_ROTATE_MAX = PI;

// wrist rotation motor params. (rotate the hand around an vertical axis)
const float WRIST_ROTATE_MAX = PI;
const float WRIST_ROTATE_MIN = -PI;

// hand rotation motor params (rotate the hand on its axis)
const float HAND_ROTATE_MAX = PI;
const float HAND_ROTATE_MIN = -PI;

/* M4310 params end */
