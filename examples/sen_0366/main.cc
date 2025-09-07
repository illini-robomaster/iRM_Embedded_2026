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


#include "bsp_uart.h"
#include "main.h"
#include "sen_0366_dist.h"
#include "cmsis_os.h"
#include "bsp_print.h"
#include "bsp_gpio.h"

static distance::SEN_0366_DIST* sensor = nullptr;



void RM_RTOS_Init(void) {
  print_use_uart(&huart6);
  sensor = distance::SEN_0366_DIST::init(&huart1,0x80);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  print("Begin\r\n");
  while (!sensor->begin()){
    print("sensor_initializing: connection_flag_: %d\r\n", sensor->connection_flag_);
    osDelay(50);
  }
  sensor->setResolution(distance::RESOLUTION_0_1_MM);

  sensor->setMeasureRange(distance::RANGE_80_M);

  sensor->continuousMeasure();
  while (true) {
    osDelay(100);
//    sensor->singleMeasure();
    sensor->readValue();
    print("distance is: %f m. \r\n", sensor->distance_);

//    for (int i = 0; i < 10; i++){
//      print("%x ",sensor->buff_[i]);
//    }
//    print ("\r\n");
  }
}

