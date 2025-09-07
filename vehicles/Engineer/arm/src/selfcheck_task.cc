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
#include "selfcheck_task.h"


void self_Check_Task(void* arg){
  UNUSED(arg);

  while(true){
    osDelay(100);
    motor8->connection_flag_ = false;
    motor7->connection_flag_ = false;
    motor6->connection_flag_ = false;
    motor5->connection_flag_ = false;
    motor4->connection_flag_ = false;
    motor3->connection_flag_ = false;
    motor2->connection_flag_ = false;
    motor1->connection_flag_ = false;
    osDelay(100);
    fl_wheel_motor_flag = motor8->connection_flag_;
    fr_wheel_motor_flag = motor7->connection_flag_;
    bl_wheel_motor_flag = motor6->connection_flag_;
    br_wheel_motor_flag = motor5->connection_flag_;
    fl_steer_motor_flag = motor4->connection_flag_;
    fr_steer_motor_flag = motor3->connection_flag_;
    br_steer_motor_flag = motor2->connection_flag_;
    bl_steer_motor_flag = motor1->connection_flag_;
    flag_summary = bl_steer_motor_flag|
                   br_steer_motor_flag<<1|
                   fr_steer_motor_flag<<2|
                   fl_steer_motor_flag<<3|
                   br_wheel_motor_flag<<4|
                   bl_wheel_motor_flag<<5|
                   fr_wheel_motor_flag<<6|
                   fl_wheel_motor_flag<<7;


  }
}