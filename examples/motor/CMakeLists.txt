# ---------------------------------------------------------------------- #
#                                                                        #
#  Copyright (C) 2022                                                    #
#  Illini RoboMaster @ University of Illinois at Urbana-Champaign.       #
#                                                                        #
#  This program is free software: you can redistribute it and/or modify  #
#  it under the terms of the GNU General Public License as published by  #
#  the Free Software Foundation, either version 3 of the License, or     #
#  (at your option) any later version.                                   #
#                                                                        #
#  This program is distributed in the hope that it will be useful,       #
#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
#  GNU General Public License for more details.                          #
#                                                                        #
#  You should have received a copy of the GNU General Public License     #
#  along with this program. If not, see <http://www.gnu.org/licenses/>.  #
#                                                                        #
# ---------------------------------------------------------------------- #

project(example_motor ASM C CXX)

irm_add_arm_executable(${PROJECT_NAME}_m3508
    TARGET DJI_Board_TypeC
    SOURCES m3508.cc)

irm_add_arm_executable(${PROJECT_NAME}_m3508_speed
    TARGET DJI_Board_TypeC
    SOURCES m3508_speed.cc)

irm_add_arm_executable(${PROJECT_NAME}_m3508_position
    TARGET DJI_Board_TypeC
    SOURCES m3508_position.cc)

irm_add_arm_executable(${PROJECT_NAME}_m3508_antijam
    TARGET DJI_Board_TypeA
    SOURCES m3508_antijam.cc)

irm_add_arm_executable(${PROJECT_NAME}_m3508_steering
    TARGET DJI_Board_TypeA
    SOURCES m3508_steering.cc)

irm_add_arm_executable(${PROJECT_NAME}_m6020
    TARGET DJI_Board_TypeC
    SOURCES m6020.cc)

irm_add_arm_executable(${PROJECT_NAME}_m6623
    TARGET DJI_Board_TypeA
    SOURCES m6623.cc)

irm_add_arm_executable(${PROJECT_NAME}_pwm
    TARGET DJI_Board_TypeA
    SOURCES pwm.cc)

irm_add_arm_executable(${PROJECT_NAME}_m2006_antijam
    TARGET DJI_Board_TypeA
    SOURCES m2006_antijam.cc)

irm_add_arm_executable(${PROJECT_NAME}_m2006_reload_modes
    TARGET DJI_Board_TypeC
    SOURCES m2006_reload_modes.cc)

irm_add_arm_executable(${PROJECT_NAME}_m4310_mit
        TARGET DJI_Board_TypeC
        SOURCES m4310_mit.cc)

irm_add_arm_executable(${PROJECT_NAME}_m4310_pos_vel
        TARGET DJI_Board_TypeC
        SOURCES m4310_pos_vel.cc)

irm_add_arm_executable(${PROJECT_NAME}_m4310_vel
        TARGET DJI_Board_TypeC
        SOURCES m4310_vel.cc)

irm_add_arm_executable(${PROJECT_NAME}_m3510
        TARGET DJI_Board_TypeC
        SOURCES m3510.cc)

irm_add_arm_executable(${PROJECT_NAME}_m3510_PID
        TARGET DJI_Board_TypeC
        SOURCES m3510_PID.cc)
irm_add_arm_executable(${PROJECT_NAME}_HERO_SERVO
        TARGET DJI_Board_TypeC
        SOURCES HERO_SERVO.cc)