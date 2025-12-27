# ---------------------------------------------------------------------- #
#                                                                        #
#  Copyright (C) 2024                                                    #
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

project(example_minipc ASM C CXX)

irm_add_arm_executable(${PROJECT_NAME}
    TARGET DJI_Board_TypeA
    SOURCES typeA.cc)

irm_add_arm_executable(${PROJECT_NAME}_stresstesttypec
    TARGET DJI_Board_TypeC
    SOURCES StressTestTypeC.cc)

irm_add_arm_executable(${PROJECT_NAME}_pingpongtest
    TARGET DJI_Board_TypeC
    SOURCES PingpongTest.cc)

irm_add_arm_executable(${PROJECT_NAME}_motor
    TARGET DJI_Board_TypeA
    SOURCES MotorTest.cc)

irm_add_arm_executable(${PROJECT_NAME}_latency
    TARGET DJI_Board_TypeC
    SOURCES LatencyTest.cc)

irm_add_arm_executable(${PROJECT_NAME}_minipccontroller
    TARGET DJI_Board_TypeC
    SOURCES MiniPCController.cc)

irm_add_arm_executable(${PROJECT_NAME}_chassis
        TARGET DJI_Board_TypeC
        SOURCES ChassisTest.cc)
