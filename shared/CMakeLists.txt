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

# Eigen interface library
add_library(eigen3 INTERFACE)
target_include_directories(eigen3 SYSTEM INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen-3.3.9)

file(GLOB_RECURSE BSP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/bsp/*.cc)
file(GLOB_RECURSE LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/libraries/*.cc)
file(GLOB_RECURSE THIRD_PARTY_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/third_party/*.c)

# Exclude FDCAN from general sources (only for DM_MC_02)
list(FILTER BSP_SOURCES EXCLUDE REGEX ".*/bsp_fdcan\\.cc$")
# Exclude USB stub from general sources (only for DM_MC_02)
list(FILTER BSP_SOURCES EXCLUDE REGEX ".*/bsp_usb_stub\\.cc$")

set(IRM_SOURCES ${BSP_SOURCES} ${LIB_SOURCES} ${THIRD_PARTY_SOURCES})
set(IRM_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp
    ${CMAKE_CURRENT_SOURCE_DIR}/libraries
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

irm_add_board_specific_library(DJI_Board_TypeA_irm
    TARGET DJI_Board_TypeA
    SOURCES ${IRM_SOURCES}
    INCLUDES ${IRM_INCLUDES}
    DEPENDS eigen3)

irm_add_board_specific_library(DJI_Board_TypeC_irm
    TARGET DJI_Board_TypeC
    SOURCES ${IRM_SOURCES}
    INCLUDES ${IRM_INCLUDES}
    DEPENDS eigen3)

irm_add_board_specific_library(DM_MC_01_irm
        TARGET DM_MC_01
        SOURCES ${IRM_SOURCES}
        INCLUDES ${IRM_INCLUDES}
        DEPENDS eigen3)

set(IRM_SOURCES_DM_MC_02 ${IRM_SOURCES})
# Exclude unsupported BSP modules for DM_MC_02
list(FILTER IRM_SOURCES_DM_MC_02 EXCLUDE REGEX ".*/bsp_imu\\.cc$")
list(REMOVE_ITEM IRM_SOURCES_DM_MC_02 ${CMAKE_CURRENT_SOURCE_DIR}/bsp/bsp_imu.cc)
list(FILTER IRM_SOURCES_DM_MC_02 EXCLUDE REGEX ".*/bsp_(laser|os|sdio|ultrasonic|usb)\\.cc$")
# Add USB stub for DM_MC_02 (provides empty implementations)
list(APPEND IRM_SOURCES_DM_MC_02 ${CMAKE_CURRENT_SOURCE_DIR}/bsp/bsp_usb_stub.cc)
# Exclude CAN (use FDCAN instead) for DM_MC_02
list(FILTER IRM_SOURCES_DM_MC_02 EXCLUDE REGEX ".*/bsp_can\\.cc$")
# Exclude unsupported library modules for DM_MC_02 (no default implementations)
# Note: Most modules now work with ARM DSP support
list(FILTER IRM_SOURCES_DM_MC_02 EXCLUDE REGEX ".*/libraries/(chassis|fortress|gimbal|lidar07|pose)\\.cc$")
list(FILTER IRM_SOURCES_DM_MC_02 EXCLUDE REGEX ".*/libraries/(minipc_protocol|oled|dbus|sbus)\\.cc$")
# Add FDCAN BSP for DM_MC_02
list(APPEND IRM_SOURCES_DM_MC_02 ${CMAKE_CURRENT_SOURCE_DIR}/bsp/bsp_fdcan.cc)

irm_add_board_specific_library(DM_MC_02_irm
        TARGET DM_MC_02
        SOURCES ${IRM_SOURCES_DM_MC_02}
        INCLUDES ${IRM_INCLUDES}
        DEPENDS eigen3)
