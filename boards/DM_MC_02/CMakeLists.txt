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

get_filename_component(CURRENT_DIR_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${CURRENT_DIR_NAME} C ASM)

set(MCU_LINE STM32H723xx)
set(MCU_ARCH cortex-m7)
set(MCU_FLOAT_ABI hard)
set(MCU_FPU fpv5-d16)
# Use custom linker script with heap in AXI SRAM for DMA compatibility
set(MCU_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/cmake/stm32h723vgtx_flash_custom.ld)
set(MCU_OPTIONS -mcpu=${MCU_ARCH} -mthumb -mthumb-interwork
        -mfpu=${MCU_FPU} -mfloat-abi=${MCU_FLOAT_ABI})

######################################################
# ----- create board specific interface library -----#
######################################################
add_library(${PROJECT_NAME}_interface INTERFACE)
target_compile_definitions(${PROJECT_NAME}_interface INTERFACE ${MCU_LINE}
        ARM_MATH_CM7
        BOARD_DM_MC_02
        BOARD_HAS_FDCAN
        BOARD_HAS_OCTOSPI
        USE_HAL_DRIVER)
target_compile_options(${PROJECT_NAME}_interface INTERFACE ${MCU_OPTIONS})
target_link_options(${PROJECT_NAME}_interface INTERFACE
        ${MCU_OPTIONS} -T${MCU_LINKER_SCRIPT})

target_link_libraries(${PROJECT_NAME}_interface INTERFACE board_interface)
target_include_directories(${PROJECT_NAME}_interface INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App
        ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/Target
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/DSP/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include
        ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
        ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F)

#####################################################
# ----- create board specific arm dsp library ----- #
#####################################################

file(GLOB_RECURSE DSP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/DSP/Source/*.c)
add_library(${PROJECT_NAME}_libDSP ${DSP_SOURCES})
target_compile_options(${PROJECT_NAME}_libDSP PRIVATE -Wno-strict-aliasing -Wno-sign-compare -Wno-unused-parameter)
target_compile_definitions(${PROJECT_NAME}_libDSP PRIVATE __FPU_PRESENT ARM_MATH_CM7)
target_link_libraries(${PROJECT_NAME}_libDSP PRIVATE ${PROJECT_NAME}_interface)

###################################################
# ----- create board specific driver library -----#
###################################################
file(GLOB_RECURSE CORE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/*.c)
file(GLOB_RECURSE USB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/*.c)
file(GLOB_RECURSE DRIVER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/*.c)
file(GLOB_RECURSE MIDDLEWARE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/*.c)
file(GLOB_RECURSE STARTUP_ASM ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32h723xx.s)

# Exclude CMSIS_RTOS V1 cmsis_os.c - we use V2 (cmsis_os2.c) instead
list(FILTER MIDDLEWARE_SOURCES EXCLUDE REGEX "CMSIS_RTOS/cmsis_os\\.c$")

# Exclude CubeMX-generated usbd_desc.c - use our user-maintained version instead
# (fixes CubeMX bug where USBD_LPM_ENABLED=1 but BOS descriptor is missing)
list(FILTER USB_SOURCES EXCLUDE REGEX "USB_DEVICE/App/usbd_desc\\.c$")
set(USER_USB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/User/user_usbd_desc.c)

add_library(${PROJECT_NAME} OBJECT
        ${CORE_SOURCES} ${USB_SOURCES} ${USER_USB_SOURCES}
        ${DRIVER_SOURCES} ${MIDDLEWARE_SOURCES} ${STARTUP_ASM})
# ST libraries throws those warnings
target_compile_options(${PROJECT_NAME} PRIVATE
        -Wno-maybe-uninitialized -Wno-unused-parameter -Wno-sign-compare -Wno-int-conversion)
# link against board specific arm dsp library and interface
target_link_libraries(${PROJECT_NAME}
        PRIVATE ${PROJECT_NAME}_libDSP
        PUBLIC ${PROJECT_NAME}_interface)
