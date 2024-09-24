
get_filename_component(CURRENT_DIR_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${CURRENT_DIR_NAME} C ASM)

set(MCU_LINE STM32H723xx)
set(MCU_ARCH cortex-m7)
set(MCU_FLOAT_ABI hard)
set(MCU_FPU fpv5-d16)
set(MCU_LINKER_SCRIPT STM32H723VGTx_FLASH.ld)
set(MCU_OPTIONS -mcpu=${MCU_ARCH} -mthumb -mthumb-interwork
        -mfpu=${MCU_FPU} -mfloat-abi=${MCU_FLOAT_ABI})

######################################################
# ----- create board specific interface library -----#
######################################################
add_library(${PROJECT_NAME}_interface INTERFACE)
target_compile_definitions(${PROJECT_NAME}_interface INTERFACE ${MCU_LINE}
        ARM_MATH_CM4
        BOARD_HAS_UART_DMA_DOUBLE_BUFFER)
target_compile_options(${PROJECT_NAME}_interface INTERFACE ${MCU_OPTIONS})
target_link_options(${PROJECT_NAME}_interface INTERFACE
        ${MCU_OPTIONS} -T${CMAKE_CURRENT_SOURCE_DIR}/${MCU_LINKER_SCRIPT})

target_link_libraries(${PROJECT_NAME}_interface INTERFACE board_interface)
target_include_directories(${PROJECT_NAME}_interface INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
        #    ${CMAKE_CURRENT_SOURCE_DIR}/FATFS/Target
        #    ${CMAKE_CURRENT_SOURCE_DIR}/FATFS/App
        ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App
        ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/Target
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/DSP/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FatFs/src
        ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include
        ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
        ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F)

#####################################################
# ----- create board specific arm dsp library ----- #
#####################################################

file(GLOB_RECURSE DSP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/DSP/Source/*.c)
add_library(${PROJECT_NAME}_libDSP ${DSP_SOURCES})
target_compile_options(${PROJECT_NAME}_libDSP PRIVATE -Wno-strict-aliasing)
target_compile_definitions(${PROJECT_NAME}_libDSP PRIVATE __FPU_PRESENT)
target_link_libraries(${PROJECT_NAME}_libDSP PRIVATE ${PROJECT_NAME}_interface)

###################################################
# ----- create board specific driver library -----#
###################################################
file(GLOB_RECURSE CORE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/*.c)
#file(GLOB_RECURSE FATFS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/FATFS/*.c)
file(GLOB_RECURSE USB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/*.c)
file(GLOB_RECURSE DRIVER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/*.c)
file(GLOB_RECURSE MIDDLEWARE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/*.c)
file(GLOB_RECURSE STARTUP_ASM ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32h723xx.s)

add_library(${PROJECT_NAME} OBJECT
        ${CORE_SOURCES} ${FATFS_SOURCES} ${USB_SOURCES}
        ${DRIVER_SOURCES} ${MIDDLEWARE_SOURCES} ${STARTUP_ASM})
# ST libraries throws those warnings
target_compile_options(${PROJECT_NAME} PRIVATE
        -Wno-maybe-uninitialized -Wno-unused-parameter -Wno-sign-compare -Wno-int-conversion)
# link against board specific arm dsp library and interface
target_link_libraries(${PROJECT_NAME}
        PRIVATE ${PROJECT_NAME}_libDSP
        PUBLIC ${PROJECT_NAME}_interface)
