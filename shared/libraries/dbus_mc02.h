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

/**
 * @file dbus_mc02.h
 * @brief DBUS 遥控器驱动 - 专门为 DM_MC_02 (STM32H7) 优化
 * 
 * STM32H7 DMA 注意事项：
 * - DMA1/DMA2 只能访问特定内存区域
 * - D2 SRAM (0x30000000-0x30007FFF) 是 DMA 可访问的
 * - 此驱动将接收缓冲区放在 D2 SRAM 中
 * 
 * 使用方法：
 * ```cpp
 * #include "dbus_mc02.h"
 * remote::DBUS_MC02* dbus = new remote::DBUS_MC02(&huart5);
 * // 然后像使用普通 DBUS 一样使用
 * ```
 */

#ifndef BSP_DBUS_MC02_H_
#define BSP_DBUS_MC02_H_

#ifdef BOARD_DM_MC_02

#include "dbus.h"
#include <cstring>

namespace remote {

/**
 * @brief DBUS 遥控器驱动 - MC02 版本
 * 
 * 与标准 DBUS 类的主要区别：
 * 1. 使用静态分配在 D2 SRAM 的缓冲区
 * 2. 自定义 DMA 初始化确保内存可访问
 */
class DBUS_MC02 : public DBUS {
 public:
    /**
     * @brief 构造函数
     * @param huart UART 句柄指针 (应该是 &huart5)
     */
    DBUS_MC02(UART_HandleTypeDef* huart);
    
    /**
     * @brief 获取接收缓冲区地址 (用于调试)
     * @return 缓冲区地址
     */
    static uint32_t GetBufferAddress() { return (uint32_t)rx_buffer_d2_; }
    
 private:
    // 静态缓冲区放在 D2 SRAM (32 字节足够存储 18 字节 DBUS 帧)
    // __attribute__((section(".RAM_D2"))) 确保放在正确的内存区域
    static uint8_t rx_buffer_d2_[2][32] __attribute__((section(".RAM_D2")));
};

}  // namespace remote

#endif  // BOARD_DM_MC_02
#endif  // BSP_DBUS_MC02_H_
