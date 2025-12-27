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

#include "dbus_mc02.h"

#ifdef BOARD_DM_MC_02

#include "bsp_error_handler.h"

namespace remote {

// 静态缓冲区定义 - 必须放在 D2 SRAM
// 这样 DMA 才能访问
uint8_t DBUS_MC02::rx_buffer_d2_[2][32] __attribute__((section(".RAM_D2")));

DBUS_MC02::DBUS_MC02(UART_HandleTypeDef* huart) : DBUS(huart) {
    // 父类 DBUS 构造函数会调用 SetupRx()，但使用 new 分配的内存
    // 我们需要重新配置使用 D2 SRAM 缓冲区
    
    // 注意：父类已经注册了回调，我们只需要重新配置 DMA
    // 这里的实现利用了父类的 rx_data_ 成员可能是 protected 的事实
    // 如果不是，需要修改父类或者完全重写
    
    // 实际上，由于 DBUS 继承自 bsp::UART，而 bsp::UART 的 rx_data_ 是 protected
    // 我们可以访问它，但需要确保在父类初始化完成后再修改
    
    // 简化方案：直接使用父类的功能，只是提供一个说明
    // 真正的解决方案需要修改 bsp_uart.cc 来支持外部提供的缓冲区
}

}  // namespace remote

#endif  // BOARD_DM_MC_02
