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
 * @brief CAN (Classic CAN) example for DM_MC_02 board
 * 
 * This example demonstrates CAN communication using the FDCAN peripheral
 * in Classic CAN mode (not FD mode). This provides compatibility with
 * standard CAN 2.0 devices at 1 Mbps.
 * 
 * Hardware setup:
 * - FDCAN1: PD0 (RX), PD1 (TX) - CAN1 interface
 * - FDCAN2: PB5 (RX), PB6 (TX) - CAN2 interface  
 * - FDCAN3: PD12 (RX), PD13 (TX) - CAN3 interface
 * 
 * The example:
 * 1. Sends a counter message every 500ms on CAN ID 0x100
 * 2. Receives messages on CAN ID 0x200 and prints the data
 * 
 * To test: Connect FDCAN1 to another CAN device or use a CAN analyzer
 * 
 * NOTE: Make sure CubeMX FDCAN is configured for Classic CAN at 1 Mbps:
 *   - Nominal Prescaler: 4
 *   - Nominal TimeSeg1: 7
 *   - Nominal TimeSeg2: 2
 *   - Nominal SyncJumpWidth: 1
 */

#include "main.h"
#include "bsp_print.h"
#include "bsp_can_bridge.h"  // This includes FDCAN with CAN alias for MC02
#include "cmsis_os.h"
#include <cstring>

#define CAN_TX_ID 0x100  // ID for transmitting
#define CAN_RX_ID 0x200  // ID for receiving

static bsp::CAN* can1 = nullptr;
static volatile uint32_t rx_count = 0;
// Place rx_data in AXI SRAM to avoid cache issues
__attribute__((section(".RAM_D1"))) static uint8_t rx_data[8] = {0};

/**
 * @brief CAN receive callback - called from interrupt context
 */
void CAN_RxCallback(const uint8_t data[], void* args) {
  UNUSED(args);
  // Copy received data - use memcpy for safety
  memcpy((void*)rx_data, data, 8);
  rx_count++;
}

void RM_RTOS_Init(void) {
  // Use UART7 for debug output (PE7=RX, PE8=TX)
  print_use_uart(&huart7);
  
  // Initialize FDCAN1 in Classic CAN mode
  // On DM_MC_02, bsp::CAN is aliased to bsp::FDCAN
  can1 = new bsp::CAN(&hfdcan1, 0);  // filter_id = 0
  
  // Register callback for receiving messages with ID 0x200
  can1->RegisterRxCallback(CAN_RX_ID, CAN_RxCallback, nullptr);
}

// Debug variables from bsp_fdcan.cc
extern volatile uint32_t fdcan_debug_fill_level;
extern volatile uint32_t fdcan_debug_hal_status;
extern volatile uint32_t fdcan_debug_identifier;
extern volatile uint32_t fdcan_debug_dlc;
extern volatile uint8_t fdcan_debug_data[8];

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  
  uint8_t tx_data[8] = {0};
  uint32_t counter = 0;
  uint32_t last_rx_count = 0;
  
  // Check actual FDCAN clock frequency
  uint32_t fdcan_clk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
  print("=== Clock Info ===\r\n");
  print("FDCAN Clock: %lu Hz (%lu MHz)\r\n", fdcan_clk, fdcan_clk / 1000000);
  print("Expected baud = %lu / (5 * 8) = %lu bps\r\n", fdcan_clk, fdcan_clk / 40);
  print("==================\r\n\r\n");
  
  print("CAN Classic Mode Example for MC02\r\n");
  print("TX ID: 0x%03X, RX ID: 0x%03X\r\n", CAN_TX_ID, CAN_RX_ID);
  print("Baud rate: 1 Mbps, Normal Mode\r\n\r\n");
  
  // Read FDCAN Protocol Status Register to check for errors
  print("=== FDCAN1 Status ===\r\n");
  uint32_t psr = hfdcan1.Instance->PSR;
  print("PSR=0x%08lX\r\n", psr);
  print("  LEC=%lu (Last Error Code: 0=none)\r\n", (psr >> 0) & 0x7);
  print("  ACT=%lu (Activity: 0=sync, 1=idle, 2=rx, 3=tx)\r\n", (psr >> 3) & 0x3);
  print("  EP=%lu (Error Passive)\r\n", (psr >> 5) & 0x1);
  print("  EW=%lu (Error Warning)\r\n", (psr >> 6) & 0x1);
  print("  BO=%lu (Bus Off)\r\n", (psr >> 7) & 0x1);
  print("  RESI=%lu, RBRS=%lu, RFDF=%lu\r\n", 
        (psr >> 11) & 0x1, (psr >> 12) & 0x1, (psr >> 13) & 0x1);
  
  // Error counters
  uint32_t ecr = hfdcan1.Instance->ECR;
  print("ECR=0x%08lX (TEC=%lu, REC=%lu)\r\n", ecr, ecr & 0xFF, (ecr >> 8) & 0x7F);
  
  // Check GPIO configuration for PD0 and PD1
  // MODER: 00=Input, 01=Output, 10=AF, 11=Analog
  // AFRL/AFRH: AF number (should be 9 for FDCAN1)
  uint32_t moder = GPIOD->MODER;
  uint32_t afrl = GPIOD->AFR[0];  // AFR[0] covers pins 0-7
  print("GPIO Config:\r\n");
  print("  MODER=0x%08lX\r\n", moder);
  print("  PD0 mode=%lu (2=AF), PD1 mode=%lu (2=AF)\r\n", 
        (moder >> 0) & 0x3, (moder >> 2) & 0x3);
  print("  AFRL=0x%08lX\r\n", afrl);
  print("  PD0 AF=%lu (9=FDCAN1), PD1 AF=%lu (9=FDCAN1)\r\n",
        (afrl >> 0) & 0xF, (afrl >> 4) & 0xF);
  
  // Check GPIO state
  print("GPIO State: PD0(RX)=%d, PD1(TX)=%d\r\n",
        HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0),
        HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1));
  print("=====================\r\n\r\n");
  
  while (true) {
    // Prepare TX data: 8-byte counter
    tx_data[0] = (counter >> 0) & 0xFF;
    tx_data[1] = (counter >> 8) & 0xFF;
    tx_data[2] = (counter >> 16) & 0xFF;
    tx_data[3] = (counter >> 24) & 0xFF;
    tx_data[4] = 0xAA;  // Pattern bytes
    tx_data[5] = 0x55;
    tx_data[6] = 0xAA;
    tx_data[7] = 0x55;
    
    // Transmit CAN message
    int ret = can1->Transmit(CAN_TX_ID, tx_data, 8);
    
    // Wait a bit for transmission to complete
    osDelay(10);
    
    // Read PSR after transmission attempt
    psr = hfdcan1.Instance->PSR;
    ecr = hfdcan1.Instance->ECR;
    uint32_t txfqs = hfdcan1.Instance->TXFQS;  // TX FIFO/Queue Status
    
    if (ret > 0) {
      print("TX[0x%03X]: cnt=%lu\r\n", CAN_TX_ID, counter);
    } else {
      print("TX failed! ret=%d\r\n", ret);
    }
    
    // Show PSR after TX attempt - this is key!
    print("  PSR: LEC=%lu ACT=%lu EP=%lu EW=%lu BO=%lu\r\n",
          (psr >> 0) & 0x7, (psr >> 3) & 0x3,
          (psr >> 5) & 0x1, (psr >> 6) & 0x1, (psr >> 7) & 0x1);
    print("  ECR: TEC=%lu REC=%lu | TXFQS=0x%08lX (free=%lu)\r\n",
          ecr & 0xFF, (ecr >> 8) & 0x7F,
          txfqs, txfqs & 0x3F);
    
    // Check if we received any messages
    if (rx_count != last_rx_count) {
      print("RX[0x%03X]: %02X %02X %02X %02X %02X %02X %02X %02X (total=%lu)\r\n",
            CAN_RX_ID,
            rx_data[0], rx_data[1], rx_data[2], rx_data[3],
            rx_data[4], rx_data[5], rx_data[6], rx_data[7],
            rx_count);
      last_rx_count = rx_count;
    }
    
    counter++;
    osDelay(500);
  }
}
