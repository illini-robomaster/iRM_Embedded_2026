/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2025 RoboMaster.                                          *
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

#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "can.h"

/**
 * @brief Send a raw CAN frame using HAL directly
 * @param can_handle Pointer to CAN handle (e.g., &hcan1)
 * @param id CAN ID (11-bit standard ID)
 * @param data Data buffer to send (up to 8 bytes)
 * @param length Data length (0-8 bytes)
 * @return HAL status (HAL_OK if successful)
 */
HAL_StatusTypeDef SendRawCANFrame(CAN_HandleTypeDef* can_handle, uint32_t id, uint8_t* data, uint8_t length) {
  // Validate parameters
  if (length > 8) {
    print("Error: CAN data length exceeds 8 bytes\r\n");
    return HAL_ERROR;
  }
  
  if (id > 0x7FF) {
    print("Error: Standard CAN ID exceeds 11-bit range (0x7FF)\r\n");
    return HAL_ERROR;
  }

  // Configure CAN transmission header
  CAN_TxHeaderTypeDef tx_header;
  tx_header.StdId = id;                    // Standard ID (11-bit)
  tx_header.ExtId = 0x0;                   // Extended ID (not used for standard frames)
  tx_header.IDE = CAN_ID_STD;              // Use standard ID format
  tx_header.RTR = CAN_RTR_DATA;            // Data frame (not remote frame)
  tx_header.DLC = length;                  // Data length code
  tx_header.TransmitGlobalTime = DISABLE;  // Don't include timestamp

  uint32_t tx_mailbox;
  HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(can_handle, &tx_header, data, &tx_mailbox);
  
  if (status != HAL_OK) {
    print("Error: Failed to add CAN message to TX mailbox, status: %d\r\n", status);
    return status;
  }

  // Wait for transmission to complete
  while (HAL_CAN_IsTxMessagePending(can_handle, tx_mailbox)) {
    osDelay(1); // Small delay to prevent busy waiting
  }

  print("CAN Frame sent successfully - ID: 0x%03X, Length: %d, Mailbox: %d\r\n", 
        (unsigned int)id, length, (unsigned int)tx_mailbox);
  
  return HAL_OK;
}

/**
 * @brief Initialize CAN and start the peripheral
 */
void InitializeRawCAN() {
  // Initialize CAN1
  MX_CAN1_Init();
  
  // Start CAN peripheral
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    print("Error: Failed to start CAN1\r\n");
    Error_Handler();
  }
  
  print("CAN1 initialized and started successfully\r\n");
  print("Ready to send raw CAN frames on hcan1\r\n");
}

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  InitializeRawCAN();
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  
  print("Raw CAN Transmission Example\r\n");
  print("Sending various CAN frames every 2 seconds...\r\n");
  
  uint32_t counter = 0;
  
  while (true) {
    // Example 1: Send motor command (similar to DJI motor protocol)
    uint8_t motor_data[8] = {0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    SendRawCANFrame(&hcan1, 0x200, motor_data, 8);
    
  
    osDelay(2);
    

    print("=== Cycle %lu completed ===\r\n", counter);
  }
}