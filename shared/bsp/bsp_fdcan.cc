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

#include "bsp_fdcan.h"

#include "bsp_error_handler.h"
#include "cmsis_os.h"

namespace bsp {

    std::map<FDCAN_HandleTypeDef*, FDCAN*> FDCAN::ptr_map;

    /**
     * @brief find instantiated FDCAN line
     *
     * @param hfdcan  HAL FDCAN handle
     *
     * @return FDCAN instance if found, otherwise NULL
     */
    FDCAN* FDCAN::FindInstance(FDCAN_HandleTypeDef* hfdcan) {
        const auto it = ptr_map.find(hfdcan);
        if (it == ptr_map.end())  return nullptr;

        return it->second;
    }

    /**
     * @brief check if any associated FDCAN instance is instantiated or not
     *
     * @param hfdcan  HAL FDCAN handle
     *
     * @return true if found, otherwise false
     */
    bool FDCAN::HandleExists(FDCAN_HandleTypeDef* hfdcan) {
        return FindInstance(hfdcan) != nullptr;
    }

    /**
     * @brief callback handler for FDCAN rx feedback data
     *
     * @param hfdcan      HAL FDCAN handle
     * @param RxFifo0ITs  FIFO 0 interrupt flags
     */
    void FDCAN::RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
        FDCAN* fdcan = FindInstance(hfdcan);
        if (!fdcan)  return;

        if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
            fdcan->RxCallback();
        }
    }

    FDCAN::FDCAN(FDCAN_HandleTypeDef* hfdcan, uint32_t filter_id): hfdcan_(hfdcan) {
        RM_ASSERT_FALSE(HandleExists(hfdcan), "Repeated FDCAN initialization");
        
        ConfigureFilter(filter_id);
        
        // Activate FIFO 0 new message notification
        RM_ASSERT_HAL_OK(HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0),
                         "Cannot activate FDCAN rx message pending notification");
        
        // Start FDCAN
        RM_ASSERT_HAL_OK(HAL_FDCAN_Start(hfdcan), "Cannot start FDCAN");

        // Save FDCAN instance as global pointer
        ptr_map[hfdcan] = this;
    }

    int FDCAN::RegisterRxCallback(uint32_t std_id, fdcan_rx_callback_t callback, void* args) {
        if (callback_count_ >= MAX_FDCAN_DEVICES) return -1;

        rx_args_[callback_count_] = args;
        rx_callbacks_[callback_count_] = callback;
        id_to_index_[std_id] = callback_count_;
        callback_count_++;

        return 0;
    }

    int FDCAN::Transmit(uint16_t id, const uint8_t data[], uint32_t length) {
        if (length > 8 || length == 0)
            return -1;

        // Map length to DLC code
        uint32_t dlc_code;
        if (length <= 8) {
            dlc_code = length << 16;  // For classic CAN, DLC is just the length
        } else {
            return -1;
        }

        FDCAN_TxHeaderTypeDef header = {
            .Identifier = id,
            .IdType = FDCAN_STANDARD_ID,
            .TxFrameType = FDCAN_DATA_FRAME,
            .DataLength = dlc_code,
            .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
            .BitRateSwitch = FDCAN_BRS_OFF,
            .FDFormat = FDCAN_CLASSIC_CAN,
            .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
            .MessageMarker = 0,
        };

        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &header, (uint8_t*)data) != HAL_OK)
            return -1;

        return length;
    }

    void FDCAN::RxCallback() {
        FDCAN_RxHeaderTypeDef header;
        uint8_t data[MAX_FDCAN_DATA_SIZE];
        
        if (HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
            return;
            
        uint16_t callback_id = header.Identifier;
        const auto it = id_to_index_.find(callback_id);
        if (it == id_to_index_.end())
            return;
            
        callback_id = it->second;
        
        // Find corresponding callback
        if (rx_callbacks_[callback_id])
            rx_callbacks_[callback_id](data, rx_args_[callback_id]);
    }

    void FDCAN::ConfigureFilter(uint32_t filter_id) {
        FDCAN_FilterTypeDef filter_config;
        
        /* Configure filter to accept all messages */
        filter_config.IdType = FDCAN_STANDARD_ID;
        filter_config.FilterIndex = filter_id;
        filter_config.FilterType = FDCAN_FILTER_MASK;
        filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        filter_config.FilterID1 = 0x0000;  // Filter ID
        filter_config.FilterID2 = 0x0000;  // Filter Mask (0 = don't care)

        RM_EXPECT_HAL_OK(HAL_FDCAN_ConfigFilter(hfdcan_, &filter_config),
                         "FDCAN filter configuration failed.");
    }

} /* namespace bsp */

/* HAL callback override - called from interrupt context */
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
    bsp::FDCAN::RxFifo0Callback(hfdcan, RxFifo0ITs);
}
