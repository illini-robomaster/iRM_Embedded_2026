/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2023 RoboMaster.                                          *
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

#include "bsp_can.h"

#include "bsp_error_handler.h"
#include "cmsis_os.h"

namespace bsp {

    // A normal 1 Mbps frame completes in well under 1 ms. Never let a missing
    // ACK / bus-off leave a motor or protocol task spinning forever.
    constexpr uint32_t CAN_TX_COMPLETION_TIMEOUT_MS = 2;

    std::map<CAN_HandleTypeDef*, CAN*> CAN::ptr_map;

    /**
     * @brief find instantiated can line
     *
     * @param hcan  HAL can handle
     *
     * @return can instance if found, otherwise NULL
     */
    CAN* CAN::FindInstance(CAN_HandleTypeDef* hcan) {
        const auto it = ptr_map.find(hcan);
        if (it == ptr_map.end())  return nullptr;

        return it->second;
    }

    /**
     * @brief check if any associated CAN instance is instantiated or not
     *
     * @param hcan  HAL can handle
     *
     * @return true if found, otherwise false
     */
    bool CAN::HandleExists(CAN_HandleTypeDef* hcan) {
        return FindInstance(hcan) != nullptr;
    }

    /**
     * @brief callback handler for CAN rx feedback data
     *
     * @param hcan  HAL can handle
     */
    void CAN::RxFIFO0MessagePendingCallback(CAN_HandleTypeDef* hcan) {
        CAN* can = FindInstance(hcan);
        if (!can)  return;

        can->RxCallback();
    }

    CAN::CAN(CAN_HandleTypeDef* hcan, bool is_master): hcan_(hcan){
        RM_ASSERT_FALSE(HandleExists(hcan), "Repeated CAN initialization");
        ConfigureFilter(is_master);
        // activate rx interrupt
        RM_ASSERT_HAL_OK(HAL_CAN_RegisterCallback(hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,
                                                  RxFIFO0MessagePendingCallback),
                         "Cannot register CAN rx callback");
        RM_ASSERT_HAL_OK(HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING),
                         "Cannot activate CAN rx message pending notification");
        RM_ASSERT_HAL_OK(HAL_CAN_Start(hcan), "Cannot start CAN");

        // save can instance as global pointer
        ptr_map[hcan] = this;
    }

    int CAN::RegisterRxCallback(uint32_t std_id, can_rx_callback_t callback, void* args) {
        if (std_id > 0x7FF) return -1;

        int result = 0;
        // Registration happens after HAL_CAN_Start, so protect the map and the
        // parallel callback arrays from the RX interrupt while they change.
        taskENTER_CRITICAL();
        const auto it = id_to_index_.find(std_id);
        if (it != id_to_index_.end()) {
            // Legacy motor wrappers intentionally override the base callback.
            // Replace it in place instead of consuming another slot or failing.
            const uint8_t slot = it->second;
            rx_args_[slot] = args;
            rx_callbacks_[slot] = callback;
            rx_dlc_args_[slot] = nullptr;
            rx_dlc_callbacks_[slot] = nullptr;
        } else if (callback_count_ >= MAX_CAN_DEVICES) {
            result = -1;
        } else {
            rx_args_[callback_count_] = args;
            rx_callbacks_[callback_count_] = callback;
            id_to_index_[std_id] = callback_count_;
            callback_count_++;
        }
        taskEXIT_CRITICAL();
        return result;
    }

    int CAN::RegisterDlcRxCallback(uint32_t std_id, can_rx_dlc_callback_t callback, void* args) {
        if (std_id > 0x7FF) return -1;

        int result = 0;
        taskENTER_CRITICAL();
        const auto it = id_to_index_.find(std_id);
        if (it != id_to_index_.end()) {
            const uint8_t slot = it->second;
            rx_args_[slot] = nullptr;
            rx_callbacks_[slot] = nullptr;
            rx_dlc_args_[slot] = args;
            rx_dlc_callbacks_[slot] = callback;
        } else if (callback_count_ >= MAX_CAN_DEVICES) {
            result = -1;
        } else {
            rx_dlc_args_[callback_count_] = args;
            rx_dlc_callbacks_[callback_count_] = callback;
            id_to_index_[std_id] = callback_count_;
            callback_count_++;
        }
        taskEXIT_CRITICAL();
        return result;
    }

    int CAN::RegisterUniqueDlcRxCallback(uint32_t std_id, can_rx_dlc_callback_t callback,
                                         void* args) {
        if (std_id > 0x7FF) return -1;

        int result = 0;
        taskENTER_CRITICAL();
        if (id_to_index_.find(std_id) != id_to_index_.end() ||
            callback_count_ >= MAX_CAN_DEVICES) {
            result = -1;
        } else {
            rx_dlc_args_[callback_count_] = args;
            rx_dlc_callbacks_[callback_count_] = callback;
            id_to_index_[std_id] = callback_count_;
            callback_count_++;
        }
        taskEXIT_CRITICAL();
        return result;
    }

    int CAN::Transmit(uint16_t id, const uint8_t data[], uint32_t length) {
        RM_EXPECT_TRUE(IS_CAN_DLC(length), "CAN tx data length exceeds limit");
        if (!IS_CAN_DLC(length))
            return -1;

        CAN_TxHeaderTypeDef header = {
            .StdId = id,
            .ExtId = 0x0,  // don't care since we use standard id mode
            .IDE = CAN_ID_STD,
            .RTR = CAN_RTR_DATA,
            .DLC = length,
            .TransmitGlobalTime = DISABLE,
        };

        uint32_t mailbox;
        HAL_StatusTypeDef status;
        // A CAN line can be shared by several tasks (e.g. motor commands and a
        // telemetry link). HAL_CAN_AddTxMessage writes several registers and is
        // not reentrant, so keep the mailbox setup atomic. The completion wait
        // stays outside the critical section.
        taskENTER_CRITICAL();
        status = HAL_CAN_AddTxMessage(hcan_, &header, (uint8_t*)data, &mailbox);
        taskEXIT_CRITICAL();
        if (status != HAL_OK)
            return -1;

        // Poll with a bounded deadline. AutoRetransmission can otherwise keep a
        // mailbox pending forever when no peer is present to ACK the frame.
        const uint32_t start_ms = HAL_GetTick();
        while (HAL_CAN_IsTxMessagePending(hcan_, mailbox)) {
            if (HAL_GetTick() - start_ms >= CAN_TX_COMPLETION_TIMEOUT_MS) {
                taskENTER_CRITICAL();
                HAL_CAN_AbortTxRequest(hcan_, mailbox);
                taskEXIT_CRITICAL();
                return -1;
            }
        }

        return length;
    }

    CAN::TxResult CAN::TryTransmit(uint16_t id, const uint8_t data[], uint32_t length) {
        // IS_CAN_DLC(0) is true, but a zero-length classic CAN data frame is not
        // useful to this interface.
        if (id > 0x7FF || data == nullptr || length == 0 || !IS_CAN_DLC(length))
            return TxResult::ERROR;

        // HAL_CAN_AddTxMessage reads all eight bytes even for a shorter DLC.
        uint8_t tx_data[MAX_CAN_DATA_SIZE] = {0};
        for (uint32_t i = 0; i < length; ++i) tx_data[i] = data[i];
        CAN_TxHeaderTypeDef header = {
            .StdId = id,
            .ExtId = 0x0,  // don't care since we use standard id mode
            .IDE = CAN_ID_STD,
            .RTR = CAN_RTR_DATA,
            .DLC = length,
            .TransmitGlobalTime = DISABLE,
        };

        uint32_t mailbox;
        TxResult result;
        taskENTER_CRITICAL();
        if (hcan_->State != HAL_CAN_STATE_READY && hcan_->State != HAL_CAN_STATE_LISTENING) {
            // A stopped/errored controller is not ordinary mailbox backpressure.
            result = TxResult::ERROR;
        } else if (HAL_CAN_GetTxMailboxesFreeLevel(hcan_) == 0) {
            result = TxResult::BUSY;
        } else if (HAL_CAN_AddTxMessage(hcan_, &header, tx_data, &mailbox) == HAL_OK) {
            result = TxResult::QUEUED;
        } else {
            // The only expected rejection after the free-level check is another
            // task taking the last mailbox; report that as backpressure.
            result = HAL_CAN_GetTxMailboxesFreeLevel(hcan_) == 0 ? TxResult::BUSY : TxResult::ERROR;
        }
        taskEXIT_CRITICAL();

        return result;
    }

    void CAN::RxCallback() {
        CAN_RxHeaderTypeDef header;
        uint8_t data[MAX_CAN_DATA_SIZE] = {0};
        if (HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &header, data) != HAL_OK)
            return;
        // This protocol family is standard-ID data frames only. The filter is
        // accept-all, so drop frames that do not match.
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA)
            return;
        uint16_t callback_id = header.StdId;
        const auto it = id_to_index_.find(callback_id);
        if (it == id_to_index_.end())
            return;
        callback_id = it->second;
        // find corresponding callback
        if (rx_dlc_callbacks_[callback_id])
            rx_dlc_callbacks_[callback_id](data, header.DLC, rx_dlc_args_[callback_id]);
        else if (rx_callbacks_[callback_id])
            rx_callbacks_[callback_id](data, rx_args_[callback_id]);
    }

    void CAN::ConfigureFilter(bool is_master) {
        CAN_FilterTypeDef CAN_FilterConfigStructure;
        /* Configure Filter Property */
        CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
        CAN_FilterConfigStructure.FilterIdLow = 0x0000;
        CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
        CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
        CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FILTER_FIFO0;
        CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
        CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
        CAN_FilterConfigStructure.FilterActivation = ENABLE;
        CAN_FilterConfigStructure.SlaveStartFilterBank = 14;  // CAN1 and CAN2 split all 28 filters
        /* Configure each CAN bus */
        if (is_master)
            CAN_FilterConfigStructure.FilterBank = 0;  // Master CAN get filter 0-13
        else
            CAN_FilterConfigStructure.FilterBank = 14;  // Slave CAN get filter 14-27

        RM_EXPECT_HAL_OK(HAL_CAN_ConfigFilter(hcan_, &CAN_FilterConfigStructure),
                         "CAN filter configuration failed.");
    }

} /* namespace bsp */
