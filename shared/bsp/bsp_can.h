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

#pragma once

#include <map>

#include "bsp_error_handler.h"
#include "can.h"

#define MAX_CAN_DATA_SIZE 8
#define MAX_CAN_DEVICES 12

namespace bsp {

/* can callback function pointer */
typedef void (*can_rx_callback_t)(const uint8_t data[], void* args);

/* can callback function pointer that also receives the received DLC */
typedef void (*can_rx_dlc_callback_t)(const uint8_t data[], uint32_t dlc, void* args);

class CAN {
  public:
    /* result of a non-blocking transmission attempt */
    enum class TxResult : uint8_t {
      QUEUED, /* a TX mailbox accepted the frame */
      BUSY,   /* all TX mailboxes are occupied, retry later */
      ERROR,  /* invalid arguments or the HAL rejected the frame */
    };

    /**
     * @brief constructor for bsp CAN instance
     *
     * @param hcan        HAL can handle
     * @param is_master   true for CAN1, false for CAN2
     */
    CAN(CAN_HandleTypeDef* hcan, bool is_master = true);
    /**
     * @brief check if it is associated with a given CAN handle
     *
     * @param hcan  HAL can handle to be checked
     *
     * @return true if associated, otherwise false
     */
    bool Uses(CAN_HandleTypeDef* hcan) {
      return hcan_ == hcan;
    }

    /**
     * @brief register callback function for a specific ID on this CAN line
     *
     * @param std_id    rx id
     * @param callback  callback function
     * @param args      argument passed into the callback function
     *
     * @return return 0 if success, -1 if invalid std_id or the table is full
     *
     * @note Registering an existing id replaces its callback in the same slot.
     *       ServoMotor/Steering6020 rely on this override behavior.
     */
    int RegisterRxCallback(uint32_t std_id, can_rx_callback_t callback, void* args = NULL);

    /**
     * @brief register a callback that also receives the received DLC
     *
     * @param std_id    rx id
     * @param callback  callback function
     * @param args      argument passed into the callback function
     *
     * @return return 0 if success, -1 if the id is invalid or the table is full
     *
     * @note Use this instead of RegisterRxCallback when the protocol has to
     *       reject frames whose DLC is not the expected length.  Registering an
     *       existing id replaces its callback in the same slot, preserving the
     *       legacy motor-wrapper override behavior.
     */
    int RegisterDlcRxCallback(uint32_t std_id, can_rx_dlc_callback_t callback, void* args = NULL);

    /**
     * @brief strict DLC callback registration for protocol links
     *
     * Unlike RegisterDlcRxCallback, this rejects an already registered id
     * instead of replacing its callback.  Use it to detect ID allocation bugs
     * without changing legacy motor override semantics.
     */
    int RegisterUniqueDlcRxCallback(uint32_t std_id, can_rx_dlc_callback_t callback,
                                    void* args = NULL);

    /**
     * @brief transmit can messages
     *
     * @param id      tx id
     * @param data[]  data bytes
     * @param length  length of data, must be in (0, 8]
     *
     * @return  number of bytes transmitted, -1 if failed or the 2 ms completion
     *          deadline expires (the pending mailbox is aborted on timeout)
     */
    int Transmit(uint16_t id, const uint8_t data[], uint32_t length);

    /**
     * @brief try to queue a can message without blocking
     *
     * @param id      tx id
     * @param data[]  data bytes
     * @param length  length of data, must be in (0, 8]
     *
     * @return QUEUED if a TX mailbox accepted the frame, BUSY if all TX
     *         mailboxes are currently occupied, ERROR otherwise
     *
     * @note Unlike Transmit(), this never waits for a mailbox.  Callers that
     *       share the bus with a control loop can retry on BUSY.  Call this from
     *       task context only: the short mailbox critical section is not
     *       interrupt-safe.
     */
    TxResult TryTransmit(uint16_t id, const uint8_t data[], uint32_t length);

    /**
     * @brief callback wrapper called from IRQ context
     *
     * @note should not be called explicitly form the application side
     */
    void RxCallback();

  private:
    void ConfigureFilter(bool is_master);

    CAN_HandleTypeDef* hcan_;

    can_rx_callback_t rx_callbacks_[MAX_CAN_DEVICES] = {0};
    void* rx_args_[MAX_CAN_DEVICES] = {NULL};

    can_rx_dlc_callback_t rx_dlc_callbacks_[MAX_CAN_DEVICES] = {0};
    void* rx_dlc_args_[MAX_CAN_DEVICES] = {NULL};

    std::map<uint16_t, uint8_t> id_to_index_;
    uint8_t callback_count_ = 0;

    static std::map<CAN_HandleTypeDef*, CAN*> ptr_map;
    static CAN* FindInstance(CAN_HandleTypeDef* hcan);
    static bool HandleExists(CAN_HandleTypeDef* hcan);
    static void RxFIFO0MessagePendingCallback(CAN_HandleTypeDef* hcan);
};

} /* namespace bsp */
