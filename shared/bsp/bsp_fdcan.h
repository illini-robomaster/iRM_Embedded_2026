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

#pragma once

#include <map>

#include "bsp_error_handler.h"
#include "fdcan.h"

#define MAX_FDCAN_DATA_SIZE 8
#define MAX_FDCAN_DEVICES 12

namespace bsp {

/* FDCAN callback function pointer */
typedef void (*fdcan_rx_callback_t)(const uint8_t data[], void* args);

class FDCAN {
  public:
    /**
     * @brief constructor for bsp FDCAN instance
     *
     * @param hfdcan      HAL FDCAN handle
     * @param filter_id   filter bank ID (0-27)
     */
    FDCAN(FDCAN_HandleTypeDef* hfdcan, uint32_t filter_id = 0);

    /**
     * @brief check if it is associated with a given FDCAN handle
     *
     * @param hfdcan  HAL FDCAN handle to be checked
     *
     * @return true if associated, otherwise false
     */
    bool Uses(FDCAN_HandleTypeDef* hfdcan) {
      return hfdcan_ == hfdcan;
    }

    /**
     * @brief register callback function for a specific ID on this FDCAN line
     *
     * @param std_id    rx id
     * @param callback  callback function
     * @param args      argument passed into the callback function
     *
     * @return return 0 if success, -1 if invalid std_id
     */
    int RegisterRxCallback(uint32_t std_id, fdcan_rx_callback_t callback, void* args = NULL);

    /**
     * @brief transmit FDCAN messages
     *
     * @param id      tx id
     * @param data[]  data bytes
     * @param length  length of data, must be in (0, 8]
     *
     * @return  number of bytes transmitted, -1 if failed
     */
    int Transmit(uint16_t id, const uint8_t data[], uint32_t length);

    /**
     * @brief callback wrapper called from IRQ context
     *
     * @note should not be called explicitly from the application side
     */
    void RxCallback();

    /**
     * @brief static callback handler for HAL interrupt
     *
     * @param hfdcan      HAL FDCAN handle
     * @param RxFifo0ITs  FIFO 0 interrupt flags
     * 
     * @note This is public to allow extern "C" HAL callback override
     */
    static void RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs);

  private:
    void ConfigureFilter(uint32_t filter_id);

    FDCAN_HandleTypeDef* hfdcan_;

    fdcan_rx_callback_t rx_callbacks_[MAX_FDCAN_DEVICES] = {0};
    void* rx_args_[MAX_FDCAN_DEVICES] = {NULL};

    std::map<uint16_t, uint8_t> id_to_index_;
    uint8_t callback_count_ = 0;

    static std::map<FDCAN_HandleTypeDef*, FDCAN*> ptr_map;
    static FDCAN* FindInstance(FDCAN_HandleTypeDef* hfdcan);
    static bool HandleExists(FDCAN_HandleTypeDef* hfdcan);
};

} /* namespace bsp */
