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

#include "bsp_uart_rx.h"
#include "usart.h"

namespace bsp {

class UART {
 public:
  /**
   * @brief constructor for uart instance
   *
   * @param huart pointer to a HAL uart handle
   */
  explicit UART(UART_HandleTypeDef* huart);

  /**
   * @brief destructor (potentially deallocate buffer memories associated with tx / rx)
   */
  virtual ~UART();

  /**
   * @brief set up uart receiver in the background optionally registering a callback
   *
   * @param rx_buffer_size  receive buffer size (all data that has not been read
   *                        out is queued into this buffer)
   */
  void SetupRx(uint32_t rx_buffer_size);

  /**
   * @brief set up non blocking transmission functionality
   *
   * @param tx_buffer_size  transmission buffer size (burst transmission calls will
   *                        be queued into this buffer)
   */
  void SetupTx(uint32_t tx_buffer_size);

  /**
   * @brief read out the pending received data
   *
   * @tparam FromISR  set to true to call inside an interrupt handler
   * @param data  pointer to an array address that gets set to the receive buffer address;
   *              always a valid buffer address, so a caller that sees a failure can
   *              distinguish it by the return value alone
   *
   * @return number of bytes read (never more than the receive buffer size), or -1 if
   *         the burst was discarded because a reception error interrupted the chain or
   *         the transfer was stopped before the burst ended. A burst that filled the
   *         whole buffer is reported as a full-length read, i.e. as truncation.
   *
   * @note memory is not copied for optimal performance, so second call to this
   *       method will invalidate the buffer produced by the previous call
   *
   * @note a UART reception error stops the whole HAL receive chain (CR3.DMAR is
   *       cleared and the RX DMA stream is aborted), so this method re-arms the
   *       chain and reports -1 for the polluted burst before the next burst is
   *       decoded. The retry of a failed re-arm happens on the next call, which the
   *       idle interrupt of the next burst triggers.
   */
  template <bool FromISR = false>
  int32_t Read(uint8_t** data);

  /**
   * @brief write data to uart without blocking
   *
   * @tparam FromISR  set to true to call inside an interrupt handler
   * @param data    pointer to the data buffer to be transmitted
   * @param length  length of the data to be transmitted
   *
   * @return number of bytes written
   *
   * @note multiple burst calls to this function can potentially cause tx buffer
   *       to fill up, so remember to check return value for the actual number
   *       of bytes successfully transmitted
   */
  template <bool FromISR = false>
  int32_t Write(const uint8_t* data, uint32_t length);

  /**
   * @brief number of reception errors captured, each error counted once
   *
   * The HAL clears the UART ISR error flags before any user code runs, so the error
   * has to be captured from huart->ErrorCode: by the HAL error callback when the HAL
   * delivers it, and otherwise by Read(), which polls the code the blocking error
   * path leaves behind (see bsp_uart_rx.h). The count is evidence for diagnostics
   * only: the burst that overlapped the error is discarded, see
   * RxDiscardedBursts().
   */
  uint32_t RxErrorCount() const { return rx_errors_.count; }

  /** @brief bsp::kRxError* mask of the last reception error, 0 if none yet. */
  uint32_t RxLastErrorCode() const { return rx_errors_.last_code; }

  /**
   * @brief number of bursts thrown away because the receive chain was interrupted
   *
   * A burst that overlapped a UART error (or a transfer that was stopped before the
   * burst ended) has an untrustworthy length and content, so Read() reports it as a
   * failure and re-arms the chain instead of handing it out. A burst that merely
   * filled the whole buffer is not counted here, Read() reports it as truncation.
   */
  uint32_t RxDiscardedBursts() const { return rx_discarded_bursts_; }

  /** @brief number of times the receive chain had to be re-armed. */
  uint32_t RxRearmCount() const { return rx_rearm_count_; }

  /**
   * @brief number of re-arm attempts that failed
   *
   * Reception stays down until an attempt succeeds; the next Read() retries, driven
   * by the idle interrupt of the next burst.
   */
  uint32_t RxRearmFailures() const { return rx_rearm_failures_; }

  /**
   * @brief live receive-chain bits of this UART
   *
   * Read-only view of CR3.DMAR, the RX DMA stream enable bit and the interrupt
   * enables that every reception depends on; meant for boot self checks, because
   * the CubeMX handle only holds what was requested.
   */
  RxChainSnapshot RxChain() const;

 protected:
  /**
   * @brief Transmission complete call back.
   */
  void TxCompleteCallback();

  /**
   * @brief Reception complete call back.
   */
  virtual void RxCompleteCallback();

  /**
   * @brief Reception error call back, called from interrupt context.
   *
   * Called with the error code the HAL captured before clearing the ISR flags
   * (bsp::kRxError* mask), but only when the HAL delivers the error callback at all:
   * for a DMA reception it depends on the stream abort callback, which a polling-mode
   * transfer never runs. The reception itself is repaired by the next Read(), which
   * also discards the polluted burst.
   *
   * @param error_code  bsp::kRxError* mask of the error that occurred
   */
  virtual void RxErrorCallback(uint32_t error_code);

  UART_HandleTypeDef* huart_;
  /* rx */
  uint32_t rx_size_;
  uint8_t* rx_data_[2];
  uint8_t rx_index_;
  /* tx */
  uint32_t tx_size_;
  uint32_t tx_pending_;
  uint8_t* tx_write_;
  uint8_t* tx_read_;

 private:
  friend void RxCompleteCallbackWrapper(UART_HandleTypeDef* huart);
  friend void TxCompleteCallbackWrapper(UART_HandleTypeDef* huart);
  friend void RxErrorCallbackWrapper(UART_HandleTypeDef* huart);

  static std::map<UART_HandleTypeDef*, UART*> ptr_map;
  static UART* FindInstance(UART_HandleTypeDef* huart);

  bool PrepareRxDmaRestart();
  HAL_StatusTypeDef StartRxDmaInto(uint8_t index);
  void RearmRxChain(RxRepair repair);
  void RecordRxError(uint32_t error_code);

  /* reception error evidence, written from interrupt context, counted once per error */
  RxErrorEvidence rx_errors_;
  /* rx recovery counters, written from interrupt context */
  volatile uint32_t rx_discarded_bursts_ = 0;
  volatile uint32_t rx_rearm_count_ = 0;
  volatile uint32_t rx_rearm_failures_ = 0;
};

} /* namespace bsp */
