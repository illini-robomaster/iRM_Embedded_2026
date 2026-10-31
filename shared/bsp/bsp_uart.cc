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

#include "bsp_uart.h"

#include <cstring>
#include <map>

#include "bsp_error_handler.h"
#include "cmsis_os.h"
#include "task.h"

#define MAX_NUM_UARTS 5

/* Upper bound for waiting the RX stream enable bit to clear. The loop reads one
 * register, so this is a handful of microseconds, and it exists because the HAL's own
 * wait cannot time out inside a critical section: HAL_GetTick() is frozen while
 * SysTick is masked, so an unbounded wait would hang the interrupt instead of
 * reporting a failure. */
constexpr uint32_t kRxDmaDisableWaitLoops = 1000u;

/* The host regression test in tests/dbus_input exercises the same error masks and
 * restart decisions without the HAL, so they must match the driver of the board
 * being built. */
static_assert(bsp::kRxErrorParity == HAL_UART_ERROR_PE, "PE mask out of sync with the HAL");
static_assert(bsp::kRxErrorNoise == HAL_UART_ERROR_NE, "NE mask out of sync with the HAL");
static_assert(bsp::kRxErrorFrame == HAL_UART_ERROR_FE, "FE mask out of sync with the HAL");
static_assert(bsp::kRxErrorOverrun == HAL_UART_ERROR_ORE, "ORE mask out of sync with the HAL");
static_assert(bsp::kRxErrorDma == HAL_UART_ERROR_DMA, "DMA mask out of sync with the HAL");
static_assert(bsp::kRxErrorNone == HAL_UART_ERROR_NONE, "neutral error code out of sync");

namespace bsp {

/* DMA_HandleTypeDef::Instance is a void* on the H7 (it also covers BDMA) and a
 * typed stream pointer on the F4/F7, so the stream registers are reached through an
 * explicit cast. UART RX DMA always uses a DMA stream, never a BDMA channel. */
static DMA_Stream_TypeDef* RxDmaStream(DMA_HandleTypeDef* hdma) {
  return reinterpret_cast<DMA_Stream_TypeDef*>(hdma->Instance);
}

/* The HAL state enum values differ between the H7 and the F4, so the state is
 * mapped by name instead of cast. */
static RxDmaState RxDmaStateOf(DMA_HandleTypeDef* hdma) {
  switch (hdma->State) {
    case HAL_DMA_STATE_READY:
      return RxDmaState::kReady;
    case HAL_DMA_STATE_BUSY:
      return RxDmaState::kBusy;
    case HAL_DMA_STATE_ABORT:
      return RxDmaState::kAbort;
    default:
      /* RESET / TIMEOUT / ERROR / UNKNOWN: no transfer the HAL would continue */
      return RxDmaState::kUnusable;
  }
}

#ifdef BOARD_HAS_UART_DMA_DOUBLE_BUFFER
/* Make a double-buffered RX stream fill rx_data_[index]. M0AR/M1AR keep their
 * physical buffers and DMA_SxCR_CT selects the written one, which is the mapping
 * Read() keeps in its healthy path (CT == rx_index_); CT may only be written while
 * the stream is disabled, so callers do this right before starting it. */
static void SelectRxDmaBuffer(DMA_HandleTypeDef* hdma, uint8_t index) {
  if (RxDoubleBufferCtForIndex(index)) {
    SET_BIT(RxDmaStream(hdma)->CR, DMA_SxCR_CT);
  } else {
    CLEAR_BIT(RxDmaStream(hdma)->CR, DMA_SxCR_CT);
  }
}
#endif

/* modified version of HAL_UART_Receive_DMA */
static HAL_StatusTypeDef UartStartDmaNoInt(UART_HandleTypeDef* huart, uint8_t* data0,
                                           uint8_t* data1, uint16_t size) {
  /* Check that a Rx process is not already ongoing */
  if (huart->RxState == HAL_UART_STATE_READY) {
    if ((data0 == NULL) || (data1 == NULL) || (size == 0U)) return HAL_ERROR;

    /* Process Locked */
    __HAL_LOCK(huart);

    huart->RxXferSize = size;
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    /* Enable the DMA stream */
#ifdef BOARD_HAS_UART_DMA_DOUBLE_BUFFER
    /* the transfer starts at buffer 0, i.e. CT = 0 with the mapping above */
    SelectRxDmaBuffer(huart->hdmarx, 0);
#if defined(STM32H7)
    HAL_DMAEx_MultiBufferStart(huart->hdmarx, (uint32_t)&huart->Instance->RDR, (uint32_t)data0,
                               (uint32_t)data1, size);
#else
    HAL_DMAEx_MultiBufferStart(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)data0,
                               (uint32_t)data1, size);
#endif
#else
#if defined(STM32H7)
    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->RDR, (uint32_t)data0, size);
#else
    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)data0, size);
#endif
#endif

    /* Clear the Overrun flag just before enabling the DMA Rx request */
    __HAL_UART_CLEAR_OREFLAG(huart);

    /* Process Unlocked */
    __HAL_UNLOCK(huart);

    /* Enable the UART Parity Error Interrupt */
    SET_BIT(huart->Instance->CR1, USART_CR1_PEIE);

    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    SET_BIT(huart->Instance->CR3, USART_CR3_EIE);

    /* Enable the DMA transfer for the receiver request by setting the DMAR bit
    in the UART CR3 register */
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    return HAL_OK;
  } else {
    return HAL_BUSY;
  }
}

/* tx dma complete -> check for pending message to keep transmitting */
void TxCompleteCallbackWrapper(UART_HandleTypeDef* huart) {
  UART* uart = UART::FindInstance(huart);
  if (!uart) return;
  uart->TxCompleteCallback();
}

/* rx idle line detected -> trigger rx callback */
void RxCompleteCallbackWrapper(UART_HandleTypeDef* huart) {
  UART* uart = UART::FindInstance(huart);
  if (!uart) return;

  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE)) {
    uart->RxCompleteCallback();
    __HAL_UART_CLEAR_IDLEFLAG(huart);
  }
  /* Hardware error flags are not cleared here: HAL_UART_IRQHandler runs first and
   * already cleared them, which is why the error is captured in the HAL error
   * callback below instead. Clearing them again would read RDR and could steal a
   * byte from the next burst. */
}

/* uart reception error -> capture evidence before any recovery discards it
 * (only reached when the HAL delivers the error callback, see SetupRx) */
void RxErrorCallbackWrapper(UART_HandleTypeDef* huart) {
  UART* uart = UART::FindInstance(huart);
  if (!uart) return;
  uart->RecordRxError(huart->ErrorCode);
}

std::map<UART_HandleTypeDef*, UART*> UART::ptr_map;

/* get initialized uart_t instance given its huart handle struct */
UART* UART::FindInstance(UART_HandleTypeDef* huart) {
  const auto it = ptr_map.find(huart);
  if (it == ptr_map.end()) {
    return nullptr;
  }

  return it->second;
}

UART::UART(UART_HandleTypeDef* huart)
    : huart_(huart),
      rx_size_(0),
      rx_data_{nullptr},
      rx_index_(0),
      tx_size_(0),
      tx_pending_(0),
      tx_write_(nullptr),
      tx_read_(nullptr) {
  RM_ASSERT_FALSE(FindInstance(huart), "Uart repeated initialization");
  ptr_map[huart] = this;
}

UART::~UART() {
  if (rx_data_[0]) delete[] rx_data_[0];
  if (rx_data_[1]) delete[] rx_data_[1];
  if (tx_write_) delete[] tx_write_;
  if (tx_read_) delete[] tx_read_;
}

void UART::SetupRx(uint32_t rx_buffer_size) {
  /* uart rx already setup */
  if (rx_size_ || rx_data_[0] || rx_data_[1]) return;

  rx_size_ = rx_buffer_size;
  rx_data_[0] = new uint8_t[rx_buffer_size];
  rx_data_[1] = new uint8_t[rx_buffer_size];
  /* the first transfer fills buffer 0; Read() flips this together with the DMA
   * buffer selection on every burst */
  rx_index_ = 0;

  /* enable uart rx dma transfer in back ground */
  UartStartDmaNoInt(huart_, rx_data_[0], rx_data_[1], rx_size_);

  /* UART IDLE Interrupt can notify application of data reception ASAP */
  __HAL_UART_CLEAR_FLAG(huart_, UART_FLAG_IDLE);
  __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  /* Best effort error evidence: the callback reports the code the HAL captured
   * before it cleared the ISR flags. For a DMA reception the HAL only reaches it
   * through HAL_DMA_Abort_IT()'s deferred abort callback, which a polling-mode
   * stream never runs (see bsp_uart_rx.h), so Read() polls huart->ErrorCode as
   * well; RxErrorEvidence keeps the two paths from counting an error twice. */
  HAL_UART_RegisterCallback(huart_, HAL_UART_ERROR_CB_ID, RxErrorCallbackWrapper);
#else
  /* Without per-instance callbacks only the weak HAL_UART_ErrorCallback remains,
   * which carries no instance mapping; the chain is still repaired by the next
   * Read() because it watches CR3.DMAR, the interrupt enables and the DMA stream
   * enable bit. */
#endif
}

void UART::SetupTx(uint32_t tx_buffer_size) {
  /* uart tx already setup */
  if (tx_size_ || tx_write_ || tx_read_) return;

  tx_size_ = tx_buffer_size;
  tx_pending_ = 0;
  tx_write_ = new uint8_t[tx_buffer_size];
  tx_read_ = new uint8_t[tx_buffer_size];

  HAL_UART_RegisterCallback(huart_, HAL_UART_TX_COMPLETE_CB_ID, TxCompleteCallbackWrapper);
}

template <bool FromISR>
int32_t UART::Read(uint8_t** data) {
  if (!data) return -1;
  *data = nullptr;
  if (rx_data_[0] == nullptr || rx_data_[1] == nullptr) return -1; /* SetupRx not called */
  /* capture pending bytes and perform hardware buffer switch */
  int32_t length = -1;

  // enter critical session
  UBaseType_t isrflags;
  if (FromISR) {
    isrflags = taskENTER_CRITICAL_FROM_ISR();
  } else {
    taskENTER_CRITICAL();
  }

  const RxChainSnapshot chain = RxChain();
  /* One error, one count: the HAL error callback records it when the HAL delivers
   * it, and the ErrorCode the blocking error path leaves behind covers the case
   * where it does not. The code is consumed and cleared here either way so the next
   * burst cannot count the same error again. */
  const bool error_pending = rx_errors_.ConsumePending(huart_->ErrorCode);
  huart_->ErrorCode = kRxErrorNone;

  const RxRepair repair = PlanRxRepair(chain, error_pending);
  /* *data is always a valid buffer address so a caller that ignores the length
   * cannot dereference an indeterminate pointer; only the length says whether the
   * burst is usable. */
  *data = rx_data_[rx_index_];

  switch (ClassifyRxBurst(chain, error_pending)) {
    case RxBurstVerdict::kUsable: {
      if (repair == RxRepair::kRearmRequest) {
        /* the stream still runs and holds a valid burst, only the request / IRQ
         * bits were cleared (e.g. by HAL_UART_EndRxTransfer) */
        RearmRxChain(repair);
      }

      __HAL_DMA_DISABLE(huart_->hdmarx);
      length = rx_size_ - __HAL_DMA_GET_COUNTER(huart_->hdmarx);
      rx_index_ = 1 - rx_index_;

#ifdef BOARD_HAS_UART_DMA_DOUBLE_BUFFER
      // hardware multi buffer switch
      __HAL_DMA_SET_COUNTER(huart_->hdmarx, rx_size_);
      huart_->hdmarx->Instance->CR ^= DMA_SxCR_CT;
      __HAL_DMA_ENABLE(huart_->hdmarx);
#else
      // software double buffer
      HAL_DMA_Abort(huart_->hdmarx);
#if defined(STM32H7)
      // STM32H7 uses RDR instead of DR
      HAL_DMA_Start(huart_->hdmarx, (uint32_t)&huart_->Instance->RDR, (uint32_t)rx_data_[rx_index_],
                    rx_size_);
#else
      // STM32F4/F7 uses DR
      HAL_DMA_Start(huart_->hdmarx, (uint32_t)&huart_->Instance->DR, (uint32_t)rx_data_[rx_index_],
                    rx_size_);
#endif
#endif

      /* return the buffer pointer currently not being used by DMA transfer */
      *data = rx_data_[1 - rx_index_];
      break;
    }

    case RxBurstVerdict::kTruncated:
      /* The stream stopped because a complete normal-mode transfer filled the whole
       * buffer: the burst did arrive, it is just longer than the buffer. Report its
       * real length so the framing rules classify it as truncation instead of the
       * caller counting it as a reception error. Switch buffers first: *data points
       * to the completed buffer, which the restarted DMA must not overwrite. */
      rx_index_ = 1 - rx_index_;
      RearmRxChain(RxRepair::kRearmStreamAndRequest);
      length = static_cast<int32_t>(rx_size_);
      break;

    case RxBurstVerdict::kDiscarded:
      /* The burst that overlapped the error has an untrustworthy length and content:
       * the counter of an aborted stream reads like a fresh full buffer. Throw it
       * away, re-arm the whole chain and report a failure so the caller does not
       * decode it. Keep even the discarded buffer separate from the DMA target. */
      rx_index_ = 1 - rx_index_;
      RearmRxChain(RxRepair::kRearmStreamAndRequest);
      ++rx_discarded_bursts_;
      break;
  }

  // exit critical session
  if (FromISR) {
    taskEXIT_CRITICAL_FROM_ISR(isrflags);
  } else {
    taskEXIT_CRITICAL();
  }

  return length;
}

template int32_t UART::Read<true>(uint8_t** data);
template int32_t UART::Read<false>(uint8_t** data);

template <bool FromISR>
int32_t UART::Write(const uint8_t* data, uint32_t length) {
  // enter critical session
  UBaseType_t isrflags;
  if (FromISR) {
    isrflags = taskENTER_CRITICAL_FROM_ISR();
  } else {
    taskENTER_CRITICAL();
  }

  if (huart_->gState == HAL_UART_STATE_BUSY_TX || tx_pending_) {
    /* uart tx currently transmitting -> atomically queue up new data */
    if (length + tx_pending_ > tx_size_) length = tx_size_ - tx_pending_;
    memcpy(tx_write_ + tx_pending_, data, length);
    tx_pending_ += length;
  } else {
    if (length > tx_size_) length = tx_size_;
    /* directly write into the read buffer and start transmission */
    memcpy(tx_read_, data, length);
    /* check if dma is enabled */
    if (huart_->hdmatx != NULL) {
      HAL_UART_Transmit_DMA(huart_, tx_read_, length);
    } else {
      HAL_UART_Transmit(huart_, tx_read_, length, 100);
    }
  }

  // exit critical session
  if (FromISR) {
    taskEXIT_CRITICAL_FROM_ISR(isrflags);
  } else {
    taskEXIT_CRITICAL();
  }

  return length;
}

template int32_t UART::Write<true>(const uint8_t* data, uint32_t length);
template int32_t UART::Write<false>(const uint8_t* data, uint32_t length);

void UART::TxCompleteCallback() {
  uint8_t* tmp;
  UBaseType_t isrflags = taskENTER_CRITICAL_FROM_ISR();
  /* check if any data is waiting to be transmitted */
  if (tx_pending_) {
    /* swap read / write buffer */
    tmp = tx_read_;
    tx_read_ = tx_write_;
    tx_write_ = tmp;
    /* initiate new transmission call for pending data */
    HAL_UART_Transmit_DMA(huart_, tx_read_, tx_pending_);
    /* clear the number of pending bytes */
    tx_pending_ = 0;
  }
  taskEXIT_CRITICAL_FROM_ISR(isrflags);
}

void UART::RxCompleteCallback() {}

void UART::RxErrorCallback(uint32_t error_code) { (void)error_code; }

/* Called from the HAL error callback, i.e. from interrupt context, once per error
 * event the HAL reports that way. The HAL has already cleared the ISR error flags at
 * this point, so huart->ErrorCode is the only surviving evidence. */
void UART::RecordRxError(uint32_t error_code) {
  rx_errors_.Record(error_code);
  RxErrorCallback(error_code);
}

RxChainSnapshot UART::RxChain() const {
  RxChainSnapshot chain = {};
  chain.uart_dma_request = (huart_->Instance->CR3 & USART_CR3_DMAR) != 0u;
  chain.error_irq_enabled = (huart_->Instance->CR3 & USART_CR3_EIE) != 0u;
  chain.parity_irq_enabled = (huart_->Instance->CR1 & USART_CR1_PEIE) != 0u;
  chain.idle_irq_enabled = (huart_->Instance->CR1 & USART_CR1_IDLEIE) != 0u;
  if (huart_->hdmarx == nullptr) return chain;

  chain.dma_stream_enabled = (RxDmaStream(huart_->hdmarx)->CR & DMA_SxCR_EN) != 0u;
  /* A normal-mode transfer that ran to completion stops the stream with an empty
   * counter while the HAL still sees it as busy (polling mode never reports the
   * completion); a reception error instead leaves HAL_DMA_STATE_ABORT behind. That
   * difference is what tells a full buffer apart from an aborted burst. */
  chain.dma_stream_full = !chain.dma_stream_enabled && rx_size_ != 0u &&
                          huart_->hdmarx->State == HAL_DMA_STATE_BUSY &&
                          __HAL_DMA_GET_COUNTER(huart_->hdmarx) == 0u;
  return chain;
}

/* Bring the RX DMA handle into the state the start calls accept and drop everything
 * the interrupted transfer left behind, so the fresh transfer cannot be reached by a
 * stale abort callback or a stale interrupt flag. Caller holds a critical section.
 *
 * @return false when the stream refuses to stop, i.e. it must not be restarted
 */
bool UART::PrepareRxDmaRestart() {
  DMA_HandleTypeDef* hdma = huart_->hdmarx;

  /* The HAL error path armed UART_DMAAbortOnError, which reports the old error
   * through HAL_UART_ErrorCallback; it must not run for the new transfer. */
  hdma->XferAbortCallback = nullptr;

  /* Polling-mode transfers enable no stream interrupt, but an aborted one can leave
   * the enables behind: clear them so a pending transfer-complete flag cannot run the
   * old XferAbortCallback (or any other DMA callback) during the new transfer. */
  RxDmaStream(hdma)->CR &= ~(DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME);
  RxDmaStream(hdma)->FCR &= ~DMA_IT_FE;

  __HAL_DMA_DISABLE(hdma);
  uint32_t wait = kRxDmaDisableWaitLoops;
  while ((RxDmaStream(hdma)->CR & DMA_SxCR_EN) != 0u) {
    if (--wait == 0u) return false;
  }

  switch (PlanRxStreamRestart(RxDmaStateOf(hdma))) {
    case RxStreamRestart::kReadyNow:
      break;
    case RxStreamRestart::kAbortThenStart:
      /* BUSY -> READY, and HAL_DMA_Abort() clears the stream flags as well; the
       * stream is already stopped, so its own enable-wait returns immediately */
      HAL_DMA_Abort(hdma);
      break;
    case RxStreamRestart::kForceReadyThenStart:
      /* A reception error leaves HAL_DMA_STATE_ABORT behind through
       * HAL_DMA_Abort_IT(), a failed abort HAL_DMA_STATE_ERROR. HAL_DMA_Abort()
       * refuses any state but BUSY and the start calls any state but READY, so the
       * state has to be forced or the restart fails on every attempt and reception
       * stays down until reset. The stream is stopped above, so resetting the
       * bookkeeping is all that is left to do. */
      hdma->State = HAL_DMA_STATE_READY;
      break;
  }

  /* The start calls take the handle lock and only give it back when they fail, so a
   * polling-mode transfer keeps it held; release it or the restart is refused with
   * HAL_ERROR/HAL_BUSY before it even looks at the state. */
  hdma->Lock = HAL_UNLOCKED;
  return true;
}

/* Start the RX DMA transfer into rx_data_[index]; only valid for a stopped stream. */
HAL_StatusTypeDef UART::StartRxDmaInto(uint8_t index) {
  DMA_HandleTypeDef* hdma = huart_->hdmarx;
#ifdef BOARD_HAS_UART_DMA_DOUBLE_BUFFER
  /* Buffers keep their physical place and CT selects the written one, so Read() can
   * keep flipping CT together with rx_index_ (see SelectRxDmaBuffer). */
  SelectRxDmaBuffer(hdma, index);
  return HAL_DMAEx_MultiBufferStart(hdma,
#if defined(STM32H7)
                                    (uint32_t)&huart_->Instance->RDR, (uint32_t)rx_data_[0],
#else
                                    (uint32_t)&huart_->Instance->DR, (uint32_t)rx_data_[0],
#endif
                                    (uint32_t)rx_data_[1], rx_size_);
#else
  return HAL_DMA_Start(hdma,
#if defined(STM32H7)
                       (uint32_t)&huart_->Instance->RDR, (uint32_t)rx_data_[index], rx_size_);
#else
                       (uint32_t)&huart_->Instance->DR, (uint32_t)rx_data_[index], rx_size_);
#endif
#endif
}

/* Restore the receive chain that a HAL error path (or a stray abort) dismantled.
 * Caller holds a critical section and is responsible for the DMA stream not being
 * about to deliver a burst it still trusts. */
void UART::RearmRxChain(RxRepair repair) {
  ++rx_rearm_count_;

  if (repair == RxRepair::kRearmStreamAndRequest) {
    if (huart_->hdmarx == nullptr) {
      ++rx_rearm_failures_;
      return;
    }

    /* Restart into the buffer that was being filled when the chain broke: the
     * interrupted burst is discarded, not completed, and the "stream fills
     * rx_data_[rx_index_]" invariant of Read() is preserved. */
    if (!PrepareRxDmaRestart()) {
      /* The stream does not stop, so it must not be reconfigured; the next burst
       * retries, and the failure counter is what the application sees. */
      ++rx_rearm_failures_;
    } else if (StartRxDmaInto(rx_index_) != HAL_OK) {
      ++rx_rearm_failures_;
      /* Reception stays down until the next burst retries the restart, so the handle
       * has to be left in a state that retry can use: HAL_DMA_Abort() reports a
       * timeout by leaving HAL_DMA_STATE_ERROR, which the start calls refuse. */
      huart_->hdmarx->State = HAL_DMA_STATE_READY;
      huart_->hdmarx->Lock = HAL_UNLOCKED;
    } else {
      huart_->ErrorCode = kRxErrorNone;
    }

    /* Clear the overrun the stopped stream caused, together with its siblings, by
     * reading ISR and RDR the way the HAL does. Bytes still pending are dropped on
     * purpose: they belong to the discarded burst. This happens even when the
     * restart failed, or the pending overrun would take the HAL error path again as
     * soon as the error interrupts come back. */
    __HAL_UART_CLEAR_OREFLAG(huart_);
  }

  /* UART request + error interrupts. HAL_UART_EndRxTransfer clears CR1.PEIE and
   * CR3.EIE (and CR1.RXNEIE, which DMA reception does not use), and the blocking
   * error path clears CR3.DMAR, so all of them have to come back before the DMA can
   * receive again. They are restored even when the restart failed: the idle
   * interrupt is what retries the restart on the next burst, and one attempt per
   * burst keeps a broken chain from spinning. */
  SET_BIT(huart_->Instance->CR1, USART_CR1_PEIE);
  SET_BIT(huart_->Instance->CR3, USART_CR3_EIE);
  SET_BIT(huart_->Instance->CR3, USART_CR3_DMAR);
  __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);
}

} /* namespace bsp */

/* overwrite the weak function defined in board specific usart.c to handle IRQ requests */
extern "C" void RM_UART_IRQHandler(UART_HandleTypeDef* huart) { bsp::RxCompleteCallbackWrapper(huart); }
