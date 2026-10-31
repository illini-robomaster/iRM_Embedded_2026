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

/**
 * @file bsp_uart_rx.h
 * @brief Receive-chain health, error evidence and restart decisions of bsp::UART.
 *
 * What the STM32 HAL does on a reception error (verified in the H7 and F4
 * stm32xxxx_hal_uart.c of this repository, HAL_UART_IRQHandler):
 * - it clears the ISR error flags of PE/FE/NE/ORE and ORs them into
 *   huart->ErrorCode, which nothing on this path clears;
 * - it calls UART_EndRxTransfer(): CR1.RXNEIE, CR1.PEIE and CR3.EIE off, RxState
 *   READY (CR1.IDLEIE stays set, ReceptionType is not TOIDLE);
 * - because CR3.DMAR is set it clears CR3.DMAR and calls HAL_DMA_Abort_IT().
 *
 * HAL_DMA_Abort_IT() is where the recovery is decided: for a DMA1/DMA2 stream it
 * only sets the handle to HAL_DMA_STATE_ABORT and disables the stream, and it
 * leaves UART_DMAAbortOnError as XferAbortCallback. That callback is the only
 * route to HAL_UART_ErrorCallback but it is invoked from the stream's
 * transfer-complete interrupt, which a polling-mode reception (what this BSP
 * uses) never enables - so the HAL error callback is normally NOT delivered and
 * huart->ErrorCode is the only surviving evidence. Both are handled here: the
 * evidence records whatever arrives and the BSP additionally polls ErrorCode.
 *
 * Consequence for a restart: HAL_DMA_Abort() accepts HAL_DMA_STATE_BUSY only and
 * HAL_DMA_Start()/HAL_DMAEx_MultiBufferStart() accept HAL_DMA_STATE_READY only,
 * so the HAL_DMA_STATE_ABORT the error left behind refuses both and has to be
 * normalized first (PlanRxStreamRestart).
 *
 * The decisions live in this HAL-free header so the host regression test in
 * tests/dbus_input can exercise them; bsp_uart.cc asserts at compile time that
 * the error masks and the neutral error code below match the board's HAL.
 */

#pragma once

#include <cstdint>

namespace bsp {

/* HAL_UART_ERROR_* bit masks, captured before the HAL forgets them: the ISR flags
 * are cleared first and only ErrorCode keeps the reason. */
constexpr uint32_t kRxErrorParity = 0x01u;  /* HAL_UART_ERROR_PE */
constexpr uint32_t kRxErrorNoise = 0x02u;   /* HAL_UART_ERROR_NE */
constexpr uint32_t kRxErrorFrame = 0x04u;   /* HAL_UART_ERROR_FE */
constexpr uint32_t kRxErrorOverrun = 0x08u; /* HAL_UART_ERROR_ORE */
constexpr uint32_t kRxErrorDma = 0x10u;     /* HAL_UART_ERROR_DMA */

/** @brief "no error" as the HAL encodes it in huart->ErrorCode. */
constexpr uint32_t kRxErrorNone = 0u;

/** @brief Receive-chain bits observed right before a read starts. */
struct RxChainSnapshot {
  bool uart_dma_request;   /* UART CR3.DMAR: DMA requests enabled */
  bool dma_stream_enabled; /* DMA_SxCR.EN: RX stream running */
  bool dma_stream_full;    /* stream stopped after a complete normal-mode transfer */
  bool error_irq_enabled;  /* UART CR3.EIE */
  bool parity_irq_enabled; /* UART CR1.PEIE */
  bool idle_irq_enabled;   /* UART CR1.IDLEIE */
};

/** @brief Repair needed to restore the complete receive chain. */
enum class RxRepair {
  kNone,                  /* chain is armed */
  kRearmRequest,          /* DMAR / error / idle interrupt bits missing, stream running */
  kRearmStreamAndRequest, /* stream stopped (HAL error abort, buffer full) or error pending */
};

/**
 * @brief Decide which part of the receive chain must be re-armed.
 *
 * `error_pending` forces the full repair: the burst that overlapped the error was
 * polluted by it and is discarded, so the stream is restarted from a known state.
 *
 * @param chain          observed chain bits, see RxChainSnapshot
 * @param error_pending  a reception error applies to this read
 * @return repair to apply before trusting any further burst
 */
inline RxRepair PlanRxRepair(const RxChainSnapshot& chain, bool error_pending) {
  if (error_pending || !chain.dma_stream_enabled) return RxRepair::kRearmStreamAndRequest;
  if (!chain.uart_dma_request || !chain.error_irq_enabled || !chain.parity_irq_enabled ||
      !chain.idle_irq_enabled) {
    return RxRepair::kRearmRequest;
  }
  return RxRepair::kNone;
}

/** @brief What the burst a read is about to hand out actually contains. */
enum class RxBurstVerdict {
  kUsable,    /* complete burst, its length describes it */
  kDiscarded, /* torn or aborted burst: no usable length or content */
  kTruncated, /* the burst filled the whole buffer: longer than the buffer */
};

/**
 * @brief Classify the burst of one read from the chain state.
 *
 * A stream that stopped is only truncation when it stopped after a complete
 * normal-mode transfer (RxChainSnapshot::dma_stream_full); every other stop is an
 * aborted transfer whose buffer must not be decoded. An error always wins: the
 * burst that overlapped it is polluted even if the bits look healthy.
 *
 * @param chain          observed chain bits, see RxChainSnapshot
 * @param error_pending  a reception error applies to this read
 */
inline RxBurstVerdict ClassifyRxBurst(const RxChainSnapshot& chain, bool error_pending) {
  if (error_pending) return RxBurstVerdict::kDiscarded;
  if (PlanRxRepair(chain, false) != RxRepair::kRearmStreamAndRequest) {
    return RxBurstVerdict::kUsable;
  }
  return chain.dma_stream_full ? RxBurstVerdict::kTruncated : RxBurstVerdict::kDiscarded;
}

/** @brief DMA handle state a restart has to deal with (subset of HAL_DMA_StateTypeDef). */
enum class RxDmaState {
  kReady,   /* HAL_DMA_STATE_READY: a start call is accepted as is */
  kBusy,    /* HAL_DMA_STATE_BUSY: HAL_DMA_Abort() moves it to READY */
  kAbort,   /* HAL_DMA_STATE_ABORT: left by HAL_DMA_Abort_IT(), stream disabled */
  kUnusable /* HAL_DMA_STATE_RESET / TIMEOUT / ERROR / UNKNOWN: stream not usable */
};

/** @brief What has to happen to the RX DMA handle before it can be started again. */
enum class RxStreamRestart {
  kReadyNow,            /* start the transfer directly */
  kAbortThenStart,      /* HAL_DMA_Abort() first: BUSY -> READY, flags cleared */
  kForceReadyThenStart, /* force State = READY: the HAL refuses both earlier steps */
};

/**
 * @brief How to prepare the RX DMA handle for a restart.
 *
 * HAL_DMA_Abort_IT() leaves HAL_DMA_STATE_ABORT (DMA1/DMA2 streams) whose deferred
 * XferAbortCallback a polling-mode reception never runs, and a failed
 * HAL_DMA_Abort() leaves HAL_DMA_STATE_ERROR. HAL_DMA_Abort() refuses any state
 * but BUSY and HAL_DMA_Start()/HAL_DMAEx_MultiBufferStart() any state but READY,
 * so those states have to be forced back to READY or every re-arm attempt fails
 * and reception stays down until reset.
 *
 * @param state handle state before the restart
 */
inline RxStreamRestart PlanRxStreamRestart(RxDmaState state) {
  switch (state) {
    case RxDmaState::kReady:
      return RxStreamRestart::kReadyNow;
    case RxDmaState::kBusy:
      return RxStreamRestart::kAbortThenStart;
    case RxDmaState::kAbort:
    case RxDmaState::kUnusable:
      return RxStreamRestart::kForceReadyThenStart;
  }
  return RxStreamRestart::kForceReadyThenStart;
}

/**
 * @brief Reception error evidence, counted exactly once per error.
 *
 * Two sources feed it and only one of them may count a given error:
 * - the HAL error callback, whenever the HAL delivers it (Record);
 * - huart->ErrorCode, which the HAL blocking error path never clears (ConsumePending).
 * A DMA reception only reaches the callback through the deferred abort callback
 * described at the top of this file, so in practice the polled source is the one
 * that reports errors on this BSP.
 */
struct RxErrorEvidence {
  volatile uint32_t count = 0;     /* number of errors captured */
  volatile uint32_t last_code = 0; /* bsp::kRxError* mask of the most recent one */
  volatile bool pending = false;   /* an error is waiting for the next read */

  /** @brief record one error delivered by the HAL error callback */
  void Record(uint32_t error_code) {
    ++count;
    last_code = error_code;
    pending = true;
  }

  /**
   * @brief absorb the error state of one read
   *
   * The HAL error path leaves ErrorCode set, so it has to be taken here as well;
   * the pending flag keeps an error the callback already recorded from being
   * counted a second time. When the callback recorded an error and a later one is
   * only visible in ErrorCode before this read, the two collapse into one count:
   * the count is diagnostic, the discard decision of this read is the same either
   * way.
   *
   * @param hal_error_code  huart->ErrorCode as left by the HAL, kRxErrorNone if none
   * @return true when this read has to treat its burst as polluted
   */
  bool ConsumePending(uint32_t hal_error_code) {
    const bool callback_pending = pending;
    if (!callback_pending && hal_error_code != kRxErrorNone) {
      ++count;
      last_code = hal_error_code;
    }
    pending = false;
    return callback_pending || hal_error_code != kRxErrorNone;
  }
};

/**
 * @brief DMA_SxCR_CT value that makes a double-buffered RX stream write rx_data_[index].
 *
 * The two buffers keep their physical place (M0AR = rx_data_[0], M1AR =
 * rx_data_[1]) and CT selects the one the stream fills. bsp::UART::Read() flips
 * CT together with rx_index_ on every burst, so "the stream fills rx_data_[index]"
 * is CT == (index != 0); a restart has to restore exactly that relationship, or
 * the DMA writes the buffer the next read hands out.
 *
 * @param rx_index  index of the buffer the stream must fill (0 or 1)
 */
inline bool RxDoubleBufferCtForIndex(uint8_t rx_index) { return rx_index != 0u; }

}  // namespace bsp
