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

namespace bsp {

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

  // TODO(alvin): add actual error handler in the future, ignore it for now
  if (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR)) {
    __HAL_UART_CLEAR_PEFLAG(huart);
  }
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

  /* enable uart rx dma transfer in back ground */
  UartStartDmaNoInt(huart_, rx_data_[0], rx_data_[1], rx_size_);

  /* UART IDLE Interrupt can notify application of data reception ASAP */
  __HAL_UART_CLEAR_FLAG(huart_, UART_FLAG_IDLE);
  __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);
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
  /* capture pending bytes and perform hardware buffer switch */
  int32_t length;

  // enter critical session
  UBaseType_t isrflags;
  if (FromISR) {
    isrflags = taskENTER_CRITICAL_FROM_ISR();
  } else {
    taskENTER_CRITICAL();
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

  // exit critical session
  if (FromISR) {
    taskEXIT_CRITICAL_FROM_ISR(isrflags);
  } else {
    taskEXIT_CRITICAL();
  }

  /* return the buffer pointer currently not being used by DMA transfer */
  *data = rx_data_[1 - rx_index_];

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

} /* namespace bsp */

/* overwrite the weak function defined in board specific usart.c to handle IRQ requests */
extern "C" void RM_UART_IRQHandler(UART_HandleTypeDef* huart) { bsp::RxCompleteCallbackWrapper(huart); }
