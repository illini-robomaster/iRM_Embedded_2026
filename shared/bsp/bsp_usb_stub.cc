/**
 * @brief USB implementation for DM_MC_02 board (USB HS mode)
 */

#include "bsp_error_handler.h"
#include "bsp_usb.h"
#include "cmsis_os.h"
#include "task.h"
#include "usbd_cdc_if.h"

static bsp::VirtualUSB* usb = nullptr;

namespace bsp {

VirtualUSB::VirtualUSB()
    : rx_size_(0),
      rx_pending_(0),
      rx_write_(nullptr),
      rx_read_(nullptr),
      tx_size_(0),
      tx_pending_(0),
      tx_write_(nullptr),
      tx_read_(nullptr) {
  RM_ASSERT_FALSE(usb, "usb initialized twice");
  usb = this;
}

VirtualUSB::~VirtualUSB() {
  if (rx_write_) delete[] rx_write_;
  if (rx_read_) delete[] rx_read_;
  if (tx_write_) delete[] tx_write_;
  if (tx_read_) delete[] tx_read_;
  usb = nullptr;
}

void VirtualUSB::SetupTx(uint32_t tx_buffer_size) {
  if (tx_size_ || tx_write_ || tx_read_) return;

  tx_size_ = tx_buffer_size;
  tx_pending_ = 0;
  tx_write_ = new uint8_t[tx_buffer_size];
  tx_read_ = new uint8_t[tx_buffer_size];
}

void VirtualUSB::SetupRx(uint32_t rx_buffer_size) {
  if (rx_size_ || rx_write_ || rx_read_) return;

  rx_size_ = rx_buffer_size;
  rx_pending_ = 0;
  rx_write_ = new uint8_t[rx_buffer_size];
  rx_read_ = new uint8_t[rx_buffer_size];
}

uint32_t VirtualUSB::Read(uint8_t** data) {
  taskENTER_CRITICAL();
  uint32_t length = rx_pending_;
  *data = rx_write_;
  uint8_t* tmp = rx_write_;
  rx_write_ = rx_read_;
  rx_read_ = tmp;
  rx_pending_ = 0;
  taskEXIT_CRITICAL();
  return length;
}

uint32_t VirtualUSB::Write(uint8_t* data, uint32_t length) {
  taskENTER_CRITICAL();
  if (length > tx_size_) length = tx_size_;
  // Use CDC_Transmit_HS for USB High Speed mode
  uint8_t status = CDC_Transmit_HS(data, length);
  if (status == USBD_BUSY || tx_pending_) {
    if (length + tx_pending_ > tx_size_) {
      length = tx_size_ - tx_pending_;
    }
    memcpy(tx_write_ + tx_pending_, data, length);
    tx_pending_ += length;
  }
  taskEXIT_CRITICAL();
  return length;
}

void VirtualUSB::TxCompleteCallback() {
  uint8_t* tmp;
  UBaseType_t isrflags = taskENTER_CRITICAL_FROM_ISR();
  if (tx_pending_) {
    tmp = tx_read_;
    tx_read_ = tx_write_;
    tx_write_ = tmp;
    CDC_Transmit_HS(tx_read_, tx_pending_);
    tx_pending_ = 0;
  }
  taskEXIT_CRITICAL_FROM_ISR(isrflags);
}

void VirtualUSB::RxCompleteCallback() {}

uint32_t VirtualUSB::QueueUpRxData(const uint8_t* data, uint32_t length) {
  if (length + rx_pending_ > rx_size_) {
    length = rx_size_ - rx_pending_;
  }
  memcpy(rx_write_ + rx_pending_, data, length);
  rx_pending_ += length;
  return length;
}

void TxCompleteCallbackWrapper() {
  if (usb) usb->TxCompleteCallback();
}

void RxCompleteCallbackWrapper(uint8_t* data, uint32_t length) {
  if (usb) {
    usb->QueueUpRxData(data, length);
    usb->RxCompleteCallback();
  }
}

}  // namespace bsp

extern "C" void RM_USB_TxCplt_Callback() { bsp::TxCompleteCallbackWrapper(); }

extern "C" void RM_USB_RxCplt_Callback(uint8_t* data, uint32_t length) {
  bsp::RxCompleteCallbackWrapper(data, length);
}
