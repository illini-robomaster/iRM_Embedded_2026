/**
 * @brief USB stub for DM_MC_02 board
 * 
 * The DM_MC_02 board uses a different USB implementation.
 * This file provides stub functions to satisfy linker dependencies from bsp_print.cc
 */

#include "bsp_usb.h"

namespace bsp {

// Stub implementation - USB not yet supported on DM_MC_02
VirtualUSB::VirtualUSB() {}
VirtualUSB::~VirtualUSB() {}
void VirtualUSB::SetupTx(uint32_t tx_buffer_size) { (void)tx_buffer_size; }
void VirtualUSB::SetupRx(uint32_t rx_buffer_size) { (void)rx_buffer_size; }
uint32_t VirtualUSB::Read(uint8_t** data) { (void)data; return 0; }
uint32_t VirtualUSB::Write(uint8_t* data, uint32_t length) { (void)data; (void)length; return 0; }
void VirtualUSB::RxCompleteCallback() {}
void VirtualUSB::TxCompleteCallback() {}

}  // namespace bsp
