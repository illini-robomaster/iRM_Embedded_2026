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

#include "main.h"

#include <cstring>

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "weighscale.h"

// CAN bus instance
static bsp::CAN* can1 = nullptr;

// Weighing scale instance
static control::WeighScale* scale = nullptr;

// Weight data storage
static control::WeighScaleData_t weight_data;

// Number of channels on the weighing transmitter
static const uint8_t NUM_CHANNELS = 4;

// Button for user interaction (K1 on TypeA board)
static bsp::GPIO* key = nullptr;

// Callback to handle weight response frames
// Response IDs: 0x302 (ch1,2), 0x303 (ch3,4), etc.
static void weight_callback(const uint8_t data[], void* args) {
  UNUSED(args);
  // Parse two channels from each frame (big-endian uint32)
  // The callback ID tells us which channels these are
  // For simplicity, this example stores to global weight_data
  
  // Parse channel N weight (bytes 0-3)
  uint32_t weight1 = (static_cast<uint32_t>(data[0]) << 24) |
                     (static_cast<uint32_t>(data[1]) << 16) |
                     (static_cast<uint32_t>(data[2]) << 8) |
                     static_cast<uint32_t>(data[3]);
  
  // Parse channel N+1 weight (bytes 4-7)
  uint32_t weight2 = (static_cast<uint32_t>(data[4]) << 24) |
                     (static_cast<uint32_t>(data[5]) << 16) |
                     (static_cast<uint32_t>(data[6]) << 8) |
                     static_cast<uint32_t>(data[7]);
  
  // Store weights - actual channel mapping depends on response ID
  // This is a simplified example
  static uint8_t frame_count = 0;
  uint8_t base_ch = frame_count * 2;
  if (base_ch < control::WEIGHSCALE_MAX_CHANNELS) {
    weight_data.weight[base_ch] = weight1;
    if (base_ch + 1 < control::WEIGHSCALE_MAX_CHANNELS) {
      weight_data.weight[base_ch + 1] = weight2;
    }
  }
  frame_count++;
  if (frame_count >= (NUM_CHANNELS + 1) / 2) {
    frame_count = 0;
    weight_data.valid_channels = NUM_CHANNELS;
  }
}

void RM_RTOS_Init(void) {
  // Initialize print output (UART8 on TypeA)
  print_use_uart(&huart8);
  
  // Initialize CAN1
  can1 = new bsp::CAN(&hcan1, true);
  
  // Initialize weighing scale with address 1, standard frame
  scale = new control::WeighScale(can1, 1, control::WeighScaleFrameType::STANDARD);
  
  // Register callbacks for weight response frames
  // Response IDs for 4-channel device: 0x302 (ch1,2), 0x303 (ch3,4)
  can1->RegisterRxCallback(0x302, weight_callback, nullptr);
  can1->RegisterRxCallback(0x303, weight_callback, nullptr);
  
  // Initialize button (K1 on TypeA: GPIOE PIN6)
  key = new bsp::GPIO(K1_GPIO_Port, K1_Pin);
  
  // Clear weight data
  memset(&weight_data, 0, sizeof(weight_data));
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);
  
  osDelay(500);  // Wait for system to stabilize
  
  print("=== WeighScale Example ===\r\n");
  print("Device Address: 1\r\n");
  print("Frame Type: Standard (11-bit ID)\r\n");
  print("CAN Baud Rate: 1Mbps\r\n");
  print("Channels: %d\r\n", NUM_CHANNELS);
  print("\r\n");
  print("Press K1 button to tare all channels\r\n");
  print("\r\n");
  
  // Set device baud rate to 1Mbps (requires power cycle to take effect)
  // Only need to do this once if device was configured differently
  // scale->WriteBaudCode(control::WeighScaleBaudCode::BAUD_1M);
  // osDelay(10);
  
  // Set sample rate to 40Hz for all channels
  for (uint8_t ch = 1; ch <= NUM_CHANNELS; ch++) {
    scale->SetSampleRate(ch, control::WeighScaleSampleRate::RATE_40HZ);
    osDelay(10);
  }
  print("Sample rate set to 40Hz\r\n\r\n");
  
  uint32_t loop_count = 0;
  bool last_key_state = false;
  
  while (true) {
    // Check button press for tare
    bool key_pressed = (key->Read() == 0);  // Active low
    if (key_pressed && !last_key_state) {
      print("Taring all channels...\r\n");
      scale->Tare(control::WEIGHSCALE_ALL_CHANNELS);
      osDelay(100);
    }
    last_key_state = key_pressed;
    
    // Request weight readings every 100ms
    if (loop_count % 10 == 0) {
      scale->ReadWeights(&weight_data, NUM_CHANNELS);
    }
    
    // Print weight values every 500ms
    if (loop_count % 50 == 0) {
      set_cursor(0, 0);
      clear_screen();
      
      print("=== Weight Readings (g) ===\r\n");
      for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
        print("CH%d: %lu g\r\n", ch + 1, weight_data.weight[ch]);
      }
      print("\r\n");
      print("Connection: %s\r\n", scale->connection_flag_ ? "OK" : "---");
      print("Loop: %lu\r\n", loop_count);
    }
    
    osDelay(10);
    loop_count++;
  }
}
