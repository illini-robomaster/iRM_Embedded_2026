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

#include "unitree_motor.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "sbus.h"
#include "bsp_gpio.h"

static control::UnitreeMotor* A1 = nullptr;
static remote::SBUS* dbus;
bsp::GPIO* user_key = nullptr;
bsp::GPIO* rs485_1_dir = nullptr;
bsp::GPIO* rs485_2_dir = nullptr;

#define RX_SIGNAL (1 << 0)

const osThreadAttr_t A1TaskAttribute = {.name = "A1Task",
                                       .attr_bits = osThreadDetached,
                                       .cb_mem = nullptr,
                                       .cb_size = 0,
                                       .stack_mem = nullptr,
                                       .stack_size = 128 * 4,
                                       .priority = (osPriority_t)osPriorityNormal,
                                       .tz_module = 0,
                                       .reserved = 0};

osThreadId_t A1TaskHandle;

class CustomUART: public bsp::UART {
public:
  using bsp::UART::UART;

protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() override final { osThreadFlagsSet(A1TaskHandle, RX_SIGNAL); }
};

static CustomUART* A1_write_uart = nullptr; // uart for sending commands to A1
static CustomUART* A1_read_uart = nullptr; // uart for receiving A1 information returned


void A1Task(void* arg) {
  UNUSED(arg);
  uint32_t length;
  uint8_t* data;

  while (true) {
    /* wait until rx data is available */
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    // print("data received\r\n");
    if (flags & RX_SIGNAL) {  // unnecessary check
      /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
      length = A1_read_uart->Read(&data);
      if ((int)length == A1->recv_length){
        A1->ExtractData(communication::package_t{data, (int)length});
        print("data length matches\r\n");
      }else{
        print("data length is %d, but need %d\r\n", length, A1->recv_length);
      }
      print("timestamp: %d", HAL_GetTick());
    }
  }
}

void RM_RTOS_Init(void) {
  print_use_uart(&huart4);
  print("Unitree Motor Test\r\n");

  A1_write_uart = new CustomUART(&huart1);
  A1_write_uart->SetupRx(300);
  A1_write_uart->SetupTx(300);

  A1_read_uart = new CustomUART(&huart2);
  A1_read_uart->SetupRx(300);
  A1_read_uart->SetupTx(300);

  A1 = new control::UnitreeMotor();
  dbus = new remote::SBUS(&huart3);
  user_key = new bsp::GPIO(GPIOA, GPIO_PIN_15);
  rs485_1_dir = new bsp::GPIO(GPIOC, GPIO_PIN_15);
  rs485_1_dir->High(); 
  rs485_2_dir = new bsp::GPIO(GPIOB, GPIO_PIN_3);
  rs485_2_dir->Low(); //rs485_1 in receive mode
}

void RM_RTOS_Threads_Init(void) {
  A1TaskHandle = osThreadNew(A1Task, nullptr, &A1TaskAttribute);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  print("Press key to start\r\n");
  while(user_key->Read()); // wait for key press
  print("Open loop spin test\r\n");
  // A1->Test(2);
  // A1_write_uart->Write((uint8_t*)(&(A1->send[2].data)), A1->send_length);
  // osDelay(3000);

  // print("Motor stop\r\n");
  // A1->Stop(2);
  // A1_write_uart->Write((uint8_t*)(&(A1->send[2].data)), A1->send_length);
  // osDelay(3000);

  // print("Constant speed mode\r\n");
  // A1->Control(2, 0.0, -1.0, 0.0, 0.0, 3.0); // constant speed mode
  // A1_write_uart->Write((uint8_t*)(&(A1->send[2].data)), A1->send_length);
  // osDelay(3000);

  // print("Motor stop\r\n");
  // A1->Stop(2);
  // A1_write_uart->Write((uint8_t*)(&(A1->send[2].data)), A1->send_length);
  // osDelay(3000);


  print("Position control mode (with remote control)\r\n");
  float pos = 0.0;
  while (true) {
    set_cursor(0, 0);
    clear_screen();
    // float vel;
    // vel = dbus->ch[1] / 660.0 * 15;
    // vel = 100 / 660.0 * 15;
    A1->Control(2, 0.0, 0.0, pos, 0.05, 3.0);
    A1_write_uart->Write((uint8_t*)(&(A1->send[2].data)), A1->send_length);
    print("command sent\r\n");
  
    // print("Motor ID: %d\r\n", A1->recv[2].id);
    // print("Mode    : %d\r\n", A1->recv[2].mode);
    // print("Temp    : %d\r\n", A1->recv[2].Temp);
    // print("MError  : %d\r\n", A1->recv[2].MError);
    // print("Torque  : %.3f\r\n", A1->recv[2].T);
    // print("Speed   : %.3f\r\n", A1->recv[2].W);
    // print("Accel   : %d\r\n", A1->recv[2].Acc);
    // print("Position: %.3f\r\n", A1->recv[2].Pos);
    osDelay(10);
  }

}
