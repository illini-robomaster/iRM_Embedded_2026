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

#include "bsp_fdcan.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "dbus.h"
#include "motor.h"
#include "omni.h"
#include "protocol.h"
#include "spi.h"

#define RX_SIGNAL (1 << 0)
#define REFEREE_RX_SIGNAL (1 << 1)

#define YAW_MOTOR_OFFSET 4.54f;                     // in rad
#define PITCH_PHYSICAL_MIN -24.0f * PI / 180.0f;    // -24 degrees in physical position
#define PITCH_PHYSICAL_OFFSET 33.0f * PI / 180.0f;  // 0 in encoder is 33 degrees in physical position, also physical maximum

typedef struct {
  bool enabled;
  bool dead;
  bool spinning;
  bool just_revived;
} status_t;

// IMU related stuff not added yet since negligible role in the Infantry robot at the moment.

const osThreadAttr_t refereeTaskAttribute = {
    .name = "refereeTask",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
    .tz_module = 0,
    .reserved = 0};

osThreadId_t refereeTaskHandle;
class RefereeUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  void RxCompleteCallback() final { osThreadFlagsSet(refereeTaskHandle, REFEREE_RX_SIGNAL); }
};

static communication::Referee* referee = nullptr;
static RefereeUART* referee_uart = nullptr;

void refereeTask(void* arg) {
  UNUSED(arg);

  while (true) {
    uint32_t length;
    uint8_t* data;
    uint32_t flags = osThreadFlagsWait(REFEREE_RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & REFEREE_RX_SIGNAL) {
      length = referee_uart->Read(&data);
      referee->Receive(communication::package_t{data, (int)length});
    }
  }
}

void RM_RTOS_Threads_Init(void) {
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
}

// Automatically aliased to FDCAN.
bsp::CAN* can = nullptr;
control::MotorDM3519* motor[4];
control::Motor6020* yaw_motor;
control::Motor4310* pitch_motor;
remote::DBUS* dbus = nullptr;

float original_max_output_vel_rad_per_s = 395 / 60.0f * 2 * PI;  // 395 rpm = 41.36 rad/s
float original_gear_ratio = 3591.0f / 187.0f;
float new_gear_ratio = 268.0f / 17.0f;

control::omni_t omni_data;
control::Omni* omni;

void RM_RTOS_Init() {
  // Print with USB
  print_use_usb();
  can = new bsp::CAN(&hfdcan1, 0);

  /* rx_id = Master id
   * tx_id = CAN id
   * mode:
   *  MIT: MIT mode
   *  POS_VEL: position-velocity mode
   *  VEL: velocity mode  */
  
  /* Make sure motor is set to the correct mode (in helper tool). Otherwise, motor won't start */
  motor[0] = new control::MotorDM3519(can, 0x00, 0x01, control::VEL);  // front right
  motor[1] = new control::MotorDM3519(can, 0x02, 0x03, control::VEL);  // back left
  motor[2] = new control::MotorDM3519(can, 0x08, 0x09, control::VEL);  // back right
  motor[3] = new control::MotorDM3519(can, 0x06, 0x07, control::VEL);  // front left
  yaw_motor = new control::Motor6020(can, 0x205);
  pitch_motor = new control::Motor4310(can, 0x0D, 0x0C, control::MIT);
  dbus = new remote::DBUS(&huart3);

  // Referee initialization
  
  // Referee UART changed to UART1
  referee_uart = new RefereeUART(&huart1);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);
  referee = new communication::Referee;
  

  // Omni class
  float pid_params[3] = {20000.0, 1.0, 10.0};
  control::ConstrainedPID pid_(pid_params, 30000, 30000);

  omni_data.motor0 = motor[0];
  omni_data.motor1 = motor[1];
  omni_data.motor2 = motor[2];
  omni_data.motor3 = motor[3];
  omni_data.pitch_motor = pitch_motor;
  omni_data.yaw_motor = yaw_motor;
  omni_data.pitch_params[0] = 30;
  omni_data.pitch_params[1] = 0.5;
  omni_data.pitch_params[2] = 0;
  omni_data.yaw_offset = YAW_MOTOR_OFFSET;
  omni_data.pitch_min = PITCH_PHYSICAL_MIN;
  omni_data.pitch_max = PITCH_PHYSICAL_OFFSET;
  omni_data.pid = pid_;

  omni = new control::Omni(&omni_data);
}

void RM_RTOS_Default_Task(const void* args) {
  /* press reset if no response */
  UNUSED(args);

  status_t status;
  status.enabled = false;
  status.dead = false;
  status.spinning = false;
  status.just_revived = false;

  // Flip swr to start.
  while (dbus->swr != remote::DOWN) {
    status.enabled = true;
  }

  osDelay(1000);
  /* In control::Omni::Init -
   * Use SetZeroPos if you want to set current motor position as zero position.
   * If uncommented, the zero position is the zero position set before.
   */
  omni->Init();
  omni->ChassisMotorsEnable();
  omni->GimbalMotorsEnable();

  osDelay(100);

  uint32_t print_tick = HAL_GetTick();

  while (true) {
    /*
     * SWITCHES START
     */
    // Flip swr down to disable.
    if (dbus->swr != remote::DOWN) {
      if (status.enabled) {
        omni->ChassisMotorsDisable();
        omni->GimbalMotorsDisable();
        status.enabled = false;
        status.spinning = false;  // Stop spinning.
      }
      osDelay(100);
      continue;
    } else {
      if (!status.enabled) {
        omni->ChassisMotorsEnable();
        omni->GimbalMotorsEnable();
        status.enabled = true;
        status.just_revived = true;
      }
    }

    // Flip swl down to spin.
    if (dbus->swr != remote::DOWN) {
      if (!status.spinning && !status.just_revived) {
        omni->Spin(15.0f);
        status.spinning = true;
      }
    } else {
      if (status.spinning) {
        omni->Spin(0.0f);
        status.spinning = false;
      }
      status.just_revived = false;
    }
    /*
     * SWITCHES END
     */

    /*
     * STICKS START
     */
    float vx = clip<float>(dbus->ch1 / 660.0 * 30.0, -30, 30);       // left
    float vy = -clip<float>(dbus->ch0 / 660.0 * 30.0, -30, 30);      // forward
    float w_pitch = clip<float>(dbus->ch3 / 660.0 * 15.0, -15, 15);  // pitch
    float w_yaw = clip<float>(-dbus->ch2 / 660.0 * 30.0, -30, 30);   // yaw
    /*
     * STICKS END
     */

    omni->SetVelocity(vx, vy);
    omni->SetGimbalVelocity(w_pitch, w_yaw);
    omni->Transmit();

    osDelay(10);

    set_cursor(0, 0);
    clear_screen();
    
    uint32_t now = HAL_GetTick();
    // Print at 10Hz only (every 100ms)
    if (now - print_tick >= 100) {
      print_tick = now;

      print("test\r\n");

      print("look yaw: %.2f look pitch: %.2f \r\n", omni->yaw, omni->pitch);
    }
  }
}
