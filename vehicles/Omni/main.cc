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

#include "bsp_print.h"
#include "bsp_imu.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"
#include "i2c.h"
#include "spi.h"
#include "dbus.h"


#define RX_SIGNAL (1 << 0)

const osThreadAttr_t imuTaskAttribute = {.name = "imuTask",
                                         .attr_bits = osThreadDetached,
                                         .cb_mem = nullptr,
                                         .cb_size = 0,
                                         .stack_mem = nullptr,
                                         .stack_size = 256 * 4,
                                         .priority = (osPriority_t)osPriorityNormal,
                                         .tz_module = 0,
                                         .reserved = 0};
osThreadId_t imuTaskHandle;

class IMU : public bsp::IMU_typeC {
 public:
  using bsp::IMU_typeC::IMU_typeC;

 protected:
  void RxCompleteCallback() final { osThreadFlagsSet(imuTaskHandle, RX_SIGNAL); }
};

static IMU* imu = nullptr;

void imuTask(void* arg) {
  UNUSED(arg);

  while (true) {
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {  // unnecessary check
      imu->Update();
    }
  }
}

void RM_RTOS_Threads_Init(void) {
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
}

bsp::CAN* can = nullptr;
control::MotorDM3519* motor[4];
control::Motor6020* yaw_motor;
control::Motor4310* pitch_motor;
remote::DBUS* dbus = nullptr;

float original_max_output_vel_rad_per_s = 395/60.0f*2*PI; //395 rpm = 41.36 rad/s
float original_gear_ratio = 3591.0f/187.0f; 
float new_gear_ratio = 268.0f/17.0f; 
const float YAW_MOTOR_OFFSET = 4.54f; // in rad
const float PITCH_PHYSICAL_OFFSET = 33.0f * PI / 180.0f; // 0 in encoder is 33 degrees in physical position, also physical maximum
const float PITCH_PHYSICAL_MIN = -24.0f * PI / 180.0f; // -24 degrees in physical position

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  can = new bsp::CAN(&hcan1, true);

  /* rx_id = Master id
   * tx_id = CAN id
   * mode:
   *  MIT: MIT mode
   *  POS_VEL: position-velocity mode
   *  VEL: velocity mode  */

  /* Make sure motor is set to the correct mode (in helper tool). Otherwise, motor won't start */
  motor[0] = new control::MotorDM3519(can, 0x00, 0x01, control::VEL); // front right
  motor[1] = new control::MotorDM3519(can, 0x02, 0x03, control::VEL); // back left
  motor[2] = new control::MotorDM3519(can, 0x08, 0x09, control::VEL); // back right
  motor[3] = new control::MotorDM3519(can, 0x06, 0x07, control::VEL); // front left
  yaw_motor = new control::Motor6020(can, 0x205);
  pitch_motor = new control::Motor4310(can, 0x0D, 0x0C, control::MIT);
  dbus = new remote::DBUS(&huart3);

  // IMU initialization
  bsp::IST8310_init_t IST8310_init;
  IST8310_init.hi2c = &hi2c3;
  IST8310_init.int_pin = DRDY_IST8310_Pin;
  IST8310_init.rst_group = GPIOG;
  IST8310_init.rst_pin = GPIO_PIN_6;
  bsp::BMI088_init_t BMI088_init;
  BMI088_init.hspi = &hspi1;
  BMI088_init.CS_ACCEL_Port = CS1_ACCEL_GPIO_Port;
  BMI088_init.CS_ACCEL_Pin = CS1_ACCEL_Pin;
  BMI088_init.CS_GYRO_Port = CS1_GYRO_GPIO_Port;
  BMI088_init.CS_GYRO_Pin = CS1_GYRO_Pin;
  bsp::heater_init_t heater_init;
  heater_init.htim = &htim10;
  heater_init.channel = 1;
  heater_init.clock_freq = 1000000;
  heater_init.temp = 45;
  bsp::IMU_typeC_init_t imu_init;
  imu_init.IST8310 = IST8310_init;
  imu_init.BMI088 = BMI088_init;
  imu_init.heater = heater_init;
  imu_init.hspi = &hspi1;
  imu_init.hdma_spi_rx = &hdma_spi1_rx;
  imu_init.hdma_spi_tx = &hdma_spi1_tx;
  imu_init.Accel_INT_pin_ = INT1_ACCEL_Pin;
  imu_init.Gyro_INT_pin_ = INT1_GYRO_Pin;
  imu = new IMU(imu_init, false);
}

void RM_RTOS_Default_Task(const void* args) {
  /* press reset if no response */
  UNUSED(args);

  control::MotorDM3519* motors[] = {motor[0], motor[1], motor[2], motor[3]};
  control::Motor4310* pitch_motors[] = {pitch_motor};
  float pid_params[3] = {20000.0, 1.0, 10.0};
  control::ConstrainedPID pid_(pid_params, 30000, 30000);

  while(dbus->swr != remote::DOWN){}  // flip swr to start

  /* Use SetZeroPos if you want to set current motor position as zero position. If uncommented, the
   * zero position is the zero position set before */
  osDelay(1000);
  motor[0]->SetZeroPos();
  motor[0]->MotorEnable();
  motor[1]->SetZeroPos();
  motor[1]->MotorEnable();
  motor[2]->SetZeroPos();
  motor[2]->MotorEnable();
  motor[3]->SetZeroPos();
  motor[3]->MotorEnable();
  imu->Calibrate();
  // pitch_motor->SetZeroPos(); // pitch motor zero pos, comment if calibrated already
  pitch_motor->MotorEnable();
  float pitch_physical = pitch_motor->GetTheta() + PITCH_PHYSICAL_OFFSET;
  osDelay(100);

  bool enabled = true;
  float yaw_target = imu->INS_angle[0] + yaw_motor->GetTheta() - YAW_MOTOR_OFFSET;
  int16_t command;
  float gimbal_yaw_measured_in_field_reference = 0.0f;

  while (true) {

    // disable logic
    if(dbus->swr != remote::DOWN){
      if(enabled){
        motor[0]->MotorDisable();
        motor[1]->MotorDisable();
        motor[2]->MotorDisable();
        motor[3]->MotorDisable();  
        pitch_motor->MotorDisable();
        enabled = false;
      }
      osDelay(100);
      continue;
    }else{
      if(!enabled){
        motor[0]->MotorEnable();
        motor[1]->MotorEnable();
        motor[2]->MotorEnable();
        motor[3]->MotorEnable();
        pitch_motor->MotorEnable();
        enabled = true;
      }
    }
    gimbal_yaw_measured_in_field_reference = imu->INS_angle[0] + yaw_motor->GetTheta() - YAW_MOTOR_OFFSET;

    float vel[4];

    float y = -clip<float>(dbus->ch0 / 660.0 * 30.0, -30, 30); // forward
    float x = clip<float>(dbus->ch1 / 660.0 * 30.0, -30, 30); // left
    float yaw_omega = clip<float>(-dbus->ch2 / 660.0 * 30.0, -30, 30); // yaw
    float pitch_omega = clip<float>(dbus->ch3 / 660.0 * 15.0, -15, 15); // pitch
    yaw_target += yaw_omega * 0.01f; // yaw target in field reference rad * 10ms
    pitch_physical += pitch_omega /200;
    pitch_physical = clip<float>(pitch_physical, PITCH_PHYSICAL_MIN, PITCH_PHYSICAL_OFFSET);

    float delta_yaw = yaw_target - gimbal_yaw_measured_in_field_reference;
    float cos = cosf(delta_yaw);
    float sin = sinf(delta_yaw);
    delta_yaw = atan2f(sin, cos); // wrap to [-pi, pi]
    command = pid_.ComputeOutput(delta_yaw);

    float chasis_yaw_measured_in_field_reference = gimbal_yaw_measured_in_field_reference - imu->INS_angle[0]; // in rad

    // rotate the x,y according to the robot's heading
    float temp_x = x * cosf(chasis_yaw_measured_in_field_reference) - y * sinf(chasis_yaw_measured_in_field_reference);
    float temp_y = x * sinf(chasis_yaw_measured_in_field_reference) + y * cosf(chasis_yaw_measured_in_field_reference);
    x = temp_x;
    y = temp_y;

    // alpha, beta equals to x y rotated by -45 degrees

    float alpha = (x - y) / 1.4142f;
    float beta = (y + x) / 1.4142f;

    // max output for omega while ensure translation
    float max_omega = abs(30.0f - max(fabsf(alpha), fabsf(beta)));
    float chassis_yaw_omega_target = dbus->swl == remote::UP ? 15.0f : 0.0f;
    chassis_yaw_omega_target = clip<float>(chassis_yaw_omega_target, -max_omega, max_omega);

    vel[0] = -beta + chassis_yaw_omega_target;
    vel[1] = beta + chassis_yaw_omega_target;
    vel[2] = -alpha + chassis_yaw_omega_target;
    vel[3] = alpha + chassis_yaw_omega_target;

    set_cursor(0, 0);
    clear_screen();

    print("g_f: %.2f c_f: %.2f \r\n", gimbal_yaw_measured_in_field_reference, chasis_yaw_measured_in_field_reference);
    UNUSED(command);
    motor[0]->SetOutput(vel[0]);
    motor[1]->SetOutput(vel[1]);
    motor[2]->SetOutput(vel[2]);
    motor[3]->SetOutput(vel[3]);
    float yaw_kF = 400.0f;
    yaw_motor->SetOutput(command + chassis_yaw_omega_target * yaw_kF);

    float pitch_rotor = pitch_physical - PITCH_PHYSICAL_OFFSET;
    pitch_motor->SetOutput(pitch_rotor, pitch_omega, 30, 0.5, 0);
    control::MotorDM3519::TransmitOutput(motors, 4);
    control::Motor6020::TransmitOutput((control::MotorCANBase**)(&yaw_motor), 1);
    control::Motor4310::TransmitOutput(pitch_motors, 1);
    // after transmit, DM's motors return their data
    osDelay(10);
  }
}
