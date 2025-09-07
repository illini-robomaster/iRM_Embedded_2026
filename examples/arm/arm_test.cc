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

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "unitree_motor.h"
#include "utils.h"

#define KEY_GPIO_GROUP GPIOA
#define KEY_GPIO_PIN GPIO_PIN_0

#define RX_SIGNAL (1 << 0)

// Motor enable list
// TODO enable motors to test them
// #define BASE_TRANSLATE_MOTOR_ENABLE
#define FOREARM_ROTATE_MOTOR_ENABLE
#define ELBOW_ROTATE_MOTOR_ENABLE

constexpr float RUN_SPEED = (2 * PI);
constexpr float ALIGN_SPEED = (0.5 * PI);
constexpr float ACCELERATION = (100 * PI);

bsp::CAN* can1 = nullptr;

#ifdef BASE_TRANSLATE_MOTOR_ENABLE
static int base_translate_motor_enable = 1;
#else
static int base_translate_motor_enable = 0;
#endif

#ifdef FOREARM_ROTATE_MOTOR_ENABLE
static int forearm_rotate_motor_enable = 1;
#else
static int forearm_rotate_motor_enable = 0;
#endif


#ifdef ELBOW_ROTATE_MOTOR_ENABLE
static int elbow_rotate_motor_enable = 1;
#else
static int elbow_rotate_motor_enable = 0;
#endif

bsp::GPIO* key = nullptr;
BoolEdgeDetector key_detector(false);

/**
 * define params for each motor
**/
#ifdef BASE_TRANSLATE_MOTOR_ENABLE

control::MotorCANBase* motor1 = nullptr;
control::SteeringMotor* base_translate_motor = nullptr;

bsp::GPIO* base_translate_pe_sensor = nullptr;

bool base_translate_align_detect() {
  return base_translate_pe_sensor->Read() == 1;
}
#endif

#ifdef FOREARM_ROTATE_MOTOR_ENABLE
constexpr float M4310_VEL = 0.25; // magic number, see m4310_mit example
static control::Motor4310* forearm_rotate_motor = nullptr;
static control::Motor4310* forearm_rotate_motors[] = {};
#endif

#ifdef ELBOW_ROTATE_MOTOR_ENABLE
// TODO this part is unstable
extern osThreadId_t defaultTaskHandle;
class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;
 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() override final { osThreadFlagsSet(defaultTaskHandle, RX_SIGNAL); }
};
constexpr int ELBOW_MOTOR_ID = 1;
static control::UnitreeMotor* elbow_rotate_motor = nullptr;
static CustomUART* elbow_rotate_motor_uart = nullptr;
#endif

/**
 * define params ends
**/

void RM_RTOS_Init() {
  print_use_uart(&huart6);
  bsp::SetHighresClockTimer(&htim5);

  /* Usage:
   *   The 'key' is the white button on TypeC boards
  **/
  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);


  can1 = new bsp::CAN(&hcan1, true);

  #ifdef BASE_TRANSLATE_MOTOR_ENABLE
  control::steering_t steering_data;
  motor1 = new control::Motor3508(can1, 0x201);

  // TODO assign a GPIO pin to the PE sensor
  base_translate_pe_sensor = new bsp::GPIO(GPIOC, GPIO_PIN_2);

  steering_data.motor = motor1;
  steering_data.max_speed = RUN_SPEED;
  steering_data.max_acceleration = ACCELERATION;
  // TODO make sure the gear ratio is correct
  steering_data.transmission_ratio = M3508P19_RATIO;
  steering_data.omega_pid_param = new float[3]{140, 1.2, 25};
  steering_data.max_iout = 1000;
  steering_data.max_out = 13000;
  // TODO measure the calibrate offset for base translate motor
  steering_data.calibrate_offset = 0;
  steering_data.align_detect_func = base_translate_align_detect;
  base_translate_motor = new control::SteeringMotor(steering_data);
  #endif

  #ifdef FOREARM_ROTATE_MOTOR_ENABLE
  static const int RX_ID = 0x02;
  static const int TX_ID = 0x01;

  /* rx_id = Master id
   * tx_id = CAN id
   * see example/m4310_mit.cc
   */
  forearm_rotate_motor = new control::Motor4310(can1, RX_ID, TX_ID, control::MIT);
  #endif


  #ifdef ELBOW_ROTATE_MOTOR_ENABLE
  elbow_rotate_motor_uart = new CustomUART(&huart1);
  elbow_rotate_motor_uart->SetupRx(300);
  elbow_rotate_motor_uart->SetupTx(300);
  elbow_rotate_motor = new control::UnitreeMotor();
  #endif

}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  // 4310 doesn't follow this control pipeline
  control::MotorCANBase* motors[] = {
    #ifdef BASE_TRANSLATE_MOTOR_ENABLE
    motor1,
    #endif
  };
  int motor_num = base_translate_motor_enable;

  // Press Key to start aligning. Else sudden current change when power is switched on might break
  // the board.
  while(!key->Read());

  // wait for release because align_detect also is key press here
  while(key->Read());

  if (base_translate_motor_enable) {
    print("base translate motor enabled\n");
  }

  if (forearm_rotate_motor_enable) {
    print("forearm rotate motor enabled\n");
  }

  if (elbow_rotate_motor_enable) {
    print("elbow rotate motor enabled\n");
  }

  print("total motor num = %d\n", motor_num
    + forearm_rotate_motor_enable
    + elbow_rotate_motor_enable);

  // base translate motor calibrate
  #ifdef BASE_TRANSLATE_MOTOR_ENABLE
  base_translate_motor->SetMaxSpeed(ALIGN_SPEED);

  // Don't put Calibrate() in the while condition
  bool base_translate_alignment_complete = false;
  while (!base_translate_alignment_complete) {
    base_translate_motor->CalcOutput();
    control::MotorCANBase::TransmitOutput(motors, motor_num);
    base_translate_alignment_complete = base_translate_motor->Calibrate();
    osDelay(2);
  }

  base_translate_motor->ReAlign();
  //base_translate_motor->SetMaxSpeed(RUN_SPEED);
  print("base translate motor alignment complete\n");
  #endif

  // forearm rotate motor calibrate
  #ifdef FOREARM_ROTATE_MOTOR_ENABLE
  forearm_rotate_motors[1] = {forearm_rotate_motor};

  // TODO: config zero pos with 4310 config assist
  forearm_rotate_motor->SetZeroPos();
  forearm_rotate_motor->MotorEnable();
  #endif


  #ifdef ELBOW_ROTATE_MOTOR_ENABLE
  elbow_rotate_motor->Stop(ELBOW_MOTOR_ID);
  elbow_rotate_motor_uart->Write((uint8_t*)(&(elbow_rotate_motor->send[0].data)),
                                elbow_rotate_motor->send_length);
  #endif

  if (motor_num >= 1) {
    control::MotorCANBase::TransmitOutput(motors, motor_num);
  }

  // Wait for key to release
  osDelay(500);

  int dir = 1;

  while (true) {
    key_detector.input(key->Read());
    if (key_detector.posEdge()){
      if (dir == 1) {
        #ifdef BASE_TRANSLATE_MOTOR_ENABLE
        base_translate_motor->TurnRelative(PI / 4);
        #endif

        #ifdef FOREARM_ROTATE_MOTOR_ENABLE
        forearm_rotate_motor->SetOutput(PI / 4, M4310_VEL, 30, 0.5, 0);
        #endif


        #ifdef ELBOW_ROTATE_MOTOR_ENABLE
        // motor_id motor ID
        // torque expect torque
        // speed expect speed
        // position expect position
        // Kp Kp parameter for the PD controller
        // Kd Kd parameter for the PD controller
        // elbow_rotate_motor->Control(ELBOW_MOTOR_ID, 0.0, 0.0, PI/8, 0.03, 0.03);

        // elbow_rotate_motor->Control(ELBOW_MOTOR_ID, 0.0, 0.0, PI/4, 0.01, 0.01);
        elbow_rotate_motor->Test(ELBOW_MOTOR_ID);
        elbow_rotate_motor_uart->Write((uint8_t*)(&(elbow_rotate_motor->send[0].data)),
                                      elbow_rotate_motor->send_length);
        #endif

      } else {
        #ifdef BASE_TRANSLATE_MOTOR_ENABLE
        base_translate_motor->ReAlign();
        #endif

        #ifdef FOREARM_ROTATE_MOTOR_ENABLE
        forearm_rotate_motor->SetOutput(0, M4310_VEL, 30, 0.5, 0);
        #endif

        #ifdef ELBOW_ROTATE_MOTOR_ENABLE
        // elbow_rotate_motor->Control(ELBOW_MOTOR_ID, 0.0, 0.0, 0, 0.003, 0.003);
        elbow_rotate_motor->Test(ELBOW_MOTOR_ID);
        elbow_rotate_motor_uart->Write((uint8_t*)(&(elbow_rotate_motor->send[0].data)),
                                      elbow_rotate_motor->send_length);
        #endif
     }
      dir *= -1;
    }

    #ifdef BASE_TRANSLATE_MOTOR_ENABLE
    base_translate_motor->CalcOutput();
    #endif

    #ifdef FOREARM_ROTATE_MOTOR_ENABLE
    forearm_rotate_motor->TransmitOutput(forearm_rotate_motors, 1);
    #endif
    if(motor_num >= 1){
      control::MotorCANBase::TransmitOutput(motors, motor_num);
      osDelay(2);
    }
  }

}

