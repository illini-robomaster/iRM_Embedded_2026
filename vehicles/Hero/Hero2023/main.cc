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

#include "chassis.h"

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "bsp_relay.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "protocol.h"
#include "rgb.h"
#include "oled.h"
#include "bsp_buzzer.h"
#include "bsp_laser.h"

// Global Variables
static remote::DBUS* dbus = nullptr;
static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;
static display::RGB* RGB = nullptr;
static bsp::Relay* relay = nullptr;

// Special Modes
static BoolEdgeDetector FakeDeath(false);
static BoolEdgeDetector LoadDetect(false);
static BoolEdgeDetector ElevationDetect(false);
static BoolEdgeDetector PreloadDetect(false);
static volatile bool Dead = false;
static volatile bool GimbalDead = false;
static volatile bool Elevation = false;
static volatile bool Preload = false;

// Delays
static const int KILLALL_DELAY = 100;
static const int DEFAULT_TASK_DELAY = 100;
static const int CHASSIS_TASK_DELAY = 2;
static const int GIMBAL_TASK_DELAY = 2;
static const int SHOOTER_INNER_TASK_DELAY = 2;
static const int SHOOTER_OUTER_TASK_DELAY = 100;
static const int SHOOTER_TASK_DELAY = 10;
static const int SELFTEST_TASK_DELAY = 100;

// Params used in both chassis and gimbal task
static volatile float yaw_sum = 0;
static control::Motor4310* yaw_motor = nullptr;

static volatile bool safe_mode = true;

//==================================================================================================
// Referee
//==================================================================================================

#define REFEREE_RX_SIGNAL (1 << 1)

const osThreadAttr_t refereeTaskAttribute = {.name = "refereeTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 1024 * 4,
                                             .priority = (osPriority_t)osPriorityAboveNormal,
                                             .tz_module = 0,
                                             .reserved = 0};

osThreadId_t refereeTaskHandle;

// Referee UART Class initialization
class RefereeUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  void RxCompleteCallback() final { osThreadFlagsSet(refereeTaskHandle, REFEREE_RX_SIGNAL); }
};

// Params Initialization
static communication::Referee* referee = nullptr;
static RefereeUART* referee_uart = nullptr;

void refereeTask(void* arg) {
  UNUSED(arg);
  uint32_t length;
  uint8_t* data;

  while (true) {
    uint32_t flags = osThreadFlagsWait(REFEREE_RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & REFEREE_RX_SIGNAL) {
      length = referee_uart->Read(&data);
      referee->Receive(communication::package_t{data, (int)length});
    }
  }
}

//==================================================================================================
// Chassis
//==================================================================================================

const osThreadAttr_t chassisTaskAttribute = {.name = "chassisTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 256 * 4,
                                             .priority = (osPriority_t)osPriorityNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
osThreadId_t chassisTaskHandle;
// TODO:
// 1. test keyboard and mouse data
// 2. test relay

// Params Initialization
static control::MotorCANBase* fl_motor = nullptr;
static control::MotorCANBase* fr_motor = nullptr;
static control::MotorCANBase* bl_motor = nullptr;
static control::MotorCANBase* br_motor = nullptr;
static control::Chassis* chassis = nullptr;

void chassisTask(void* arg) {
  UNUSED(arg);
  // motors initialization
  control::MotorCANBase* motors_can1_chassis[] = {fl_motor, fr_motor, bl_motor, br_motor};
  control::Motor4310* motor_can1_gimbal[] = {yaw_motor};
  float vx_keyboard = 0, vy_keyboard = 0;
  float vx_remote = 0, vy_remote = 0;
  float vx = 0, vy = 0, wz = 0;

  // waiting for the start signal
  while (true) {
    if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN) break;
    osDelay(100);
  }

  while (true) {

    // Elevation(shift)
    ElevationDetect.input(dbus->keyboard.bit.SHIFT);
    if (ElevationDetect.posEdge()) {
      Elevation = !Elevation;
      if (Elevation) {
        relay->On();
      } else {
        relay->Off();
      }
      osDelay(100);
    }

    // read the data from keyboard
    if (dbus->keyboard.bit.A) vx_keyboard -= 61.5;
    if (dbus->keyboard.bit.D) vx_keyboard += 61.5;
    if (dbus->keyboard.bit.W) vy_keyboard += 61.5;
    if (dbus->keyboard.bit.S) vy_keyboard -= 61.5;
    // process the data from keyboard
    if (-35 <= vx_keyboard && vx_keyboard <= 35) vx_keyboard = 0;
    if (-35 <= vy_keyboard && vy_keyboard <= 35) vy_keyboard = 0;
    // soft start and stop in the keyboard
    if (vx_keyboard > 0)
      vx_keyboard -= 60;
    else if (vx_keyboard < 0)
      vx_keyboard += 60;
    if (vy_keyboard > 0)
      vy_keyboard -= 60;
    else if (vy_keyboard < 0)
      vy_keyboard += 60;
    // clip the keyboard data
    vx_keyboard = clip<float>(vx_keyboard, -1200, 1200);
    vy_keyboard = clip<float>(vy_keyboard, -1200, 1200);
    // read the data from the remote controller
    vx_remote = dbus->ch0;
    vy_remote = dbus->ch1;
    // sum whole x and y direction data
    vx = vx_keyboard + vx_remote;
    vy = vy_keyboard + vy_remote;
    // rotation velocity calculation
    wz = yaw_sum * 120000; // 120000 is magic number

    // Update the speed by power limit from referee
    if (!Dead && !Elevation) {
      chassis->SetSpeed(vx, vy, wz);
      yaw_motor->SetOutput(0, 0, 8, 1.5, 0);
      control::Motor4310::TransmitOutput(motor_can1_gimbal, 1);
    } else {
      chassis->SetSpeed(0, 0, 0);
    }

    chassis->Update(true, (float)referee->game_robot_status.chassis_power_limit,
                    referee->power_heat_data.chassis_power,
                    (float)referee->power_heat_data.chassis_power_buffer);

    control::MotorCANBase::TransmitOutput(motors_can1_chassis, 4);
    osDelay(CHASSIS_TASK_DELAY);
  }
}

//==================================================================================================
// Gimbal
//==================================================================================================

const osThreadAttr_t gimbalTaskAttribute = {.name = "gimbal_task",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 512 * 4,
                                            .priority = (osPriority_t)osPriorityHigh,
                                            .tz_module = 0,
                                            .reserved = 0};
osThreadId_t gimbalTaskHandle;
// TODO:
// 1. test keyboard and mouse data

// Params Initialization
static control::Motor4310* pitch_motor = nullptr;
//static bsp::Laser* laser = nullptr;

void gimbal_task(void* arg) {
  UNUSED(arg);
  // motor pointer initialization
  control::Motor4310* motors_can2_gimbal[] = {pitch_motor};
  control::Motor4310* motors_can1_gimbal[] = {yaw_motor};

  print("Wait for beginning signal...\r\n");
  RGB->Display(display::color_red);
//  laser->On(); // has some problem.
  // waiting for start signal
  while (true) {
    if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN) break;
    osDelay(100);
  }

//  pitch_motor->SetZeroPos(); //（for test）
//  yaw_motor->SetZeroPos(); // for test
  osDelay(1000); // wait for 4310s enable

  // the start code for motor 4310s
  pitch_motor->MotorEnable();
  yaw_motor->MotorEnable();
  osDelay(GIMBAL_TASK_DELAY);

  // initialize start position
  RGB->Display(display::color_yellow);
//  laser->Off();
  // need test Pos_Vel mode params
  pitch_motor->SetOutput(0, 10);
  // need test MIT mode params
  yaw_motor->SetOutput(0, 0, 8, 1.5, 0);
  control::Motor4310::TransmitOutput(motors_can1_gimbal, 1);
  control::Motor4310::TransmitOutput(motors_can2_gimbal, 1);
  osDelay(200);

  print("Gimbal Begin!\r\n");
  RGB->Display(display::color_green);
//  laser->On();

  float pitch_keyboard = 0; // E is lifting, Q is lowering
  float pitch_mouse = 0, yaw_mouse = 0;
  float pitch_remote = 0, yaw_remote = 0;
  float pitch_sum = 0;
  float pitch_pos = 0, yaw_pos = 0;
  while (true) {
    // Dead
    while (Dead) osDelay(100);

    // Data Collection
    // pitch data from keyboard
    if (dbus->keyboard.bit.E) pitch_keyboard -= PI; // offset need test
    if (dbus->keyboard.bit.Q) pitch_keyboard += PI; // offset need test
    if (-0.55 * PI <= pitch_keyboard && pitch_keyboard <= 0.55 * PI) pitch_keyboard = 0;
    if (pitch_keyboard > 0)
      pitch_keyboard -= 0.85 * PI;
    if (pitch_keyboard < 0)
      pitch_keyboard += 0.85 * PI;
    pitch_keyboard = clip<float>(pitch_keyboard, -30 * PI, 30 * PI);

    // pitch data from mouse (direction and offset need test)
    pitch_mouse = dbus->mouse.y / 32767.0 * 1000 * PI;
    // pitch data from remote controller (offset and data need test)
    pitch_remote = -dbus->ch3 / 660.0 * 30 * PI;
    // sum whole pitch data
    pitch_sum = pitch_keyboard + pitch_mouse + pitch_remote;
    if (GimbalDead) {
      pitch_sum = 0;
    }

    // yaw data from mouse(direction and offset need test)
    yaw_mouse = dbus->mouse.x / 32767.0 / 20 * PI;
    yaw_remote = dbus->ch2 / 660.0 / 210.0 / 5 * PI;
    // sum whole yaw data
    yaw_sum = yaw_mouse + yaw_remote;

    // Data Processing and Update(yaw clip range need test)
    // pitch processing
    if (dbus->swl == remote::DOWN) {
      pitch_pos = clip<float>(pitch_pos + pitch_sum, -60 * PI, 60 * PI);
    } else {
      pitch_pos = clip<float>(pitch_pos + pitch_sum, -25 * PI, 0);
    }
    // pitch update
    pitch_motor->SetOutput(pitch_pos, pitch_sum);
    control::Motor4310::TransmitOutput(motors_can2_gimbal, 1);
    // if not elevation, update the yaw position in the chassis
    if (Elevation) {
      // yaw processing (gear ratio 1 : 4)
      yaw_pos = clip<float>(yaw_pos + yaw_sum, -PI / 3, PI / 3);
      // yaw update
      yaw_motor->SetOutput(yaw_pos, 8, 20, 1, 0);
      control::Motor4310::TransmitOutput(motors_can1_gimbal, 1);
    }

    osDelay(GIMBAL_TASK_DELAY);
  }
}

//==================================================================================================
// Shooter(TODO)
//==================================================================================================
const osThreadAttr_t shooterTaskAttribute = {.name = "shooter_task",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 256 * 4,
                                             .priority = (osPriority_t)osPriorityNormal,
                                             .tz_module = 0,
                                             .reserved = 0};

osThreadId_t shooterTaskHandle;

// TODO:
// All test params

// Params Initialization
static control::MotorCANBase* force_motor = nullptr;
static control::MotorCANBase* load_motor = nullptr;
static control::MotorCANBase* reload_motor = nullptr;
static control::PDIHV* trigger = nullptr; // CCW for positive angle, CW for negative angle (trigger angle is 100 degree)
static control::ServoMotor* load_servo = nullptr;
static control::ServoMotor* reload_servo = nullptr;
static control::ServoMotor* force_servo = nullptr;

static BoolEdgeDetector ForceWeakDetect(false);
static BoolEdgeDetector ForceStrongDetect(false);

static float force_angle = 65 * PI;

void shooter_task(void* arg) {
  UNUSED(arg);
  // motors initialization
  control::MotorCANBase* can2_reloader[] = { reload_motor};
  control::MotorCANBase* can2_loader[] = { load_motor};
  control::MotorCANBase* can2_force[] = { force_motor};

  // Variable initialization (params need test)
  // reload variables
  bool reload_pull = false;
  bool reload_push = false;
  // the 99.506 is the ratio of the reload servo and devide the 3508 ratio to reload one bullet
  float reload_pos_weak = 3.78 * PI * 99.506 / M3508P19_RATIO;
  float reload_pos_strong = 4.7 * PI * 99.506 / M3508P19_RATIO;

  // load variable
  bool loading = false;
  // 99.506 is the ratio of the load servo and devide the 3508 ratio to load one bullet
  float load_angle = 2 * PI / 6 * 99.506 / M3508P19_RATIO;

  // force variable
  bool force_weak = true; // always start at the weak position
  bool force_strong = false;
  bool force_transforming = false;
  float force_pos = 0;

  int i= 0;

  // waiting for the start signal
  while (true) {
    if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN) break;
    osDelay(SHOOTER_OUTER_TASK_DELAY);
  }

  safe_mode = false;

  while (true) {
    // Dead
    while (Dead || GimbalDead) osDelay(SHOOTER_OUTER_TASK_DELAY);

    i = 0;
    if (Preload && !GimbalDead) {
      while (true) {
        // break condition (loading)
        if (++i > 10 && abs(load_servo->GetOmega()) <= 0.001 && !GimbalDead) break;
        // loading once
        if (i == 1) {
          Preload = false;
          load_servo->SetTarget(load_servo->GetTarget() - load_angle);
        }
        load_servo->CalcOutput();
        control::MotorCANBase::TransmitOutput(can2_loader, 1);
        osDelay(SHOOTER_INNER_TASK_DELAY);
      }
      // after loading
      i = 0;
      load_motor->SetOutput(0);
      control::MotorCANBase::TransmitOutput(can2_loader, 1);
      osDelay(100); // need test the delay time
    }

    if (dbus->swl == remote::DOWN) {
      force_weak = false;
    }

    // whether change the force motor position
    ForceWeakDetect.input(dbus->keyboard.bit.F || dbus->wheel.wheel < 1024);
    ForceStrongDetect.input(dbus->keyboard.bit.C
                          || (dbus->wheel.wheel == remote::WheelDigitalValue && dbus->previous_wheel_value == remote::WheelDigitalValue));
    // just execute force transform once
    if (ForceWeakDetect.posEdge() && !ForceStrongDetect.posEdge() && !force_weak) {
      force_transforming = true;
      force_weak = true;
      force_strong = false;
    } else if (ForceStrongDetect.posEdge() && !ForceWeakDetect.posEdge() && !force_strong) {
      force_transforming = true;
      force_weak = false;
      force_strong = true;
    }
    // force transforming
    if (force_transforming && !GimbalDead) {
      while (true) {
        // break condition (reach the desire position)
        if (++i > 30 && abs(force_motor->GetOmega()) <= 0.001 && !GimbalDead) break;
        // set the speed and acceleration for the reload motor
        // set target pull position once
        if (i == 1) {
          // direction need test
          if (force_weak && !force_strong) {
            force_pos += force_angle;
          } else if (force_strong && !force_weak) {
            force_pos -= force_angle;
          }
          force_servo->SetTarget(force_pos);
        }
        force_servo->CalcOutput();
        control::MotorCANBase::TransmitOutput(can2_force, 1);
        osDelay(SHOOTER_INNER_TASK_DELAY);
      }
      // after changing the force motor position
      i = 0;
      force_transforming = false;
      force_motor->SetOutput(0);
      control::MotorCANBase::TransmitOutput(can2_force, 1);
      osDelay(SHOOTER_OUTER_TASK_DELAY); // need test the delay time(wait for the)
    }

    // the shoot and load process is automatic
    // 1. using servo pull the trigger (shooting process)
    // 2. reload motor pull the bullet board to the desire position (load process below)
    // 3. using servo push the trigger the hold the bullet board
    // 4. load motor load one bullet on the board
    // 5. reload motor release to the original position to prepare for the next load

    // Load Detector
    LoadDetect.input(dbus->swr == remote::UP || dbus->mouse.l);
    if (LoadDetect.posEdge() && !GimbalDead) {
      osDelay(100);
      // step 1
      trigger->SetOutPutAngle(0);
      osDelay(SHOOTER_OUTER_TASK_DELAY); // wait for the bullet out
      // step 2
      while (true) {
        // break condition (reach the desire position)
        if (++i > 30 && abs(reload_servo->GetOmega()) <= 0.00005 && !GimbalDead) break;
        // set the speed and acceleration for the reload motor
        // set target pull position once
        if (!reload_pull) {
          reload_pull = true;
          reload_servo->SetMaxSpeed(10 * PI);
          reload_servo->SetMaxAcceleration(25 * PI);
          if (force_weak) {
            reload_servo->SetTarget(reload_pos_weak);
          } else if (force_strong) {
            reload_servo->SetTarget(reload_pos_strong);
          }
        }
        reload_servo->CalcOutput();
        control::MotorCANBase::TransmitOutput(can2_reloader, 1);
        osDelay(SHOOTER_INNER_TASK_DELAY);
      }
      // step 3 (before release the reload board)
      trigger->SetOutPutAngle(-80); // need test the trigger angle
      osDelay(1500); // need test the delay time(wait for the)
      // after reload pulling
      i = 0;
      reload_motor->SetOutput(0);
      control::MotorCANBase::TransmitOutput(can2_reloader, 1);
      reload_pull = false;
      osDelay(100); // need test the delay time(wait for the)

      // step 4
      while (true) {
        // break condition (loading)
        if (++i > 30 && abs(load_servo->GetOmega()) <= 0.0001 && !GimbalDead) break;
        // loading once
        if (!loading) {
          loading = true;
          load_servo->SetTarget(load_servo->GetTarget() - load_angle);
        }
        load_servo->CalcOutput();
        control::MotorCANBase::TransmitOutput(can2_loader, 1);
        osDelay(SHOOTER_INNER_TASK_DELAY);
      }
      // after loading
      i = 0;
      load_motor->SetOutput(0);
      control::MotorCANBase::TransmitOutput(can2_loader, 1);
      loading = false;
      osDelay(SHOOTER_OUTER_TASK_DELAY); // need test the delay time
      // step 5
      while (true) {
        // break condition (reach the desire position)
        if (++i > 20 && abs(reload_servo->GetOmega()) <= 0.0001 && !GimbalDead) break;
        // set target push position once
        if (!reload_push) {
          reload_push = true;
          reload_servo->SetTarget(0, true);
          reload_servo->SetMaxSpeed(50 * PI);
          reload_servo->SetMaxAcceleration(100 * PI);
        }
        reload_servo->CalcOutput();
        control::MotorCANBase::TransmitOutput(can2_reloader, 1);
        osDelay(SHOOTER_INNER_TASK_DELAY);
      }
      // after reload pushing
      i = 0;
      reload_motor->SetOutput(0);
      control::MotorCANBase::TransmitOutput(can2_reloader, 1);
      reload_push = false;
      osDelay(SHOOTER_OUTER_TASK_DELAY);
    }

    dbus->previous_wheel_value = dbus->wheel.wheel;
    osDelay(SHOOTER_TASK_DELAY);
  }
}

//==================================================================================================
// SelfTest(TODO: need to modify the position)
//==================================================================================================

const osThreadAttr_t selfTestingTask = {.name = "selfTestTask",
                                        .attr_bits = osThreadDetached,
                                        .cb_mem = nullptr,
                                        .cb_size = 0,
                                        .stack_mem = nullptr,
                                        .stack_size = 256 * 4,
                                        .priority = (osPriority_t)osPriorityBelowNormal,
                                        .tz_module = 0,
                                        .reserved = 0};
osThreadId_t selfTestTaskHandle;

// Params Initialization
static bsp::Buzzer* buzzer = nullptr;
static display::OLED* OLED = nullptr;
using Note = bsp::BuzzerNote;
static bsp::BuzzerNoteDelayed Mario[] = {
    {Note::Mi3M, 80}, {Note::Silent, 80}, {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::Do1M, 80}, {Note::Silent, 80}, {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::So5M, 80}, {Note::Silent, 560}, {Note::So5L, 80}, {Note::Silent, 0}, {Note::Finish, 0}};

// Chassis motor flags
static bool fl_motor_flag = false;
static bool fr_motor_flag = false;
static bool bl_motor_flag = false;
static bool br_motor_flag = false;
// Gimbal motor flags
static bool pitch_motor_flag = false;
static bool yaw_motor_flag = false;
// Dbus
static bool dbus_flag = false;

void self_Check_Task(void* arg) {
  UNUSED(arg);
  osDelay(SELFTEST_TASK_DELAY);
  // Enter screen
  OLED->ShowIlliniRMLOGO();
  buzzer->SingSong(Mario, [](uint32_t milli) { osDelay(milli);});
  OLED->OperateGram(display::PEN_CLEAR);

  // Showing Name for self-check
  // Gimbal motor
  OLED->ShowString(0, 0, (uint8_t*)"GP");
  OLED->ShowString(0, 5, (uint8_t*)"GY");
  // Lidar
  OLED->ShowString(2, 5, (uint8_t*)"Ldr");
  // Dbus
  OLED->ShowString(3, 6, (uint8_t*)"Dbs");
  // Chassis motor
  OLED->ShowString(1, 12, (uint8_t*)"FL");
  OLED->ShowString(2, 12, (uint8_t*)"FR");
  OLED->ShowString(3, 12, (uint8_t*)"BL");
  OLED->ShowString(4, 12, (uint8_t*)"BR");

  while (true) {
    // chassis motors
    fl_motor->connection_flag_ = false;
    fr_motor->connection_flag_ = false;
    bl_motor->connection_flag_ = false;
    br_motor->connection_flag_ = false;
    // gimbal motors
    pitch_motor->connection_flag_ = false;
    yaw_motor->connection_flag_ = false;
    // Dbus
    dbus->connection_flag_ = false;
    osDelay(SELFTEST_TASK_DELAY);
    // chassis motors
    fl_motor_flag = fl_motor->connection_flag_;
    fr_motor_flag = fr_motor->connection_flag_;
    bl_motor_flag = bl_motor->connection_flag_;
    br_motor_flag = br_motor->connection_flag_;
    // gimbal motors
    pitch_motor_flag = pitch_motor->connection_flag_;
    yaw_motor_flag = yaw_motor->connection_flag_;
    // Dbus
    dbus_flag = dbus->connection_flag_;

    //Showing the result for self check
    // Chassis motors
    OLED->ShowBlock(1, 15, fl_motor_flag);
    OLED->ShowBlock(2, 15, fr_motor_flag);
    OLED->ShowBlock(3, 15, bl_motor_flag);
    OLED->ShowBlock(4, 15, br_motor_flag);
    // Gimbal motors
    OLED->ShowBlock(0, 2, pitch_motor_flag);
    OLED->ShowBlock(0, 7, yaw_motor_flag);
    // Dbus
    OLED->ShowBlock(3, 9, dbus_flag);

    osDelay(100);
  }
}

//==================================================================================================
// RM Init
//==================================================================================================

void RM_RTOS_Init() {
  // peripherals initialization
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);
  // Dbus
  dbus = new remote::DBUS(&huart3);
  // Can
  can1 = new bsp::CAN(&hcan1, true);
  can2 = new bsp::CAN(&hcan2, false);
  // RGB
  RGB = new display::RGB(&htim5, 3, 2, 1, 1000000);

  // Chassis class initialization
  // motor initialization
  fl_motor = new control::Motor3508(can1, 0x202);
  fr_motor = new control::Motor3508(can1, 0x201);
  bl_motor = new control::Motor3508(can1, 0x203);
  br_motor = new control::Motor3508(can1, 0x204);
  // motor distribution
  control::MotorCANBase* motors[control::FourWheel::motor_num];
  motors[control::FourWheel::front_left] = fl_motor;
  motors[control::FourWheel::front_right] = fr_motor;
  motors[control::FourWheel::back_left] = bl_motor;
  motors[control::FourWheel::back_right] = br_motor;
  // chassis struct initialization
  control::chassis_t chassis_data;
  chassis_data.motors = motors;
  chassis_data.model = control::CHASSIS_MECANUM_WHEEL;
  chassis = new control::Chassis(chassis_data);

  // Gimbal initialization
  pitch_motor = new control::Motor4310(can2, 0x02, 0x01, control::POS_VEL);
  yaw_motor = new control::Motor4310(can1, 0x04, 0x03, control::MIT);

  //Shooter initialization
  load_motor = new control::Motor3508(can2, 0x201);
  reload_motor = new control::Motor3508(can2, 0x202);
  force_motor = new control::Motor3508(can2, 0x203);
  // magic number from the data test for this servo
  trigger = new control::PDIHV(&htim1, 1, 1980000, 500, 1500);

  // Servo control for each shooter motor
  control::servo_t servo_data;
  servo_data.motor = load_motor;
  servo_data.max_speed = 3 * PI; // params need test
  servo_data.max_acceleration = 8 * PI;
  servo_data.transmission_ratio = M3508P19_RATIO;
  servo_data.omega_pid_param = new float [3] {150, 1.2, 5};
  servo_data.max_iout = 1000;
  servo_data.max_out = 13000;
  load_servo = new control::ServoMotor(servo_data);

  servo_data.motor = reload_motor;
  servo_data.max_speed = 15 * PI; // params need test
  servo_data.max_acceleration = 25 * PI;
  servo_data.omega_pid_param = new float [3] {130, 15, 5};
  reload_servo = new control::ServoMotor(servo_data);

  servo_data.motor = force_motor;
  servo_data.max_speed = 100 * PI;
  servo_data.max_acceleration = 100 * PI;
  servo_data.omega_pid_param = new float [3] {150, 1.2, 5};
  force_servo = new control::ServoMotor(servo_data);

  // Referee initialization
  referee_uart = new RefereeUART(&huart6);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);
  referee = new communication::Referee;

  // Relay initialization
  relay = new bsp::Relay(RELAY_1_GPIO_Port, RELAY_1_Pin); /* USE GPIO_1 (PB14)->PIN 7 in the board */
}

//==================================================================================================
// RTOS Threads Init
//==================================================================================================

void RM_RTOS_Threads_Init(void) {
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
  chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
  gimbalTaskHandle = osThreadNew(gimbal_task, nullptr, &gimbalTaskAttribute);
  shooterTaskHandle = osThreadNew(shooter_task, nullptr, &shooterTaskAttribute);
//  selfTestTaskHandle = osThreadNew(self_Check_Task, nullptr, &selfTestingTask);
}

//==================================================================================================
// Kill All
//==================================================================================================

void KillAll() {
  RM_EXPECT_TRUE(false, "Operation Killed!\r\n");

  control::MotorCANBase* motors_can1_chassis[] = {fl_motor, fr_motor, bl_motor, br_motor};
  control::MotorCANBase* motors_can2_shooter[] = {load_motor, reload_motor, force_motor};
  RGB->Display(display::color_blue);

  while (true) {
    // Restart after emergency stop
    FakeDeath.input(dbus->swr == remote::DOWN && safe_mode == false);
    if (FakeDeath.posEdge()) {
      Dead = false;
      RGB->Display(display::color_green);
//      laser->On();
      pitch_motor->MotorEnable();
      yaw_motor->MotorEnable();
      break;
    }
    // Dead, wait for restart signal from User
    // Gimbal motors
    pitch_motor->MotorDisable();
    yaw_motor->MotorDisable();
    // Chassis motors
    fl_motor->SetOutput(0);
    bl_motor->SetOutput(0);
    fr_motor->SetOutput(0);
    br_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(motors_can1_chassis, 4);
    // shooter motors
    load_motor->SetOutput(0);
    reload_motor->SetOutput(0);
    force_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(motors_can2_shooter, 3);

    osDelay(KILLALL_DELAY);
  }
}

//==================================================================================================
// RTOS Default Task
//==================================================================================================

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* motors_can2_shooter_part[] = {reload_motor, force_motor, load_motor};
  safe_mode = true;

  while (true) {
    // Emergency stop
    FakeDeath.input(dbus->swr == remote::DOWN && safe_mode == false);
    if (FakeDeath.posEdge()) {
      Dead = true;
      KillAll();
    }
    // preloading
    PreloadDetect.input(dbus->swl == remote::UP);
    if (PreloadDetect.posEdge() && !Preload) {
      Preload = true;
    }

    if (referee->game_robot_status.mains_power_shooter_output == 0) {
      GimbalDead = true;
      pitch_motor->MotorDisable();
      reload_motor->SetOutput(0);
      force_motor->SetOutput(0);
      load_motor->SetOutput(0);
      control::MotorCANBase::TransmitOutput(motors_can2_shooter_part, 3);
    } else {
      GimbalDead = false;
      pitch_motor->MotorEnable();
    }

    osDelay(DEFAULT_TASK_DELAY);
  }
}
