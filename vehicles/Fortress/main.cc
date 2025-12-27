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

#include <cstdio>

#include "bsp_buzzer.h"
#include "bsp_gpio.h"
#include "bsp_imu.h"
#include "bsp_laser.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "fortress.h"
#include "gimbal.h"
#include "i2c.h"
#include "lidar07.h"
#include "oled.h"
#include "protocol.h"
#include "rgb.h"
#include "shooter.h"
#include "supercap.h"
#include "user_interface.h"
#include "utils.h"

static const int GIMBAL_TASK_DELAY = 1;
static const int CHASSIS_TASK_DELAY = 2;
static const int SHOOTER_TASK_DELAY = 10;
static const int SELFTEST_TASK_DELAY = 100;
static const int UI_TASK_DELAY = 20;
static const int FORTRESS_TASK_DELAY = 2;
static const int KILLALL_DELAY = 100;
static const int DEFAULT_TASK_DELAY = 100;

static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;
static remote::DBUS* dbus = nullptr;
static display::RGB* RGB = nullptr;
// static control::SuperCap* supercap = nullptr;

static BoolEdgeDetector FakeDeath(false);
static volatile bool Dead = false;
static BoolEdgeDetector ChangeSpinMode(false);
static volatile bool SpinMode = false;
static BoolEdgeDetector PeekModeLeft(false);
static BoolEdgeDetector PeekModeRight(false);
static volatile bool PeekMode = false;
static BoolEdgeDetector ChangeFortressMode(false);
static volatile bool FortressMode = false;

static volatile float relative_angle = 0;

static bool volatile pitch_motor_flag = false;
static bool volatile yaw_motor_flag = false;
static bool volatile sl_motor_flag = false;
static bool volatile sr_motor_flag = false;
static bool volatile ld_motor_flag = false;
static bool volatile fl_motor_flag = false;
static bool volatile fr_motor_flag = false;
static bool volatile bl_motor_flag = false;
static bool volatile br_motor_flag = false;
static bool volatile calibration_flag = false;
static bool volatile referee_flag = false;
static bool volatile dbus_flag = false;
static bool volatile lidar_flag = false;

static volatile bool selftestStart = false;

static BoolEdgeDetector SuperCapKey(false);
static bool volatile UseSuperCap = false;

//==================================================================================================
// IMU
//==================================================================================================

#define IMU_RX_SIGNAL (1 << 0)

const osThreadAttr_t imuTaskAttribute = {.name = "imuTask",
                                         .attr_bits = osThreadDetached,
                                         .cb_mem = nullptr,
                                         .cb_size = 0,
                                         .stack_mem = nullptr,
                                         .stack_size = 512 * 4,
                                         .priority = (osPriority_t)osPriorityRealtime,
                                         .tz_module = 0,
                                         .reserved = 0};
osThreadId_t imuTaskHandle;

class IMU : public bsp::IMU_typeC {
 public:
  using bsp::IMU_typeC::IMU_typeC;

 protected:
  void RxCompleteCallback() final { osThreadFlagsSet(imuTaskHandle, IMU_RX_SIGNAL); }
};

static IMU* imu = nullptr;

void imuTask(void* arg) {
  UNUSED(arg);

  while (true) {
    uint32_t flags = osThreadFlagsWait(IMU_RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & IMU_RX_SIGNAL) imu->Update();
  }
}

//==================================================================================================
// Gimbal
//==================================================================================================

const osThreadAttr_t gimbalTaskAttribute = {.name = "gimbalTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 512 * 4,
                                            .priority = (osPriority_t)osPriorityHigh,
                                            .tz_module = 0,
                                            .reserved = 0};
osThreadId_t gimbalTaskHandle;

static control::MotorCANBase* pitch_motor = nullptr;
static control::MotorCANBase* yaw_motor = nullptr;
static control::Gimbal* gimbal = nullptr;
static control::gimbal_data_t* gimbal_param = nullptr;
static bsp::Laser* laser = nullptr;

void gimbalTask(void* arg) {
  UNUSED(arg);

  control::MotorCANBase* motors_can1_gimbal[] = {pitch_motor, yaw_motor};

  print("Wait for beginning signal...\r\n");
  RGB->Display(display::color_red);
  laser->On();

  while (true) {
    if (dbus->keyboard.bit.V || dbus->swr == remote::DOWN) break;
    osDelay(100);
  }

  int i = 0;
  while (i < 1000 || !imu->DataReady()) {
    gimbal->TargetAbsWOffset(0, 0);
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_gimbal, 2);
    osDelay(GIMBAL_TASK_DELAY);
    ++i;
  }

  print("Start Calibration.\r\n");
  RGB->Display(display::color_yellow);
  laser->Off();
  imu->Calibrate();

  while (!imu->DataReady() || !imu->CaliDone()) {
    gimbal->TargetAbsWOffset(0, 0);
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_gimbal, 2);
    osDelay(GIMBAL_TASK_DELAY);
  }

  print("Gimbal Begin!\r\n");
  RGB->Display(display::color_green);
  laser->On();

  float pitch_ratio, yaw_ratio;
  float pitch_curr, yaw_curr;
  float pitch_target = 0, yaw_target = 0;
  float pitch_diff, yaw_diff;

  while (true) {
    while (Dead) osDelay(100);

    pitch_ratio = dbus->mouse.y / 32767.0;
    yaw_ratio = -dbus->mouse.x / 32767.0;

    pitch_ratio += -dbus->ch3 / 660.0 / 210.0;
    yaw_ratio += -dbus->ch2 / 660.0 / 210.0;

    pitch_target = clip<float>(pitch_target + pitch_ratio, -gimbal_param->pitch_max_,
                               gimbal_param->pitch_max_);
    yaw_target = wrap<float>(yaw_target + yaw_ratio, -PI, PI);

    pitch_curr = imu->INS_angle[1];
    yaw_curr = imu->INS_angle[0];

    pitch_diff = clip<float>(pitch_target - pitch_curr, -PI, PI);
    yaw_diff = wrap<float>(yaw_target - yaw_curr, -PI, PI);

    gimbal->TargetRel(-pitch_diff, yaw_diff);

    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_gimbal, 2);
    osDelay(GIMBAL_TASK_DELAY);
  }
}

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

static control::MotorCANBase* fl_motor = nullptr;
static control::MotorCANBase* fr_motor = nullptr;
static control::MotorCANBase* bl_motor = nullptr;
static control::MotorCANBase* br_motor = nullptr;
static control::Chassis* chassis = nullptr;

static const float CHASSIS_DEADZONE = 0.04;

// static bool PeekDirection = false;

void chassisTask(void* arg) {
  UNUSED(arg);

  control::MotorCANBase* motors[] = {fl_motor, fr_motor, bl_motor, br_motor};

  float sin_yaw, cos_yaw;
  float vx_keyboard = 0, vy_keyboard = 0;
  float vx_remote, vy_remote;
  float vx_set, vy_set, wz_set;

  float spin_speed = 600;
  float follow_speed = 400;

  while (true) {
    if (dbus->keyboard.bit.V || dbus->swr == remote::DOWN) break;
    osDelay(100);
  }

  while (!imu->CaliDone()) osDelay(100);

  while (true) {
    while (Dead) osDelay(100);

    //    PeekModeLeft.input(dbus->keyboard.bit.Q);
    //    if (PeekModeLeft.posEdge()) {
    //      PeekMode = true;
    //      PeekDirection = false;
    //    }
    //
    //    PeekModeRight.input(dbus->keyboard.bit.E);
    //    if (PeekModeLeft.posEdge()) {
    //      PeekMode = true;
    //      PeekDirection = true;
    //    }

    ChangeSpinMode.input(dbus->keyboard.bit.SHIFT || dbus->swl == remote::UP);
    if (ChangeSpinMode.posEdge()) SpinMode = !SpinMode;

    if (dbus->keyboard.bit.A) vx_keyboard -= 61.5;
    if (dbus->keyboard.bit.D) vx_keyboard += 61.5;
    if (dbus->keyboard.bit.W) vy_keyboard += 61.5;
    if (dbus->keyboard.bit.S) vy_keyboard -= 61.5;

    if (-35 <= vx_keyboard && vx_keyboard <= 35) vx_keyboard = 0;
    if (-35 <= vy_keyboard && vy_keyboard <= 35) vy_keyboard = 0;

    if (vx_keyboard > 0)
      vx_keyboard -= 60;
    else if (vx_keyboard < 0)
      vx_keyboard += 60;

    if (vy_keyboard > 0)
      vy_keyboard -= 60;
    else if (vy_keyboard < 0)
      vy_keyboard += 60;

    vx_keyboard = clip<float>(vx_keyboard, -1200, 1200);
    vy_keyboard = clip<float>(vy_keyboard, -1200, 1200);

    vx_remote = dbus->ch0;
    vy_remote = dbus->ch1;

    vx_set = vx_keyboard + vx_remote;
    vy_set = vy_keyboard + vy_remote;

    if (yaw_motor_flag)
      relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);
    else
      relative_angle = 0;

    if (SpinMode) {
      sin_yaw = arm_sin_f32(relative_angle);
      cos_yaw = arm_cos_f32(relative_angle);
      vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
      vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
      wz_set = spin_speed;
      //    } else if (PeekMode) {
      //      sin_yaw = arm_sin_f32(relative_angle);
      //      cos_yaw = arm_cos_f32(relative_angle);
      //      vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
      //      vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
      //
      //      float peek_angle = 30.0 / 180 * PI;
      //      peek_angle = PeekDirection ? peek_angle : -peek_angle;
      //
      //      wz_set = std::min(follow_speed, follow_speed * (relative_angle - peek_angle));
      //      if (-CHASSIS_DEADZONE < (relative_angle - peek_angle) &&
      //          (relative_angle - peek_angle) < CHASSIS_DEADZONE)
      //        wz_set = 0;
    } else {
      sin_yaw = arm_sin_f32(relative_angle);
      cos_yaw = arm_cos_f32(relative_angle);
      vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
      vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
      wz_set = std::min(follow_speed, follow_speed * relative_angle);
      if (-CHASSIS_DEADZONE < relative_angle && relative_angle < CHASSIS_DEADZONE) wz_set = 0;
    }

    chassis->SetSpeed(vx_set, vy_set, wz_set);

    chassis->Update(true, (float)referee->game_robot_status.chassis_power_limit,
                    referee->power_heat_data.chassis_power,
                    (float)referee->power_heat_data.chassis_power_buffer);

    if (FortressMode) {
      fl_motor->SetOutput(0);
      fr_motor->SetOutput(0);
      bl_motor->SetOutput(0);
      br_motor->SetOutput(0);
    }

    control::MotorCANBase::TransmitOutput(motors, 4);
    osDelay(CHASSIS_TASK_DELAY);
  }
}

//==================================================================================================
// Shooter
//==================================================================================================

const osThreadAttr_t shooterTaskAttribute = {.name = "shooterTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 256 * 4,
                                             .priority = (osPriority_t)osPriorityNormal,
                                             .tz_module = 0,
                                             .reserved = 0};

osThreadId_t shooterTaskHandle;

static control::MotorCANBase* sl_motor = nullptr;
static control::MotorCANBase* sr_motor = nullptr;
static control::MotorCANBase* ld_motor = nullptr;
static control::Shooter* shooter = nullptr;

static volatile bool flywheelFlag = false;

void shooterTask(void* arg) {
  UNUSED(arg);

  control::MotorCANBase* motors_can1_shooter[] = {sl_motor, sr_motor, ld_motor};

  while (true) {
    if (dbus->keyboard.bit.V || dbus->swr == remote::DOWN) break;
    osDelay(100);
  }

  while (!imu->CaliDone()) osDelay(100);

  while (true) {
    while (Dead) osDelay(100);

    if (referee->game_robot_status.mains_power_shooter_output &&
        referee->power_heat_data.shooter_id1_17mm_cooling_heat <
            referee->game_robot_status.shooter_barrel_cooling_value - 20 &&
        (dbus->mouse.l || dbus->swr == remote::UP))
      shooter->LoadNext();
    if (!referee->game_robot_status.mains_power_shooter_output || dbus->keyboard.bit.Q ||
        dbus->swr == remote::DOWN) {
      flywheelFlag = false;
      shooter->SetFlywheelSpeed(0);
    } else if (referee->game_robot_status.shooter_barrel_heat_limit == 15) {
      flywheelFlag = true;
      shooter->SetFlywheelSpeed(440);  // 445 MAX
    } else if (referee->game_robot_status.shooter_barrel_heat_limit >= 18) {
      flywheelFlag = true;
      shooter->SetFlywheelSpeed(485);  // 490 MAX
    } else {
      flywheelFlag = false;
      shooter->SetFlywheelSpeed(0);
    }

    shooter->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_shooter, 3);
    osDelay(SHOOTER_TASK_DELAY);
  }
}

//==================================================================================================
// Fortress
//==================================================================================================

const osThreadAttr_t fortressTaskAttribute = {.name = "fortressTask",
                                              .attr_bits = osThreadDetached,
                                              .cb_mem = nullptr,
                                              .cb_size = 0,
                                              .stack_mem = nullptr,
                                              .stack_size = 256 * 4,
                                              .priority = (osPriority_t)osPriorityNormal,
                                              .tz_module = 0,
                                              .reserved = 0};

osThreadId_t fortressTaskHandle;

static bsp::GPIO* left = nullptr;
static bsp::GPIO* right = nullptr;

static control::MotorCANBase* elevator_left_motor = nullptr;
static control::MotorCANBase* elevator_right_motor = nullptr;
static control::MotorCANBase* fortress_motor = nullptr;
static control::Fortress* fortress = nullptr;

void fortressTask(void* arg) {
  UNUSED(arg);

  control::MotorCANBase* motors_can2_fortress[] = {elevator_left_motor, elevator_right_motor,
                                                   fortress_motor};

  while (true) {
    if (dbus->keyboard.bit.V || dbus->swr == remote::DOWN) break;
    osDelay(100);
  }

  while (!imu->CaliDone()) osDelay(100);

  while (!fortress->Calibrate()) {
    while (Dead) osDelay(100);
    if (fortress->Error()) {
      while (true) {
        fortress->Stop(control::ELEVATOR);
        fortress->Stop(control::SPINNER);
        control::MotorCANBase::TransmitOutput(motors_can2_fortress, 3);
        osDelay(100);
      }
    }
    fortress->Stop(control::SPINNER);
    control::MotorCANBase::TransmitOutput(motors_can2_fortress, 3);
    osDelay(FORTRESS_TASK_DELAY);
  }

  while (true) {
    ChangeFortressMode.input(dbus->keyboard.bit.X);
    if (ChangeFortressMode.posEdge()) FortressMode = !FortressMode;

    if (FortressMode) {
      int i = 0;
      while (true) {
        if (++i > 100 / 2 && fortress->Finished()) break;
        if (fortress->Error()) {
          while (true) {
            fortress->Stop(control::ELEVATOR);
            fortress->Stop(control::SPINNER);
            control::MotorCANBase::TransmitOutput(motors_can2_fortress, 3);
            osDelay(100);
          }
        }
        fortress->Transform(true);
        fortress->Spin(true, (float)referee->game_robot_status.chassis_power_limit,
                       referee->power_heat_data.chassis_power,
                       (float)referee->power_heat_data.chassis_power_buffer);
        control::MotorCANBase::TransmitOutput(motors_can2_fortress, 3);
        osDelay(FORTRESS_TASK_DELAY);
      }
    } else {
      int i = 0;
      while (true) {
        if (++i > 100 / 2 && fortress->Finished()) break;
        if (fortress->Error()) {
          while (true) {
            fortress->Stop(control::ELEVATOR);
            fortress->Stop(control::SPINNER);
            control::MotorCANBase::TransmitOutput(motors_can2_fortress, 3);
            osDelay(100);
          }
        }
        fortress->Transform(false);
        fortress->Stop(control::SPINNER);
        control::MotorCANBase::TransmitOutput(motors_can2_fortress, 3);
        osDelay(FORTRESS_TASK_DELAY);
      }
    }
  }
}

//==================================================================================================
// SelfTest
//==================================================================================================

const osThreadAttr_t selfTestTaskAttribute = {.name = "selfTestTask",
                                              .attr_bits = osThreadDetached,
                                              .cb_mem = nullptr,
                                              .cb_size = 0,
                                              .stack_mem = nullptr,
                                              .stack_size = 256 * 4,
                                              .priority = (osPriority_t)osPriorityBelowNormal,
                                              .tz_module = 0,
                                              .reserved = 0};

osThreadId_t selfTestTaskHandle;

using Note = bsp::BuzzerNote;

static bsp::BuzzerNoteDelayed Mario[] = {
    {Note::Mi3M, 80}, {Note::Silent, 80},  {Note::Mi3M, 80}, {Note::Silent, 240},
    {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::Do1M, 80}, {Note::Silent, 80},
    {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::So5M, 80}, {Note::Silent, 560},
    {Note::So5L, 80}, {Note::Silent, 0},   {Note::Finish, 0}};

static bsp::Buzzer* buzzer = nullptr;
static display::OLED* OLED = nullptr;

void selfTestTask(void* arg) {
  UNUSED(arg);

  OLED->ShowIlliniRMLOGO();
  buzzer->SingSong(Mario, [](uint32_t milli) { osDelay(milli); });
  OLED->OperateGram(display::PEN_CLEAR);

  OLED->ShowString(0, 0, (uint8_t*)"GP");
  OLED->ShowString(0, 5, (uint8_t*)"GY");
  OLED->ShowString(1, 0, (uint8_t*)"SL");
  OLED->ShowString(1, 5, (uint8_t*)"SR");
  OLED->ShowString(1, 10, (uint8_t*)"LD");
  OLED->ShowString(2, 0, (uint8_t*)"FL");
  OLED->ShowString(2, 5, (uint8_t*)"FR");
  OLED->ShowString(2, 10, (uint8_t*)"BL");
  OLED->ShowString(2, 15, (uint8_t*)"BR");
  OLED->ShowString(3, 0, (uint8_t*)"Cali");
  OLED->ShowString(3, 7, (uint8_t*)"Temp:");
  OLED->ShowString(4, 0, (uint8_t*)"Ref");
  OLED->ShowString(4, 6, (uint8_t*)"Dbus");
  OLED->ShowString(4, 13, (uint8_t*)"Lidar");

  char temp[6] = "";
  while (true) {
    pitch_motor->connection_flag_ = false;
    yaw_motor->connection_flag_ = false;
    sl_motor->connection_flag_ = false;
    sr_motor->connection_flag_ = false;
    ld_motor->connection_flag_ = false;
    fl_motor->connection_flag_ = false;
    fr_motor->connection_flag_ = false;
    bl_motor->connection_flag_ = false;
    br_motor->connection_flag_ = false;
    referee->connection_flag_ = false;
    dbus->connection_flag_ = false;
    osDelay(SELFTEST_TASK_DELAY);
    pitch_motor_flag = pitch_motor->connection_flag_;
    yaw_motor_flag = yaw_motor->connection_flag_;
    sl_motor_flag = sl_motor->connection_flag_;
    sr_motor_flag = sr_motor->connection_flag_;
    ld_motor_flag = ld_motor->connection_flag_;
    fl_motor_flag = fl_motor->connection_flag_;
    fr_motor_flag = fr_motor->connection_flag_;
    bl_motor_flag = bl_motor->connection_flag_;
    br_motor_flag = br_motor->connection_flag_;
    calibration_flag = imu->CaliDone();
    referee_flag = referee->connection_flag_;
    dbus_flag = dbus->connection_flag_;

    OLED->ShowBlock(0, 2, pitch_motor_flag);
    OLED->ShowBlock(0, 7, yaw_motor_flag);
    OLED->ShowBlock(1, 2, sl_motor_flag);
    OLED->ShowBlock(1, 7, sr_motor_flag);
    OLED->ShowBlock(1, 12, ld_motor_flag);
    OLED->ShowBlock(2, 2, fl_motor_flag);
    OLED->ShowBlock(2, 7, fr_motor_flag);
    OLED->ShowBlock(2, 12, bl_motor_flag);
    OLED->ShowBlock(2, 17, br_motor_flag);
    OLED->ShowBlock(3, 4, imu->CaliDone());
    snprintf(temp, 6, "%.2f", imu->Temp);
    OLED->ShowString(3, 12, (uint8_t*)temp);
    OLED->ShowBlock(4, 3, referee_flag);
    OLED->ShowBlock(4, 10, dbus_flag);
    OLED->ShowBlock(4, 18, lidar_flag);

    OLED->RefreshGram();

    selftestStart = true;
  }
}

//==================================================================================================
// UI
//==================================================================================================

const osThreadAttr_t UITaskAttribute = {.name = "UITask",
                                        .attr_bits = osThreadDetached,
                                        .cb_mem = nullptr,
                                        .cb_size = 0,
                                        .stack_mem = nullptr,
                                        .stack_size = 1024 * 4,
                                        .priority = (osPriority_t)osPriorityBelowNormal,
                                        .tz_module = 0,
                                        .reserved = 0};

osThreadId_t UITaskHandle;

// static distance::LIDAR07_UART* LIDAR = nullptr;
static communication::UserInterface* UI = nullptr;

void UITask(void* arg) {
  UNUSED(arg);

  while (!selftestStart) osDelay(100);

  //   int tryLIDAR = 0;
  //   while (!LIDAR->begin()) {
  //     if (++tryLIDAR >= 5) break;
  //     osDelay(10);
  //   }
  //   tryLIDAR = 0;
  //   while (!LIDAR->startFilter()) {
  //     if (++tryLIDAR >= 5) break;
  //     osDelay(10);
  //   }

  UI->SetID(referee->game_robot_status.robot_id);

  communication::package_t frame;
  communication::graphic_data_t graphGimbal;
  communication::graphic_data_t graphChassis;
  communication::graphic_data_t graphArrow;
  communication::graphic_data_t graphCali;
  communication::graphic_data_t graphEmpty2;
  communication::graphic_data_t graphCrosshair1;
  communication::graphic_data_t graphCrosshair2;
  communication::graphic_data_t graphCrosshair3;
  communication::graphic_data_t graphCrosshair4;
  communication::graphic_data_t graphCrosshair5;
  communication::graphic_data_t graphCrosshair6;
  communication::graphic_data_t graphCrosshair7;
  communication::graphic_data_t graphBarFrame;
  communication::graphic_data_t graphBar;
  communication::graphic_data_t graphPercent;
  communication::graphic_data_t graphDiag;
  communication::graphic_data_t graphMode;
  communication::graphic_data_t graphDist;
  //  communication::graphic_data_t graphLid;
  communication::graphic_data_t graphWheel;

  char msgBuffer1[30] = "PITCH MOTOR UNCONNECTED";
  char msgBuffer2[30] = "YAW MOTOR UNCONNECTED";
  char msgBuffer3[30] = "L SHOOTER MOTOR UNCONNECTED";
  char msgBuffer4[30] = "R SHOOTER MOTOR UNCONNECTED";
  char msgBuffer5[30] = "LOAD MOTOR UNCONNECTED";
  char msgBuffer6[30] = "FRONT L MOTOR UNCONNECTED";
  char msgBuffer7[30] = "FRONT R MOTOR UNCONNECTED";
  char msgBuffer8[30] = "BACK L MOTOR UNCONNECTED";
  char msgBuffer9[30] = "BACK R MOTOR UNCONNECTED";

  bool pitch_motor_flag_ui = pitch_motor_flag;
  bool yaw_motor_flag_ui = yaw_motor_flag;
  bool sl_motor_flag_ui = sl_motor_flag;
  bool sr_motor_flag_ui = sr_motor_flag;
  bool ld_motor_flag_ui = ld_motor_flag;
  bool fl_motor_flag_ui = fl_motor_flag;
  bool fr_motor_flag_ui = fr_motor_flag;
  bool bl_motor_flag_ui = bl_motor_flag;
  bool br_motor_flag_ui = br_motor_flag;

  // Initialize chassis GUI
  UI->ChassisGUIInit(&graphChassis, &graphArrow, &graphGimbal, &graphCali, &graphEmpty2);
  UI->GraphRefresh((uint8_t*)(&referee->graphic_five), 5, graphChassis, graphArrow, graphGimbal,
                   graphCali, graphEmpty2);
  referee->PrepareUIContent(communication::FIVE_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Initialize crosshair GUI
  UI->CrosshairGUI(&graphCrosshair1, &graphCrosshair2, &graphCrosshair3, &graphCrosshair4,
                   &graphCrosshair5, &graphCrosshair6, &graphCrosshair7);
  UI->GraphRefresh((uint8_t*)(&referee->graphic_seven), 7, graphCrosshair1, graphCrosshair2,
                   graphCrosshair3, graphCrosshair4, graphCrosshair5, graphCrosshair6,
                   graphCrosshair7);
  referee->PrepareUIContent(communication::SEVEN_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Initialize supercapacitor GUI
  UI->CapGUIInit(&graphBarFrame, &graphBar);
  UI->GraphRefresh((uint8_t*)(&referee->graphic_double), 2, graphBarFrame, graphBar);
  referee->PrepareUIContent(communication::DOUBLE_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Initialize Supercapacitor string GUI
  UI->CapGUICharInit(&graphPercent);
  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphPercent, UI->getPercentStr(),
                  UI->getPercentLen());
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Initialize self-diagnosis GUI
  char diagStr[30] = "";
  UI->DiagGUIInit(&graphDiag, 30);
  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphDiag, diagStr, 2);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Initialize current mode GUI
  char followModeStr[15] = "FOLLOW MODE";
  char spinModeStr[15] = "SPIN  MODE";
  uint32_t modeColor = UI_Color_Orange;
  UI->ModeGUIInit(&graphMode);
  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphMode, followModeStr,
                  sizeof followModeStr);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Initialize distance GUI
  char distanceStr[15] = "0.0";
  UI->DistanceGUIInit(&graphDist);
  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphDist, distanceStr,
                  sizeof distanceStr);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // TODO: add lid UI in the future

  //  // Initialize lid status GUI
  //  char lidOpenStr[15] = "LID OPENED";
  //  char lidCloseStr[15] = "LID CLOSED";
  //  UI->LidGUIInit(&graphLid);
  //  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphLid, lidOpenStr, sizeof
  //  lidOpenStr); referee->PrepareUIContent(communication::CHAR_GRAPH); frame =
  //  referee->Transmit(communication::STUDENT_INTERACTIVE); referee_uart->Write(frame.data,
  //  frame.length); osDelay(UI_TASK_DELAY);

  // Initialize flywheel status GUI
  char wheelOnStr[15] = "FLYWHEEL ON";
  char wheelOffStr[15] = "FLYWHEEL OFF";
  UI->WheelGUIInit(&graphWheel);
  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphWheel, wheelOffStr,
                  sizeof wheelOffStr);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  float j = 1;
  while (true) {
    //     lidar_flag = LIDAR->startMeasure();

    // Update chassis GUI
    UI->ChassisGUIUpdate(relative_angle, calibration_flag);
    UI->GraphRefresh((uint8_t*)(&referee->graphic_five), 5, graphChassis, graphArrow, graphGimbal,
                     graphCali, graphEmpty2);
    referee->PrepareUIContent(communication::FIVE_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(UI_TASK_DELAY);

    // Update supercapacitor GUI
    UI->CapGUIUpdate(std::abs(sin(j)));
    UI->GraphRefresh((uint8_t*)(&referee->graphic_single), 1, graphBar);
    referee->PrepareUIContent(communication::SINGLE_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    j += 0.1;
    osDelay(UI_TASK_DELAY);

    // Update supercapacitor string GUI
    UI->CapGUICharUpdate();
    UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphPercent, UI->getPercentStr(),
                    UI->getPercentLen());
    referee->PrepareUIContent(communication::CHAR_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(UI_TASK_DELAY);

    // Update current mode GUI
    char* modeStr = SpinMode ? spinModeStr : followModeStr;
    modeColor = SpinMode ? UI_Color_Green : UI_Color_Orange;
    UI->ModeGuiUpdate(&graphMode, modeColor);
    UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphMode, modeStr, 15);
    referee->PrepareUIContent(communication::CHAR_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(UI_TASK_DELAY);

    // Update distance GUI
    //     uint32_t distColor = UI_Color_Cyan;
    //     float currDist = LIDAR->distance / 1000.0;
    //     if (currDist < 60) {
    //       snprintf(distanceStr, 15, "%.2f m", currDist);
    //       distColor = UI_Color_Cyan;
    //     } else {
    //       snprintf(distanceStr, 15, "ERROR");
    //       distColor = UI_Color_Pink;
    //     }
    //     UI->DistanceGUIUpdate(&graphDist, distColor);
    //     UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphDist, distanceStr, 15);
    //     referee->PrepareUIContent(communication::CHAR_GRAPH);
    //     frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    //     referee_uart->Write(frame.data, frame.length);
    //     osDelay(UI_TASK_DELAY);

    //    // Update lid status GUI
    //    char lidStr[15] = lidFlag ? lidOpenStr : lidCloseStr;
    //    uint32_t lidColor = lidFlag ? UI_Color_Pink : UI_Color_Green;
    //    UI->LidGuiUpdate(&graphLid, lidColor);
    //    UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphLid, lidStr, 15);
    //    referee->PrepareUIContent(communication::CHAR_GRAPH);
    //    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    //    referee_uart->Write(frame.data, frame.length);
    //    osDelay(UI_TASK_DELAY);

    // Update wheel status GUI
    char* wheelStr = flywheelFlag ? wheelOnStr : wheelOffStr;
    uint32_t wheelColor = flywheelFlag ? UI_Color_Pink : UI_Color_Green;
    UI->WheelGUIUpdate(&graphWheel, wheelColor);
    UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphWheel, wheelStr, 15);
    referee->PrepareUIContent(communication::CHAR_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(UI_TASK_DELAY);

    // Update self-diagnosis messages
    if (!pitch_motor_flag_ui && !pitch_motor_flag) {
      UI->AddMessage(msgBuffer1, sizeof msgBuffer1, UI, referee, &graphDiag);
      referee->PrepareUIContent(communication::CHAR_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      pitch_motor_flag_ui = true;
      osDelay(UI_TASK_DELAY);
    }

    if (!yaw_motor_flag_ui && !yaw_motor_flag) {
      UI->AddMessage(msgBuffer2, sizeof msgBuffer2, UI, referee, &graphDiag);
      referee->PrepareUIContent(communication::CHAR_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      yaw_motor_flag_ui = true;
      osDelay(UI_TASK_DELAY);
    }

    if (!sl_motor_flag_ui && !sl_motor_flag) {
      UI->AddMessage(msgBuffer3, sizeof msgBuffer3, UI, referee, &graphDiag);
      referee->PrepareUIContent(communication::CHAR_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      sl_motor_flag_ui = true;
      osDelay(UI_TASK_DELAY);
    }

    if (!sr_motor_flag_ui && !sr_motor_flag) {
      UI->AddMessage(msgBuffer4, sizeof msgBuffer4, UI, referee, &graphDiag);
      referee->PrepareUIContent(communication::CHAR_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      sr_motor_flag_ui = true;
      osDelay(UI_TASK_DELAY);
    }

    if (!ld_motor_flag_ui && !ld_motor_flag) {
      UI->AddMessage(msgBuffer5, sizeof msgBuffer5, UI, referee, &graphDiag);
      referee->PrepareUIContent(communication::CHAR_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      ld_motor_flag_ui = true;
      osDelay(UI_TASK_DELAY);
    }

    if (!fl_motor_flag_ui && !fl_motor_flag) {
      UI->AddMessage(msgBuffer6, sizeof msgBuffer6, UI, referee, &graphDiag);
      referee->PrepareUIContent(communication::CHAR_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      fl_motor_flag_ui = true;
      osDelay(UI_TASK_DELAY);
    }

    if (!fr_motor_flag_ui && !fr_motor_flag) {
      UI->AddMessage(msgBuffer7, sizeof msgBuffer7, UI, referee, &graphDiag);
      referee->PrepareUIContent(communication::CHAR_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      fr_motor_flag_ui = true;
      osDelay(UI_TASK_DELAY);
    }

    if (!bl_motor_flag_ui && !bl_motor_flag) {
      UI->AddMessage(msgBuffer8, sizeof msgBuffer8, UI, referee, &graphDiag);
      referee->PrepareUIContent(communication::CHAR_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      bl_motor_flag_ui = true;
      osDelay(UI_TASK_DELAY);
    }

    if (!br_motor_flag_ui && !br_motor_flag) {
      UI->AddMessage(msgBuffer9, sizeof msgBuffer9, UI, referee, &graphDiag);
      referee->PrepareUIContent(communication::CHAR_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      br_motor_flag_ui = true;
      osDelay(UI_TASK_DELAY);
    }

    // clear self-diagnosis messages
    if (dbus->keyboard.bit.C) {
      for (int i = 1; i <= UI->getMessageCount(); ++i) {
        UI->DiagGUIClear(UI, referee, &graphDiag, i);
        frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
        referee_uart->Write(frame.data, frame.length);
        osDelay(UI_TASK_DELAY);
      }
    }
  }
}

//==================================================================================================
// RM Init
//==================================================================================================

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);

  can1 = new bsp::CAN(&hcan1, true);
  can2 = new bsp::CAN(&hcan2, false);
  dbus = new remote::DBUS(&huart3);
  RGB = new display::RGB(&htim5, 3, 2, 1, 1000000);

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

  laser = new bsp::Laser(LASER_GPIO_Port, LASER_Pin);
  pitch_motor = new control::Motor6020(can1, 0x205);
  yaw_motor = new control::Motor6020(can1, 0x206);
  control::gimbal_t gimbal_data;
  gimbal_data.pitch_motor = pitch_motor;
  gimbal_data.yaw_motor = yaw_motor;
  gimbal_data.model = control::GIMBAL_FORTRESS;
  gimbal = new control::Gimbal(gimbal_data);
  gimbal_param = gimbal->GetData();

  referee_uart = new RefereeUART(&huart6);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);
  referee = new communication::Referee;

  fl_motor = new control::Motor3508(can2, 0x201);
  fr_motor = new control::Motor3508(can2, 0x202);
  bl_motor = new control::Motor3508(can2, 0x203);
  br_motor = new control::Motor3508(can2, 0x204);
  control::MotorCANBase* motors[control::FourWheel::motor_num];
  motors[control::FourWheel::front_left] = fl_motor;
  motors[control::FourWheel::front_right] = fr_motor;
  motors[control::FourWheel::back_left] = bl_motor;
  motors[control::FourWheel::back_right] = br_motor;
  control::chassis_t chassis_data;
  chassis_data.motors = motors;
  chassis_data.model = control::CHASSIS_MECANUM_WHEEL;
  chassis = new control::Chassis(chassis_data);

  sl_motor = new control::Motor3508(can1, 0x201);
  sr_motor = new control::Motor3508(can1, 0x202);
  ld_motor = new control::Motor2006(can1, 0x203);
  control::shooter_t shooter_data;
  shooter_data.left_flywheel_motor = sl_motor;
  shooter_data.right_flywheel_motor = sr_motor;
  shooter_data.load_motor = ld_motor;
  shooter_data.model = control::SHOOTER_STANDARD;
  shooter = new control::Shooter(shooter_data);

  left = new bsp::GPIO(IN1_GPIO_Port, IN1_Pin);
  right = new bsp::GPIO(IN2_GPIO_Port, IN2_Pin);
  elevator_left_motor = new control::Motor3508(can2, 0x205);
  elevator_right_motor = new control::Motor3508(can2, 0x208);
  fortress_motor = new control::Motor6020(can2, 0x207);
  control::fortress_t fortress_data;
  fortress_data.leftSwitch = left;
  fortress_data.rightSwitch = right;
  fortress_data.leftElevatorMotor = elevator_left_motor;
  fortress_data.rightElevatorMotor = elevator_right_motor;
  fortress_data.fortressMotor = fortress_motor;
  fortress = new control::Fortress(fortress_data);

  //  supercap = new control::SuperCap(can2, 0x301);

  buzzer = new bsp::Buzzer(&htim4, 3, 1000000);
  OLED = new display::OLED(&hi2c2, 0x3C);

  //  LIDAR = new distance::LIDAR07_UART(&huart1, [](uint32_t milli) { osDelay(milli); });
  UI = new communication::UserInterface();
}

//==================================================================================================
// RM Thread Init
//==================================================================================================

void RM_RTOS_Threads_Init(void) {
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
  gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
  chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
  shooterTaskHandle = osThreadNew(shooterTask, nullptr, &shooterTaskAttribute);
  fortressTaskHandle = osThreadNew(fortressTask, nullptr, &fortressTaskAttribute);
  selfTestTaskHandle = osThreadNew(selfTestTask, nullptr, &selfTestTaskAttribute);
  UITaskHandle = osThreadNew(UITask, nullptr, &UITaskAttribute);
}

//==================================================================================================
// RM Default Task
//==================================================================================================

void KillAll() {
  RM_EXPECT_TRUE(false, "Operation Killed!\r\n");

  control::MotorCANBase* motors_can1_gimbal[] = {pitch_motor};
  control::MotorCANBase* motors_can2_gimbal[] = {yaw_motor};
  control::MotorCANBase* motors_can2_chassis[] = {fl_motor, fr_motor, bl_motor, br_motor};
  control::MotorCANBase* motors_can1_shooter[] = {sl_motor, sr_motor, ld_motor};
  control::MotorCANBase* motors_can2_elevator[] = {elevator_left_motor, elevator_right_motor};
  control::MotorCANBase* motors_can2_fortress[] = {fortress_motor};

  RGB->Display(display::color_blue);
  laser->Off();

  while (true) {
    FakeDeath.input(dbus->keyboard.bit.B || dbus->swl == remote::DOWN);
    if (FakeDeath.posEdge()) {
      SpinMode = false;
      PeekMode = false;
      Dead = false;
      RGB->Display(display::color_green);
      laser->On();
      break;
    }

    pitch_motor->SetOutput(0);
    yaw_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(motors_can1_gimbal, 1);
    control::MotorCANBase::TransmitOutput(motors_can2_gimbal, 1);

    fl_motor->SetOutput(0);
    bl_motor->SetOutput(0);
    fr_motor->SetOutput(0);
    br_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(motors_can2_chassis, 4);

    sl_motor->SetOutput(0);
    sr_motor->SetOutput(0);
    ld_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(motors_can1_shooter, 3);

    elevator_left_motor->SetOutput(0);
    elevator_right_motor->SetOutput(0);
    fortress_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(motors_can2_elevator, 2);
    control::MotorCANBase::TransmitOutput(motors_can2_fortress, 1);

    osDelay(KILLALL_DELAY);
  }
}

static bool debug = false;
static bool pass = true;

void RM_RTOS_Default_Task(const void* arg) {
  UNUSED(arg);

  while (true) {
    FakeDeath.input(dbus->keyboard.bit.B || dbus->swl == remote::DOWN);
    if (FakeDeath.posEdge()) {
      Dead = true;
      KillAll();
    }

    //    SuperCapKey.input(dbus->keyboard.bit.CTRL);
    //    if (SuperCapKey.posEdge())
    //      UseSuperCap = !UseSuperCap;
    //    if (supercap->info.energy < 15)
    //      UseSuperCap = false;

    if (debug) {
      set_cursor(0, 0);
      clear_screen();

      print("# %.2f s, IMU %s\r\n", HAL_GetTick() / 1000.0,
            imu->CaliDone() ? "\033[1;42mReady\033[0m" : "\033[1;41mNot Ready\033[0m");
      print("Temp: %.2f, Effort: %.2f\r\n", imu->Temp, imu->TempPWM);
      print("Euler Angles: %.2f, %.2f, %.2f\r\n", imu->INS_angle[0] / PI * 180,
            imu->INS_angle[1] / PI * 180, imu->INS_angle[2] / PI * 180);

      print("\r\n");

      print("CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d ", dbus->ch0, dbus->ch1, dbus->ch2, dbus->ch3);
      print("SWL: %d SWR: %d @ %d ms\r\n", dbus->swl, dbus->swr, dbus->timestamp);

      print("\r\n");

      print("%Robot HP: %d / %d\r\n", referee->game_robot_status.remain_HP,
            referee->game_robot_status.max_HP);

      print("\r\n");

      print("Chassis Volt: %.3f\r\n", referee->power_heat_data.chassis_volt / 1000.0);
      print("Chassis Curr: %.3f\r\n", referee->power_heat_data.chassis_current / 1000.0);
      print("Chassis Power: %.2f / %d\r\n", referee->power_heat_data.chassis_power,
            referee->game_robot_status.chassis_power_limit);
      print("Chassis Buffer: %d / 60\r\n", referee->power_heat_data.chassis_power_buffer);

      print("\r\n");

      print("Shooter Heat: %hu / %d\r\n", referee->power_heat_data.shooter_id1_17mm_cooling_heat,
            referee->game_robot_status.shooter_barrel_cooling_value);
      print("Bullet Speed: %.3f / %d\r\n", referee->shoot_data.bullet_speed,
            referee->game_robot_status.shooter_barrel_heat_limit);
      print("Bullet Frequency: %hhu\r\n", referee->shoot_data.bullet_freq);

      if (referee->shoot_data.bullet_speed >
          referee->game_robot_status.shooter_barrel_heat_limit)
        pass = false;
      print("\r\nSpeed Limit Test: %s\r\n", pass ? "PASS" : "FAIL");
    }

    osDelay(DEFAULT_TASK_DELAY);
  }
}

//==================================================================================================
// END
//==================================================================================================
