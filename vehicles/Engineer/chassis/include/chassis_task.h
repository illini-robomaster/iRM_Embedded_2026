#pragma once
#include "cmsis_os.h"
#include "motor.h"
#include "engineer_steering.h"
#include "steering_6020.h"
#include "can.h"
#include "rgb.h"
#include "protocol.h"
#include "bsp_can_bridge.h"

#ifdef USING_DBUS

#include "dbus.h"
#else
#include "sbus.h"
#endif

//MAGIC NUMBERS, Represent motor physical install angle offsets.
// #define FL_MOTOR_OFFSET 4.663
// #define FR_MOTOR_OFFSET 0.771
// #define BL_MOTOR_OFFSET 6.110
// #define BR_MOTOR_OFFSET 6.065
#define FL_MOTOR_OFFSET 1.250961 
#define FR_MOTOR_OFFSET -2.408350
#define BL_MOTOR_OFFSET 2.301738 
#define BR_MOTOR_OFFSET 2.407583 


extern osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTaskAttribute = {.name = "chassisTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 512 * 4,
                                             .priority = (osPriority_t)osPriorityAboveNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
void chassisTask(void* arg);
void init_chassis();
void kill_chassis();
void revive_chassis();

extern remote::SBUS* sbus;

extern bsp::CAN* can1;
extern bsp::CAN* can2;
extern display::RGB* RGB;
extern communication::Referee* referee;
extern bsp::CanBridge* receive;
extern control::Motor4310* rotate_motor;


// speed for steering motors (rad/s)
constexpr float RUN_SPEED = (4 * PI);
constexpr float ALIGN_SPEED = (PI);
constexpr float ACCELERATION = (100 * PI);
// speed for chassis rotation (no unit)
constexpr float FOLLOW_SPEED = 40;
constexpr float CHASSIS_DEADZONE = 0.04;
