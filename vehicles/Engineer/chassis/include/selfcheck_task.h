#include "cmsis_os.h"
#include "motor.h"



extern osThreadId_t selfTestTaskHandle;
const osThreadAttr_t selfTestingTask = {.name = "selfTestTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 256 * 4,
                                             .priority = (osPriority_t)osPriorityBelowNormal,
                                             .tz_module = 0,
                                             .reserved = 0};

static bool fl_steer_motor_flag = false;
static bool fr_steer_motor_flag = false;
static bool bl_steer_motor_flag = false;
static bool br_steer_motor_flag = false;
static bool fl_wheel_motor_flag = false;
static bool fr_wheel_motor_flag = false;
static bool bl_wheel_motor_flag = false;
static bool br_wheel_motor_flag = false;


extern control::MotorCANBase* motor1;
extern control::MotorCANBase* motor2;
extern control::MotorCANBase* motor3;
extern control::MotorCANBase* motor4;
extern control::MotorCANBase* motor5;
extern control::MotorCANBase* motor6;
extern control::MotorCANBase* motor7;
extern control::MotorCANBase* motor8;

static unsigned int flag_summary = 0;

