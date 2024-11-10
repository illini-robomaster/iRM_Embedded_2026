#pragma once

#include "cmsis_os.h"
#include "motor.h"

#include "can.h"
#include "bsp_gpio.h"
#include "bsp_print.h"
#include "controller.h"
#include "bsp_can_bridge.h"
// #include "utils.h"
#ifdef USING_DBUS
#include "dbus.h"
#else
#include "sbus.h"
#endif

extern osThreadId_t armTranslateTaskHandle;
const osThreadAttr_t armTranslateAttribute = {.name = "armTranslateTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 512 * 4,
                                             .priority = (osPriority_t)osPriorityAboveNormal,
                                             .tz_module = 0,
                                             .reserved = 0};



void armTranslateTask(void* arg);
void init_arm_translate();
void kill_arm_translate();
void revive_arm_translate();


/* M3508 params start */ 
const int BASE_TRANSLATE_ID = 0x209; // TODO: need to reconnect
//const GPIO_TypeDef* BASE_TRANS_INIT_GPIO_PORT = GPIOI;
//const uint16_t BASE_TRANS_INIT_GPIO_PIN = GPIO_PIN_7;  // PWM pin 1 on C board

// M3508 steering params. (translate 3508 motor) */
const float BASE_TRANSLATE_MAX = PI;
const float BASE_TRANSLATE_MIN = -PI;
const float BASE_TRANSLATE_RUN_SPEED = (2 * PI);
const float BASE_TRANSLATE_ALIGN_SPEED = (0.5 * PI);
const float BASE_TRANSLATE_ACCELERATION = (100 * PI);

/* M3508 params end */
#ifdef USING_DBUS
extern remote::DBUS* dbus;
#else
extern remote::SBUS* sbus;
#endif

extern bsp::CAN* can1;
extern bsp::CAN* can2;
extern bsp::CanBridge* receive;
