#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"

/**
 * @brief when installing, please use this example to set angle to 0.0 degree for calibration.
 * The current program is functioning, however, might not be 100% accurate
 */

// All of these following parameters are tuned for this servo.
uint8_t PWM_CHANNEL = 1;
uint32_t TIM_CLOCK_FREQ = 1000000; /* TODO: could use more calibration if data is available*/
// rule of thumb, TIM_CLOCK_FREQ could use more calibration if more data is available for PDI-HV5932
uint32_t MOTOR_OUT_FREQ = 50; /* TODO: could use more calibration if data is available*/
uint32_t PULSE_WIDTH = 1000;
// PULSE_WIDTH when servo is idle

bsp::GPIO* key = nullptr;

#define KEY_GPIO_GROUP GPIOA
#define KEY_GPIO_PIN GPIO_PIN_0
#define MAP_RANGE(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

control::MotorPWMBase* trigger_motor;

remote::DBUS* dbus = nullptr;

void RM_RTOS_Init(){
  print_use_uart(&huart1);
  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);
  trigger_motor = new control::MotorPWMBase(&htim1, PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, 0);
  dbus = new remote::DBUS(&huart3);
  //  motor1->SetOutput(1500);
  osDelay(300);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  int power = 1000;

  while (true) {
    power = MAP_RANGE(dbus->ch3, -660, 660, 500, 1000);
    trigger_motor->SetOutput(power);
    print("power: %d\r\n", power);
    osDelay(20);
  }
  }
