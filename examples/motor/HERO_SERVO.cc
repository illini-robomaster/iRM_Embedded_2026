#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"

/**
 * @brief when installing, please use this example to set angle to 0.0 degree for calibration.
 * The current program is functioning, however, might not be 100% accurate
 */

// All of these following parameters are tuned for this servo.
uint8_t PWM_CHANNEL = 1;
uint32_t TIM_CLOCK_FREQ = 1980000; /* TODO: could use more calibration if data is available*/
// rule of thumb, TIM_CLOCK_FREQ could use more calibration if more data is available for PDI-HV5932
uint32_t MOTOR_OUT_FREQ = 500; /* TODO: could use more calibration if data is available*/
uint32_t PULSE_WIDTH = 1500;
// PULSE_WIDTH when servo is idle

bsp::GPIO* key = nullptr;

#define KEY_GPIO_GROUP GPIOA
#define KEY_GPIO_PIN GPIO_PIN_0

control::PDIHV* trigger_motor;

void RM_RTOS_Init(){
  print_use_uart(&huart1);
  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);
  trigger_motor = new control::PDIHV(&htim1, PWM_CHANNEL,TIM_CLOCK_FREQ,MOTOR_OUT_FREQ, PULSE_WIDTH);
//  motor1->SetOutput(1500);
  osDelay(300);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  float angle = 0.0;
  // angle could be set to 80 ~ -80 deg
  int16_t power = 1947;
  // power is from range 972 to 1947, data on purchasing page is not available, pulse width for central point is 1500
  while(true){
    trigger_motor->SetOutPutAngle(angle);

//    motor1->SetOutput(power);
    if(!key->Read()){
      angle += 10.0;
      power -= 10;
    }
//    power += 10;
    osDelay(2);
//    angle += 1.0;
    print("angle: %f\r\n", angle);
    print("power: %d\r\n", power);
    print("\r\n");


  }
  }
