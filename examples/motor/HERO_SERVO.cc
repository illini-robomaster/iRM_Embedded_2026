#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"
#include "bsp_pwm.h"
#include "bsp_buzzer.h"
/**
 * @brief when installing, please use this example to set angle to 0.0 degree for calibration.
 * The current program is functioning, however, might not be 100% accurate
 */

#define neutralPOS 1500

// All of these following parameters are tuned for this servo.
uint8_t PWM_CHANNEL = 1;
uint32_t TIM_CLOCK_FREQ = 1000000; /* TODO: could use more calibration if data is available*/
// rule of thumb, TIM_CLOCK_FREQ could use more calibration if more data is available for PDI-HV5932
uint32_t MOTOR_OUT_FREQ = 50; /* TODO: could use more calibration if data is available*/
uint32_t PULSE_WIDTH = 1500;
// PULSE_WIDTH when servo is idle
int16_t output;
bsp::GPIO* key = nullptr;
BoolEdgeDetector keyEdgeDetector(false);
control::MotorPWMBase* motor1;


//bsp::GPIO* pe = nullptr;

bool nintyDegree = false;

void RM_RTOS_Init(){
  print_use_uart(&huart6);
  key = new bsp::GPIO(KEY_GPIO_Port, KEY_Pin);

  motor1 = new control::MotorPWMBase(&htim1, PWM_CHANNEL,TIM_CLOCK_FREQ,MOTOR_OUT_FREQ, PULSE_WIDTH);
//  motor1->SetOutput(1500);
  osDelay(300);
//  pe = new bsp::GPIO(IN1_GPIO_Port, IN1_Pin);
  output = 50;
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  bsp::Buzzer buzzer(&htim4, 3, 1000000);
  // power is from range 972 to 1947, data on purchasing page is not available, pulse width for central point is 1500
  while(true){
    keyEdgeDetector.input(!(key->Read()));
    if(keyEdgeDetector.posEdge()){
      nintyDegree = !nintyDegree;
    }
    if(nintyDegree) {
      output = 3;
    } else {
      output = 1500;
    }
    motor1->SetOutput(output);
//    if (pe->Read()==0){
//      print("pe is 0\r\n");
//    } else {
//      print("no read\r\n");
//    }
//    power += 10;
    osDelay(2);
//    angle += 1.0;m
  }
}