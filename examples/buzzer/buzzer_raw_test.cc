/**
 * @brief Raw register test for TIM12 buzzer on DM-MC-02
 * 
 * This test directly manipulates TIM12 registers to verify hardware works.
 * TIM12_CH2 is on PB15.
 */

#include "main.h"
#include "tim.h"
#include "cmsis_os.h"

// Clock: 80MHz / 24 = 3.33MHz
// For 523Hz (Do1M): ARR = 3333333 / 523 - 1 = 6373
// For 659Hz (Mi3M): ARR = 3333333 / 659 - 1 = 5056
// For 784Hz (So5M): ARR = 3333333 / 784 - 1 = 4251

#define TIM12_CLOCK  (80000000 / 24)  // 3.33MHz after prescaler

static void buzzer_set_freq(uint32_t freq) {
  if (freq == 0) {
    // Silent
    TIM12->CCR2 = 0;
  } else {
    uint32_t arr = TIM12_CLOCK / freq - 1;
    uint32_t ccr = arr / 2;  // 50% duty cycle
    
    TIM12->ARR = arr;
    TIM12->CCR2 = ccr;
    TIM12->CNT = 0;  // Reset counter
    TIM12->EGR = TIM_EGR_UG;  // Generate update event
  }
}

void RM_RTOS_Init(void) {
  // Start PWM
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);
  
  // Test frequencies (Hz)
  uint32_t notes[] = {523, 587, 659, 698, 784, 880, 988, 1047};  // Do Re Mi Fa So La Si Do
  int idx = 0;
  
  while (true) {
    buzzer_set_freq(notes[idx]);
    osDelay(500);
    
    idx = (idx + 1) % 8;
  }
}
