/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void RM_RTOS_Init(void);
void RM_RTOS_Mutexes_Init(void);
void RM_RTOS_Semaphores_Init(void);
void RM_RTOS_Timers_Init(void);
void RM_RTOS_Queues_Init(void);
void RM_RTOS_Threads_Init(void);
void RM_RTOS_Ready(void);
void RM_RTOS_Default_Task(const void *argument);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Power_OUT1_EN_Pin GPIO_PIN_13
#define Power_OUT1_EN_GPIO_Port GPIOC
#define Power_5V_EN_Pin GPIO_PIN_14
#define Power_5V_EN_GPIO_Port GPIOC
#define RS485_DIR1_Pin GPIO_PIN_15
#define RS485_DIR1_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOC
#define LCD_DC_Pin GPIO_PIN_1
#define LCD_DC_GPIO_Port GPIOC
#define Accel_INT_Pin GPIO_PIN_2
#define Accel_INT_GPIO_Port GPIOC
#define Accel_INT_EXTI_IRQn EXTI2_IRQn
#define Gyro_INT_Pin GPIO_PIN_3
#define Gyro_INT_GPIO_Port GPIOC
#define Gyro_INT_EXTI_IRQn EXTI3_IRQn
#define ADC_VBAT_Pin GPIO_PIN_0
#define ADC_VBAT_GPIO_Port GPIOA
#define ADC_KEY_Pin GPIO_PIN_1
#define ADC_KEY_GPIO_Port GPIOA
#define Power_OUT2_EN_Pin GPIO_PIN_4
#define Power_OUT2_EN_GPIO_Port GPIOC
#define DBUS_Pin GPIO_PIN_5
#define DBUS_GPIO_Port GPIOC
#define Gyro_CS_Pin GPIO_PIN_0
#define Gyro_CS_GPIO_Port GPIOB
#define Accel_CS_Pin GPIO_PIN_1
#define Accel_CS_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_2
#define BUZZER_GPIO_Port GPIOB
#define LCD_Light_Pin GPIO_PIN_8
#define LCD_Light_GPIO_Port GPIOA
#define KEY_Pin GPIO_PIN_15
#define KEY_GPIO_Port GPIOA
#define RS485_DIR2_Pin GPIO_PIN_3
#define RS485_DIR2_GPIO_Port GPIOB
#define LCD_RES_Pin GPIO_PIN_4
#define LCD_RES_GPIO_Port GPIOB
#define IMU_TEMP_Pin GPIO_PIN_5
#define IMU_TEMP_GPIO_Port GPIOB
#define IN1_GPIO_Port GPIOC
#define IN1_Pin GPIO_PIN_6
#define OUT_5V_Port GPIOC
#define OUT_5V_Pin GPIO_PIN_14

/* USER CODE BEGIN Private defines */
#define DM_MC01_GENERAL
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
