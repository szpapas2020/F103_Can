/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STB_Pin GPIO_PIN_13
#define STB_GPIO_Port GPIOC
#define ADC_SYSTEM_VOLTAGE_Pin GPIO_PIN_0
#define ADC_SYSTEM_VOLTAGE_GPIO_Port GPIOC
#define ADC_SYSTEM_CURRENT_Pin GPIO_PIN_1
#define ADC_SYSTEM_CURRENT_GPIO_Port GPIOC
#define ADC_12V_CURRENT_Pin GPIO_PIN_2
#define ADC_12V_CURRENT_GPIO_Port GPIOC
#define ADC_VM_CURRENT_Pin GPIO_PIN_3
#define ADC_VM_CURRENT_GPIO_Port GPIOC
#define MOTOR_IN1_0_Pin GPIO_PIN_0
#define MOTOR_IN1_0_GPIO_Port GPIOA
#define MOTOR_IN2_0_Pin GPIO_PIN_1
#define MOTOR_IN2_0_GPIO_Port GPIOA
#define MOTOR_IN1_1_Pin GPIO_PIN_2
#define MOTOR_IN1_1_GPIO_Port GPIOA
#define MOTOR_IN2_1_Pin GPIO_PIN_3
#define MOTOR_IN2_1_GPIO_Port GPIOA
#define WDI_Pin GPIO_PIN_4
#define WDI_GPIO_Port GPIOA
#define BEEPER_Pin GPIO_PIN_5
#define BEEPER_GPIO_Port GPIOA
#define LM5176_EN_Pin GPIO_PIN_6
#define LM5176_EN_GPIO_Port GPIOA
#define KEY_Pin GPIO_PIN_7
#define KEY_GPIO_Port GPIOA
#define KEY_EXTI_IRQn EXTI9_5_IRQn
#define PWR_12V_EN_Pin GPIO_PIN_4
#define PWR_12V_EN_GPIO_Port GPIOC
#define PWR_5V_EN_Pin GPIO_PIN_5
#define PWR_5V_EN_GPIO_Port GPIOC
#define ADC_VBAT_CHAG_CURRENT_Pin GPIO_PIN_0
#define ADC_VBAT_CHAG_CURRENT_GPIO_Port GPIOB
#define INT_Pin GPIO_PIN_2
#define INT_GPIO_Port GPIOB
#define INT_EXTI_IRQn EXTI2_IRQn
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOB
#define LM5176_VIN_EN_Pin GPIO_PIN_15
#define LM5176_VIN_EN_GPIO_Port GPIOB
#define MOTOR_IN2_2_Pin GPIO_PIN_6
#define MOTOR_IN2_2_GPIO_Port GPIOC
#define MOTOR_IN1_2_Pin GPIO_PIN_7
#define MOTOR_IN1_2_GPIO_Port GPIOC
#define MOTOR_IN2_3_Pin GPIO_PIN_8
#define MOTOR_IN2_3_GPIO_Port GPIOC
#define MOTOR_IN1_3_Pin GPIO_PIN_9
#define MOTOR_IN1_3_GPIO_Port GPIOC
#define MOTOR_IN2_4_Pin GPIO_PIN_8
#define MOTOR_IN2_4_GPIO_Port GPIOA
#define MOTOR_IN1_4_Pin GPIO_PIN_9
#define MOTOR_IN1_4_GPIO_Port GPIOA
#define MOTOR_IN2_5_Pin GPIO_PIN_10
#define MOTOR_IN2_5_GPIO_Port GPIOA
#define MOTOR_IN1_5_Pin GPIO_PIN_11
#define MOTOR_IN1_5_GPIO_Port GPIOA
#define RS485_EN_1_Pin GPIO_PIN_15
#define RS485_EN_1_GPIO_Port GPIOA
#define MOTOR_IN2_6_Pin GPIO_PIN_12
#define MOTOR_IN2_6_GPIO_Port GPIOC
#define MOTOR_IN1_6_Pin GPIO_PIN_2
#define MOTOR_IN1_6_GPIO_Port GPIOD
#define RS485_EN_0_Pin GPIO_PIN_3
#define RS485_EN_0_GPIO_Port GPIOB
#define MOTOR_IN2_7_Pin GPIO_PIN_4
#define MOTOR_IN2_7_GPIO_Port GPIOB
#define MOTOR_IN1_7_Pin GPIO_PIN_5
#define MOTOR_IN1_7_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
