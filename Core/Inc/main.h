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
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "tim.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
void encoder_init();

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
#define RR_LED_Pin GPIO_PIN_0
#define RR_LED_GPIO_Port GPIOC
#define RR_ADC1_Pin GPIO_PIN_1
#define RR_ADC1_GPIO_Port GPIOC
#define RF_LED_Pin GPIO_PIN_2
#define RF_LED_GPIO_Port GPIOC
#define EncoderL_TIM2_CH1_Pin GPIO_PIN_0
#define EncoderL_TIM2_CH1_GPIO_Port GPIOA
#define EncoderL_TIM2_CH2_Pin GPIO_PIN_1
#define EncoderL_TIM2_CH2_GPIO_Port GPIOA
#define EncoderR_TIM3_CH1_Pin GPIO_PIN_6
#define EncoderR_TIM3_CH1_GPIO_Port GPIOA
#define EncoderR_TIM3_CH2_Pin GPIO_PIN_7
#define EncoderR_TIM3_CH2_GPIO_Port GPIOA
#define LL_LED_Pin GPIO_PIN_0
#define LL_LED_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define SW0_Pin GPIO_PIN_7
#define SW0_GPIO_Port GPIOC
#define SW1_Pin GPIO_PIN_8
#define SW1_GPIO_Port GPIOC
#define SW2_Pin GPIO_PIN_9
#define SW2_GPIO_Port GPIOC
#define MotorL_TIM1_CH1_Pin GPIO_PIN_8
#define MotorL_TIM1_CH1_GPIO_Port GPIOA
#define MotorL_TIM1_CH2_Pin GPIO_PIN_9
#define MotorL_TIM1_CH2_GPIO_Port GPIOA
#define MotorR_TIM1_CH3_Pin GPIO_PIN_10
#define MotorR_TIM1_CH3_GPIO_Port GPIOA
#define MotorR_TIM1_CH4_Pin GPIO_PIN_11
#define MotorR_TIM1_CH4_GPIO_Port GPIOA
#define Motor_Mode_Pin GPIO_PIN_12
#define Motor_Mode_GPIO_Port GPIOA
#define Debug_LED_Pin GPIO_PIN_9
#define Debug_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
