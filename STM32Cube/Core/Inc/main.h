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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MP6500_CS_Pin GPIO_PIN_14
#define MP6500_CS_GPIO_Port GPIOC
#define HALL1_EXTI_Pin GPIO_PIN_0
#define HALL1_EXTI_GPIO_Port GPIOA
#define HALL1_EXTI_EXTI_IRQn EXTI0_IRQn
#define HALL2_EXTI_Pin GPIO_PIN_1
#define HALL2_EXTI_GPIO_Port GPIOA
#define HALL2_EXTI_EXTI_IRQn EXTI1_IRQn
#define IR1_ADC_Pin GPIO_PIN_2
#define IR1_ADC_GPIO_Port GPIOA
#define IR4_ADC_Pin GPIO_PIN_5
#define IR4_ADC_GPIO_Port GPIOA
#define IR5_ADC_Pin GPIO_PIN_6
#define IR5_ADC_GPIO_Port GPIOA
#define TP4056_CHRG_Pin GPIO_PIN_7
#define TP4056_CHRG_GPIO_Port GPIOA
#define HALL4_EXTI_Pin GPIO_PIN_4
#define HALL4_EXTI_GPIO_Port GPIOC
#define HALL4_EXTI_EXTI_IRQn EXTI4_IRQn
#define MOT_PWR_EN_Pin GPIO_PIN_5
#define MOT_PWR_EN_GPIO_Port GPIOC
#define IR3_ADC_Pin GPIO_PIN_0
#define IR3_ADC_GPIO_Port GPIOB
#define IR2_ADC_Pin GPIO_PIN_1
#define IR2_ADC_GPIO_Port GPIOB
#define WS2812B_Pin GPIO_PIN_2
#define WS2812B_GPIO_Port GPIOB
#define MOT2P1_Pin GPIO_PIN_6
#define MOT2P1_GPIO_Port GPIOC
#define MOT2P2_Pin GPIO_PIN_7
#define MOT2P2_GPIO_Port GPIOC
#define MOT1P1_Pin GPIO_PIN_8
#define MOT1P1_GPIO_Port GPIOC
#define MOT1P2_Pin GPIO_PIN_9
#define MOT1P2_GPIO_Port GPIOC
#define TP4056_STDBY_Pin GPIO_PIN_8
#define TP4056_STDBY_GPIO_Port GPIOA
#define HALL3_EXTI_Pin GPIO_PIN_15
#define HALL3_EXTI_GPIO_Port GPIOA
#define HALL3_EXTI_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
