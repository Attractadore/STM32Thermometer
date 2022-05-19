/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define DebugLED_Pin GPIO_PIN_13
#define DebugLED_GPIO_Port GPIOC
#define DPY_CONTRAST_Pin GPIO_PIN_0
#define DPY_CONTRAST_GPIO_Port GPIOA
#define DPY_RS_Pin GPIO_PIN_12
#define DPY_RS_GPIO_Port GPIOB
#define DPY_EN_Pin GPIO_PIN_13
#define DPY_EN_GPIO_Port GPIOB
#define DPY_4_Pin GPIO_PIN_14
#define DPY_4_GPIO_Port GPIOB
#define DPY_5_Pin GPIO_PIN_15
#define DPY_5_GPIO_Port GPIOB
#define DPY_6_Pin GPIO_PIN_8
#define DPY_6_GPIO_Port GPIOA
#define DPY_7_Pin GPIO_PIN_9
#define DPY_7_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
