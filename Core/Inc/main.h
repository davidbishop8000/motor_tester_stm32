/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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
#define RE_DE_Pin GPIO_PIN_13
#define RE_DE_GPIO_Port GPIOC
#define OLED_DC_Pin GPIO_PIN_6
#define OLED_DC_GPIO_Port GPIOA
#define OLED_Res_Pin GPIO_PIN_0
#define OLED_Res_GPIO_Port GPIOB
#define OLED_CS_Pin GPIO_PIN_1
#define OLED_CS_GPIO_Port GPIOB
#define BUTTON1_Pin GPIO_PIN_12
#define BUTTON1_GPIO_Port GPIOB
#define BUTTON2_Pin GPIO_PIN_13
#define BUTTON2_GPIO_Port GPIOB
#define BUTTON3_Pin GPIO_PIN_14
#define BUTTON3_GPIO_Port GPIOB
#define BUTTON4_Pin GPIO_PIN_15
#define BUTTON4_GPIO_Port GPIOB
#define RE_DE_ON HAL_GPIO_WritePin(RE_DE_GPIO_Port, RE_DE_Pin, GPIO_PIN_SET); HAL_Delay(2)
#define RE_DE_OFF HAL_GPIO_WritePin(RE_DE_GPIO_Port, RE_DE_Pin, GPIO_PIN_RESET); HAL_Delay(1)

/* USER CODE BEGIN Private defines */
#define ENC_ONE_PERIOD 65536
#define ENC_HALF_PERIOD 32768

void rcGetBattery();
int32_t unwrap_encoder(uint16_t in, int32_t *prev);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
