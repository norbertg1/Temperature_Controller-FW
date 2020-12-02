/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "temp_controller.h"
#include "u8g2_callback.h"
#include "INA226.h"
#include "snake_start.h"
#include <flash.h>
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
extern I2C_HandleTypeDef hi2c1;
extern INA226 INA226_1,INA226_2;
extern SDADC_HandleTypeDef hsdadc1;
extern SDADC_HandleTypeDef hsdadc2;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENCODER_PUSH_BUTTON_Pin GPIO_PIN_13
#define ENCODER_PUSH_BUTTON_GPIO_Port GPIOC
#define ENCODER_PUSH_BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define ENCODER_A_Pin GPIO_PIN_14
#define ENCODER_A_GPIO_Port GPIOC
#define ENCODER_A_EXTI_IRQn EXTI15_10_IRQn
#define ENCODER_B_Pin GPIO_PIN_15
#define ENCODER_B_GPIO_Port GPIOC
#define ENCODER_B_EXTI_IRQn EXTI15_10_IRQn
#define PWM2_Pin GPIO_PIN_1
#define PWM2_GPIO_Port GPIOA
#define EN2_Pin GPIO_PIN_2
#define EN2_GPIO_Port GPIOA
#define GREEN_LED_Pin GPIO_PIN_3
#define GREEN_LED_GPIO_Port GPIOA
#define PUSH_BUTTON_Pin GPIO_PIN_4
#define PUSH_BUTTON_GPIO_Port GPIOA
#define RED_LED_Pin GPIO_PIN_5
#define RED_LED_GPIO_Port GPIOA
#define ADC_RES__Pin GPIO_PIN_0
#define ADC_RES__GPIO_Port GPIOB
#define ADC_RES_B1_Pin GPIO_PIN_1
#define ADC_RES_B1_GPIO_Port GPIOB
#define NTC__Pin GPIO_PIN_8
#define NTC__GPIO_Port GPIOE
#define NTC_E9_Pin GPIO_PIN_9
#define NTC_E9_GPIO_Port GPIOE
#define PWM1_Pin GPIO_PIN_15
#define PWM1_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
