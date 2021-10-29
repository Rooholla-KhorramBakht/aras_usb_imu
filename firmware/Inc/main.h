/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define MOOD1_Pin GPIO_PIN_10
#define MOOD1_GPIO_Port GPIOB
#define MOOD0_Pin GPIO_PIN_11
#define MOOD0_GPIO_Port GPIOB
#define rc_ch1_Pin GPIO_PIN_12
#define rc_ch1_GPIO_Port GPIOB
#define rc_ch1_EXTI_IRQn EXTI15_10_IRQn
#define rc_ch2_Pin GPIO_PIN_13
#define rc_ch2_GPIO_Port GPIOB
#define rc_ch2_EXTI_IRQn EXTI15_10_IRQn
#define rc_ch3_Pin GPIO_PIN_14
#define rc_ch3_GPIO_Port GPIOB
#define rc_ch3_EXTI_IRQn EXTI15_10_IRQn
#define rc_ch4_Pin GPIO_PIN_15
#define rc_ch4_GPIO_Port GPIOB
#define rc_ch4_EXTI_IRQn EXTI15_10_IRQn
#define rc_ch8_Pin GPIO_PIN_3
#define rc_ch8_GPIO_Port GPIOB
#define rc_ch8_EXTI_IRQn EXTI3_IRQn
#define cam_trigg_Pin GPIO_PIN_4
#define cam_trigg_GPIO_Port GPIOB
#define cam_trigg_EXTI_IRQn EXTI4_IRQn
#define DIR_Pin GPIO_PIN_6
#define DIR_GPIO_Port GPIOB
#define STEP_Pin GPIO_PIN_7
#define STEP_GPIO_Port GPIOB
#define trigger_output_Pin GPIO_PIN_9
#define trigger_output_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
