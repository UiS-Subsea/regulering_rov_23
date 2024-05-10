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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rovconfig.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern double tau[6];
extern double U[8]; // newtons
extern double u[8]; // percent
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Test_Handler(void);
void canfd_callback(uint16_t id, void* data);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_5_Pin GPIO_PIN_0
#define PWM_5_GPIO_Port GPIOA
#define PWM_6_Pin GPIO_PIN_1
#define PWM_6_GPIO_Port GPIOA
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define PWM_8_Pin GPIO_PIN_4
#define PWM_8_GPIO_Port GPIOA
#define PWM_7_Pin GPIO_PIN_6
#define PWM_7_GPIO_Port GPIOA
#define PWM_4_Pin GPIO_PIN_0
#define PWM_4_GPIO_Port GPIOB
#define PWM_2_Pin GPIO_PIN_9
#define PWM_2_GPIO_Port GPIOA
#define PWM_3_Pin GPIO_PIN_10
#define PWM_3_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_5
#define SPI1_CS_GPIO_Port GPIOB
#define PWM_1_Pin GPIO_PIN_7
#define PWM_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define PWM1 TIM1->CCR1
#define PWM2 TIM1->CCR2
#define PWM3 TIM1->CCR3
#define PWM4 TIM1->CCR4
#define PWM5 TIM2->CCR1
#define PWM6 TIM2->CCR2
#define PWM7 TIM3->CCR1
#define PWM8 TIM3->CCR2

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
