/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define wheel 6.4f
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EA1_Pin GPIO_PIN_0
#define EA1_GPIO_Port GPIOA
#define EA2_Pin GPIO_PIN_1
#define EA2_GPIO_Port GPIOA
#define EC1_Pin GPIO_PIN_6
#define EC1_GPIO_Port GPIOA
#define EB2_Pin GPIO_PIN_11
#define EB2_GPIO_Port GPIOE
#define PWMA_Pin GPIO_PIN_6
#define PWMA_GPIO_Port GPIOC
#define PWMB_Pin GPIO_PIN_7
#define PWMB_GPIO_Port GPIOC
#define PWMC_Pin GPIO_PIN_8
#define PWMC_GPIO_Port GPIOC
#define PWMD_Pin GPIO_PIN_9
#define PWMD_GPIO_Port GPIOC
#define EB1_Pin GPIO_PIN_8
#define EB1_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_3
#define BIN2_GPIO_Port GPIOD
#define BIN1_Pin GPIO_PIN_4
#define BIN1_GPIO_Port GPIOD
#define AIN2_Pin GPIO_PIN_5
#define AIN2_GPIO_Port GPIOD
#define AIN1_Pin GPIO_PIN_6
#define AIN1_GPIO_Port GPIOD
#define EC2_Pin GPIO_PIN_5
#define EC2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern volatile int32_t encoder_count_prev_E2;

typedef struct speed_data{
  float speed;
  float distance;//
  int32_t encoder_count_prev;
  int32_t current_count;
  float delta_pulses;
  float delta_distance;
} Speed_Data;
extern char imu_rx_buffer[512]__attribute__((section(".out")));
extern char speed_rx_buffer[512]__attribute__((section(".out")));
extern char screen_rx_buffer[512]__attribute__((section(".out")));
extern char k230_rx_buffer[2048]__attribute__((section(".o2")));
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
