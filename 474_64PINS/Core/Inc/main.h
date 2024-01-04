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
#include "stm32g4xx_hal.h"

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
#define H_F_Pin GPIO_PIN_13
#define H_F_GPIO_Port GPIOC
#define TE485_Pin GPIO_PIN_15
#define TE485_GPIO_Port GPIOC
#define SPI2_NSS_Pin GPIO_PIN_0
#define SPI2_NSS_GPIO_Port GPIOF
#define NINT_W5500_Pin GPIO_PIN_0
#define NINT_W5500_GPIO_Port GPIOC
#define DOUT2_Pin GPIO_PIN_2
#define current_out_select_Pin GPIO_PIN_6
#define current_in_select_Pin GPIO_PIN_7
#define DOUT2_GPIO_Port GPIOC
#define DIN2_Pin GPIO_PIN_3
#define DIN2_GPIO_Port GPIOC

#define AIN_1_0_10V_Pin GPIO_PIN_0
#define AIN_1_0_10V_GPIO_Port GPIOA
#define AIN_X_20MA_Pin GPIO_PIN_2
#define AIN_X_20MA_GPIO_Port GPIOA


#define KA_Pin GPIO_PIN_3
#define KA_GPIO_Port GPIOA

//#define DIN_Pin GPIO_PIN_7
//#define DIN_GPIO_Port GPIOA
#define AOUT1_0_10V_Pin GPIO_PIN_4
#define AOUT1_0_10V_GPIO_Port GPIOA
#define SWSERIAL_Pin GPIO_PIN_5//agregado
#define SWSERIAL_GPIO_Port GPIOA//agregado
#define AOUT2_X_20MA_Pin GPIO_PIN_6
#define AOUT2_X_20MA_GPIO_Port GPIOA
#define DIN1_Pin GPIO_PIN_7
#define DIN1_GPIO_Port GPIOA

#define DOUT_Pin GPIO_PIN_0
#define DOUT_GPIO_Port GPIOB
#define BUSSY_1_Pin GPIO_PIN_1
#define BUSSY_1_GPIO_Port GPIOB
#define LORA_TX_Pin GPIO_PIN_2
#define LORA_TX_GPIO_Port GPIOB
#define NRST_LORA_1_Pin GPIO_PIN_10
#define NRST_LORA_1_GPIO_Port GPIOB
#define DIO1_1_Pin GPIO_PIN_12

#define DIO1_1_GPIO_Port GPIOB
#define DIO3_1_Pin GPIO_PIN_13
#define DIO3_1_GPIO_Port GPIOB


/*
#define current_out_select_Pin GPIO_PIN_6
#define current_out_select_GPIO_Port GPIOC
#define current_in_select_Pin GPIO_PIN_7
#define current_in_select_GPIO_Port GPIOC
*/
#define SWOUT_X_20MA_Pin GPIO_PIN_6
#define SWOUT_X_20MA_GPIO_Port GPIOC
#define SWIN_X_20MA_Pin GPIO_PIN_7
#define SWIN_X_20MA_GPIO_Port GPIOC

#define SPI1_NSS_Pin GPIO_PIN_15
#define SPI1_NSS_GPIO_Port GPIOA
#define BUSSY_2_Pin GPIO_PIN_10
#define BUSSY_2_GPIO_Port GPIOC
#define DIO1_2_Pin GPIO_PIN_11
#define DIO1_2_GPIO_Port GPIOC
#define DIO3_2_Pin GPIO_PIN_12
#define DIO3_2_GPIO_Port GPIOC
#define NRST_LORA_2_Pin GPIO_PIN_2
#define NRST_LORA_2_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
