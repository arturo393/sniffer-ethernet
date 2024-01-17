
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : eeprom.h
 * @brief          : Header for eeprom.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 *
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LM75_H
#define __LM75_H


#include "main.h"


extern I2C_HandleTypeDef hi2c1;

//  LM75B IIC address
#define    LM75_ADDR 0x4f
#define LM75_TIMEOUT 5000
//  LM75B registers
typedef enum LM75_REG {
	LM75_Temp, LM75_Conf, LM75_Thyst, LM75_Tos
} LM75_REG_t;

HAL_StatusTypeDef lm75_init();
uint16_t lm75_read(void);

#endif /* __LM75_H */
