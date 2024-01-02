/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : m24c64.h
 * @brief          : Header for m24c64.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 *
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __K24C02_H
#define K24C02_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stdlib.h"

#define CHIP_ADDR 0x50
#define PAGE_SIZE 16
#define PAGE_NUM 32
#define IS_READY 0xaa
#define PADDRPOSITION 3

// EEPROM ADDRESS (8bits)
#define EEPROM_ADDR 0x50
//#define EEPROM_ADDR 0x50

#define BASE_ADDR 0x03

#define K24C02_PAGES 32
#define K24C02_PAGE_SIZE 8

#define K24C02_PAGE_ADDR(page) ((page) * K24C02_PAGE_SIZE)
#define K24C02_PAGE_START_ADDR(page) K24C02_PAGE_ADDR(page)
#define K24C02_PAGE_END_ADDR(page) (K24C02_PAGE_ADDR(page) + K24C02_PAGE_SIZE - 1)

#define K24C02_PAGE0 0
#define K24C02_PAGE1 1
#define K24C02_PAGE2 2

typedef struct {
	uint8_t page;
	uint8_t offset;
	uint8_t size;
	I2C_HandleTypeDef *i2c;
}K24C02_t;

K24C02_t* k24c02();
uint8_t readByte(I2C_HandleTypeDef*i2c ,uint8_t page,uint8_t offset);
uint32_t read4Byte(I2C_HandleTypeDef*i2c ,uint8_t page,uint8_t offset);

uint16_t read2Byte(I2C_HandleTypeDef *i2c, uint8_t page, uint8_t offset);
HAL_StatusTypeDef readPage(I2C_HandleTypeDef*i2c ,uint8_t page, uint8_t *data, uint8_t offset, uint8_t size);
HAL_StatusTypeDef savePage(I2C_HandleTypeDef*i2c , uint8_t page, uint8_t *data, uint8_t offset, uint8_t size);
HAL_StatusTypeDef saveByte(I2C_HandleTypeDef *i2c, uint8_t page,uint8_t *data, uint8_t offset);
HAL_StatusTypeDef save2Byte(I2C_HandleTypeDef *i2c, uint8_t page, uint8_t *data,uint8_t offset);
HAL_StatusTypeDef save4Byte(I2C_HandleTypeDef *i2c, uint8_t page, uint8_t *data,uint8_t offset);


#ifdef __cplusplus
}
#endif

#endif /* M24C64_H */
