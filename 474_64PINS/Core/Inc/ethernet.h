/*
 * ethernet.h
 *
 *  Created on: Jan 8, 2024
 *      Author: Alfonso
 */

#ifndef INC_ETHERNET_H_
#define INC_ETHERNET_H_
#include "main.h"
#include "SX1278.h"
#include "rdss.h"
#include "stm32g4xx_hal_spi.h"

extern SPI_HandleTypeDef hspi1;
extern uint8_t *p;
extern uint8_t data_ethernet;
// Funciones de transmisión recepción vía SPI:
void transmitir_spi(uint8_t* p, uint8_t len);
void transmitir_recibir_spi(uint8_t* p_t, uint8_t len_t, uint8_t* p_r, uint16_t len_r);

// Funciones para el ajuste de los registros del chip w5500:
void common_register_block(uint8_t* buff, uint16_t address, uint8_t* data,uint8_t len);
void socket_register(uint8_t* buff, uint16_t address,uint8_t bsb, uint8_t* data,uint8_t len);



#endif /* INC_ETHERNET_H_ */
