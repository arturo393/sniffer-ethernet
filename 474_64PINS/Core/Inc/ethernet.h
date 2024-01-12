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

//Selección de socket registers//

#define s_IR_REG 0x02
#define s_TX_RD_REG 0x22
#define s_TX_WR_REG 0x24
#define s_RX_RS_REG 0x26
#define s_RX_RD_REG 0x28
#define s_RX_WR_REG 0x2A
#define s_SR_REG 0x03
#define s_CR_REG 0x01
#define s_PORT_REG 0x04
#define s_DHAR_REG 0x06
#define s_DIPR_REG 0x0C
#define s_DPORT_REG 0x10
#define s_MSS_REG 0x12
#define s_TOS_REG 0x15
#define s_TTL_REG 0x16
#define s_RXBUF_SIZE_REG 0x1E
#define s_TXBUF_SIZE_REG 0x1F
#define s_TX_FS_REG 0x20
#define s_IMR_REG 0x2C
#define s_FRAG_REG 0x2D
#define s_KPALVTR_REG 0x2F

// PARA SELECCIÓN DE BLOQUES DE REGISTROS DE SOCKET//
#define socket_0_register 0x01
#define socket_1_register 0x05
#define socket_2_register 0x09
#define socket_3_register 0x0D
#define socket_4_register 0x11
#define socket_5_register 0x15
#define socket_6_register 0x19
#define socket_7_register 0x1D

//PARA SELECCIÓN DE BUFFER DE TX DE SOCKET//
#define socket_0_tx_buffer 0x02
#define socket_1_tx_buffer 0x06
#define socket_2_tx_buffer 0x0A
#define socket_3_tx_buffer 0x0E
#define socket_4_tx_buffer 0x12
#define socket_5_tx_buffer 0x16
#define socket_6_tx_buffer 0x1A
#define socket_7_tx_buffer 0x1E

//PARA SELECCIÓN DE BUFFER DE RX DE SOCKET//
#define socket_0_rx_buffer 0x03
#define socket_1_rx_buffer 0x07
#define socket_2_rx_buffer 0x0B
#define socket_3_rx_buffer 0x0F
#define socket_4_rx_buffer 0x13
#define socket_5_rx_buffer 0x17
#define socket_6_rx_buffer 0x1B
#define socket_7_rx_buffer 0x1F


// Funciones de transmisión recepción vía SPI:
void transmitir_spi(uint8_t* p, uint8_t len);
void transmitir_recibir_spi(uint8_t* p_t, uint8_t len_t, uint8_t* p_r, uint16_t len_r);
void socket_read_register(uint8_t BSB_SELECT,uint8_t addr, uint8_t *buffer_r, uint16_t buffer_r_len);

// Funciones para el ajuste de los registros del chip w5500:
void common_register_block(uint8_t* buff, uint16_t address, uint8_t* data,uint8_t len);
void socket_write_register(uint8_t* buff, uint16_t address,uint8_t bsb, uint8_t* data,uint16_t len);

void common_reg_config(uint8_t buffer[243], uint8_t mode, uint8_t gar[],uint8_t sub_r[], uint8_t shar[], uint8_t sipr[]);
void socket_reg_config(uint8_t buffer[243], uint8_t S_MR, uint8_t S_PORT[2],
		uint8_t S_DHAR[6], uint8_t S_DPORT[2], uint8_t S_MMS[2], uint8_t S_TTL,
		uint8_t S_RXBUF_SIZE, uint8_t S_TXBUF_SIZE, uint8_t S_CR_open,
		uint8_t S_CR_listen);


#endif /* INC_ETHERNET_H_ */
