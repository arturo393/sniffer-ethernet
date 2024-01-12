/*
 * ethernet.c
 *
 *  Created on: Jan 8, 2024
 *      Author: A.R.T.
 */
#include "ethernet.h"

void transmitir_spi(uint8_t *p, uint8_t len) { //Solo Transmite para modificar o acceder o escribir a cierto registro.
	HAL_StatusTypeDef res;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // pull the pin low
	res = HAL_SPI_Transmit(&hspi1, p, len, 1000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // pull the pin high
	if (res != HAL_OK)
		Error_Handler();
	HAL_Delay(10);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void transmitir_recibir_spi(uint8_t *p_t, uint8_t len_t, uint8_t *p_r,
		uint16_t len_r) { //manda un comando (Transmite) para recibir data que envía el chip w5500
	HAL_StatusTypeDef res;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // pull the pin low
	res = HAL_SPI_Transmit(&hspi1, p_t, len_t, 1000);
	HAL_SPI_Receive(&hspi1, p_r, len_r, 1000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // pull the pin high
	if (res != HAL_OK)
		Error_Handler();
	HAL_Delay(10);

}

/////////////////////////////////////////////////////////////////////////////////////////////////
void common_register_block(uint8_t *buff, uint16_t address, uint8_t *data,
		uint8_t len) { // Configuración de los registros comunes: encargados de la dirección IP y dirección MAC
	uint16_t os_address;
	uint8_t bsb;
	uint8_t rwb;
	uint8_t om;
	uint8_t c_phase;
	bsb = 0x00;
	rwb = 0x01 << 2; // write
	om = 00; // VDM
	c_phase = bsb | rwb | om;
	os_address = address;
	os_address = os_address << 8;
	p = buff;
	memcpy(p, &os_address, 2);
	p += 2;
	memcpy(p, &c_phase, 1);
	p += 1;
	for (int i = 0; i < len; i++) {
		memcpy(p, &data[i], 1);
		p += 1;
	}
	transmitir_spi(buff, (3 + len));
}
//////////////////////////////////////////////////////////////////////////////////////////////////

void socket_write_register(uint8_t *buff, uint16_t address, uint8_t bsb,
		uint8_t *data, uint16_t len) { // Los registros de socket brindan la comunicación del canal. Con BSB[4:0] se puede seleccionar el socket a utilizar
	uint16_t os_address;
	uint8_t rwb;
	uint8_t om;
	uint8_t c_phase;
	uint8_t t = 3 + len;
	bsb = bsb << 3;
	rwb = 0x01 << 2; // write
	om = 00; // VDM
	c_phase = bsb | rwb | om;
	//os_address_1 = address << 8;
	os_address = (address << 8) + ((address >> 8) & 0x00FF);
	p = buff;
	memcpy(p, &os_address, 2);
	p += 2;
	memcpy(p, &c_phase, 1);
	p += 1;
	for (int i = 0; i < len; i++) {
		memcpy(p, &data[i], 1);
		p += 1;
	}
	transmitir_spi(buff, t);
}

void socket_read_register(uint8_t BSB_SELECT, uint8_t addr, uint8_t *buffer_r, uint16_t buffer_r_len) {
	uint16_t offset_address = 0; /*revisar esta inicialización*/
	uint8_t buffer_t[3];
	offset_address = addr << 8;
	uint8_t BSB = BSB_SELECT << 3; // block select bit: 0x01 SOCKET REGISTER, 0x02 SOCKET TX BUFFER, 0x03 SOCKET RX BUFFER
	uint8_t RWB = 0x00 << 2; // read
	uint8_t OM = 00; // VDM
	uint8_t control_phase = BSB | RWB | OM;
	p = buffer_t;
	p += 2;
	memcpy(p, &control_phase, 1);
	p = buffer_t;
	memcpy(p, &offset_address, 2);
	transmitir_recibir_spi(buffer_t, 3, buffer_r, buffer_r_len);
}

void common_reg_config(uint8_t buffer[243], uint8_t mode, uint8_t gar[], uint8_t sub_r[], uint8_t shar[], uint8_t sipr[]) {

	//---------------------- configuracion common register
	common_register_block(buffer, 0x00, (uint8_t*) &mode, sizeof(mode));
	common_register_block(buffer, 0x01, gar, sizeof(gar));
	common_register_block(buffer, 0x05, sub_r, sizeof(sub_r));
	common_register_block(buffer, 0x09, shar, sizeof(shar));
	common_register_block(buffer, 0x0F, sipr, sizeof(sipr));
}

void socket_reg_config(uint8_t buffer[243], uint8_t S_MR, uint8_t S_PORT[2],
		uint8_t S_DHAR[6], uint8_t S_DPORT[2], uint8_t S_MMS[2], uint8_t S_TTL,
		uint8_t S_RXBUF_SIZE, uint8_t S_TXBUF_SIZE, uint8_t S_CR_open,
		uint8_t S_CR_listen) {
	//---------------------- configuracion socket register
	socket_write_register(buffer, 0x00, 0x01, (uint8_t*) &S_MR, sizeof(S_MR));
	socket_write_register(buffer, 0x04, 0x01, (uint8_t*) S_PORT, sizeof(S_PORT));
	socket_write_register(buffer, 0x06, 0x01, (uint8_t*) &S_DHAR, sizeof(S_DHAR));
	socket_write_register(buffer, 0x10, 0x01, (uint8_t*) S_DPORT, sizeof(S_DPORT));
	socket_write_register(buffer, 0x12, 0x01, (uint8_t*) &S_MMS, sizeof(S_MMS));
	socket_write_register(buffer, 0x16, 0x01, (uint8_t*) &S_TTL, sizeof(S_TTL));
	socket_write_register(buffer, 0x1E, 0x01, (uint8_t*) &S_RXBUF_SIZE,sizeof(S_RXBUF_SIZE));
	socket_write_register(buffer, 0x1F, 0x01, (uint8_t*) &S_TXBUF_SIZE,sizeof(S_TXBUF_SIZE));
	socket_write_register(buffer, 0x01, 0x01, (uint8_t*) &S_CR_open,sizeof(S_CR_open));
	socket_write_register(buffer, 0x01, 0x01, (uint8_t*) &S_CR_listen,sizeof(S_CR_listen));
}






/*
void read_socket_interrupt(uint8_t s_IR_eth, uint8_t s_IR_eth,uint8_t s_IR_eth,uint8_t s_IR_eth){

}

*/









