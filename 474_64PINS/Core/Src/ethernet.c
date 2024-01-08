/*
 * ethernet.c
 *
 *  Created on: Jan 8, 2024
 *      Author: A.R.T.
 */
#include "ethernet.h"



 void transmitir_spi(uint8_t *p, uint8_t len){ //Solo Transmisión
	HAL_StatusTypeDef res;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // pull the pin low
	res = HAL_SPI_Transmit(&hspi1, p, len, 1000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // pull the pin high
	if (res != HAL_OK)
		Error_Handler();
	HAL_Delay(10);
}

 /////////////////////////////////////////////////////////////////////////////////////////////////
 void transmitir_recibir_spi(uint8_t *p_t, uint8_t len_t, uint8_t *p_r, uint16_t len_r){ //Recepción *y transmisión adicional para verificar comunicación desde el peer*
 	HAL_StatusTypeDef res;
 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // pull the pin low
 	res = HAL_SPI_Transmit(&hspi1, p_t, len_t, 1000);
 	HAL_SPI_Receive(&hspi1, p_r, len_r, 1000);
 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // pull the pin high
 	if (res != HAL_OK)
 		Error_Handler();
 	else
 		data_ethernet == HAL_OK; /////////#####///////
 	HAL_Delay(10);

 }

/////////////////////////////////////////////////////////////////////////////////////////////////
 void common_register_block(uint8_t *buff, uint16_t address, uint8_t *data,uint8_t len){ // Configuración de los registros comunes: encargados de la dirección IP y dirección MAC
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
 	p +=1;
 	for(int i=0;i<len;i++){
 		memcpy(p,&data[i],1);
 		p +=1;
 	}
 	transmitir_spi(buff, (3+len));
 }
 //////////////////////////////////////////////////////////////////////////////////////////////////

 void socket_register(uint8_t *buff, uint16_t address, uint8_t bsb, uint8_t *data, uint8_t len){ // Los registros de socket brindan la comunicación del canal. Con BSB[4:0] se puede seleccionar el socket a utilizar
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
 	os_address = (address << 8) + ((address>>8) & 0x00FF);
 	p = buff;
 	memcpy(p, &os_address, 2);
 	p += 2;
 	memcpy(p, &c_phase, 1);
 	p +=1;
 	for(int i=0;i<len;i++){
 		memcpy(p,&data[i],1);
 		p +=1;
 	}
 	transmitir_spi(buff, t);
 }
