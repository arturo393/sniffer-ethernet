/*
 * rx_ethernet_master.c
 *
 *  Created on: Feb 1, 2024
 *      Author: artur
 */

#include "rx_ethernet_master.h"

void process_lora_rx(SX1278_t *l) {

	uint8_t flags = 0;

	flags = readRegister(l->spi,LR_RegIrqFlags);

	if( !(flags & RX_DONE_MASK) )
		return;


	if (read_rx_fifo(l) <= 0) {
		clear_irq_flags_and_reset_rx_data(l);
		return;
	}



	if (validate(l->rxData, l->rxSize) != DATA_OK) {
		clear_irq_flags_and_reset_rx_data(l);
		setRxFifoAddr(l);
		configureLoRaRx(l, MASTER_RECEIVER);
		return;
	}

	LORA_RX_LED_ON();

	send_valid_data_to_uart(l->rxData, l->rxSize);
	clear_irq_flags_and_reset_rx_data(l);
	setRxFifoAddr(l);
	configureLoRaRx(l, MASTER_RECEIVER);

	LORA_RX_LED_OFF();
}


void send_valid_data_to_uart(uint8_t *buffer, uint8_t length) {
	uint8_t error[] = { 0xff, 0xff, 0xff, 0xff };
	switch (buffer[CMD_INDEX]) {
	case NONE:
	case QUERY_MODULE_ID:
	case QUERY_STATUS:
	case SET_VLAD_ATTENUATION:
	case QUERY_MASTER_STATUS:
	case QUERY_TX_FREQ:
	case QUERY_RX_FREQ:
	case QUERY_UART_BAUDRATE:
	case QUERY_BANDWIDTH:
	case QUERY_SPREAD_FACTOR:
	case QUERY_CODING_RATE:
	case SET_MODULE_ID:
	case SET_TX_FREQ:
	case SET_RX_FREQ:
	case SET_UART_BAUDRATE:
	case SET_BANDWIDTH:
	case SET_SPREAD_FACTOR:
	case SET_CODING_RATE:
	case SET_AOUT_0_10V:
	case SET_AOUT_4_20mA:
	case SET_AOUT_0_20mA:
	case SET_DOUT1:
	case SET_OUT:
	case SET_VLAD_MODE:
	case SET_PARAMETER_FREQOUT:
	case SET_PARAMETERS:
	case SET_PARAMETER_FREQBASE:
	case QUERY_PARAMETER_PdBm:
	case QUERY_UART1:
		for (uint8_t i = 0; i < length; i++)
			writeTxReg(buffer[i]);
		break;

	case QUERY_ETH:
		for (uint8_t i = 0; i < (length); i++)
			writeTxReg_uart2(buffer[i]);
		break;

	default:
		for (uint8_t i = 0; i < sizeof(error); i++)
			writeTxReg(error[i]);
		break;
	}
}

