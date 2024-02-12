/*
 * sniffer.c
 *
 *  Created on: Jul 20, 2023
 *      Author: artur
 */
#include "sniffer.h"

uint32_t adcValues[ADC_CHANNELS];
uint8_t adcCounter[ADC_CHANNELS] = { 0 };
uint16_t adcReadings[ADC_CHANNELS][ADC_WINDOW_SIZE] = { 0 };
uint16_t adcMA[ADC_CHANNELS];
uint32_t adcSum[ADC_CHANNELS] = { 0 };

Sniffer_t* sniffer(I2C_HandleTypeDef *i2c, uint16_t *adcBuffer) {
	Sniffer_t *s;
	s = malloc(sizeof(Sniffer_t));

	if (s != NULL) {
		s->function = SNIFFER;
		s->i2c = i2c;
		s->id = readEepromData(s, ID);
		s->function = readEepromData(s, FUNCTION);
		s->eth_lenRX = 0;


	}
	return (s);
}

void processReceivedSerialConfig(Sniffer_t *sniffer) {
	UART_t *serialConfig = sniffer->serial_lora;
	uint8_t *dataReceived = serialConfig->data;
	uint8_t *len = &(serialConfig->len);
	LORA_t *loRa = sniffer->lora;
	SX1276_HW_t *hw = loRa->rxhw;

	// Data validation
	if (*len < MINIMUM_FRAME_LEN || dataReceived[START_INDEX] != RDSS_START_MARK
			|| dataReceived[*len - 1] != RDSS_END_MARK
			|| dataReceived[MODULE_TYPE_INDEX] != SNIFFER
			|| checkCRCValidity(dataReceived, *len) != DATA_OK
			|| dataReceived[MODULE_ID_INDEX] != sniffer->id
			|| dataReceived[CMD_INDEX] == 0) {
		memset(dataReceived, 0, *len);
		return;
	}
	/*
	 *len = exec(sniffer, dataReceived);
	 // Send response via UART
	 HAL_UART_Transmit(serialConfig->handler, dataReceived, *len, 100);
	 //serialConfig->isReceivedDataReady = false;
	 memset(serialConfig->data, 0, UART_SIZE);
	 */

	*len = packet_message(sniffer, dataReceived, *len, QUERY_UART1); /// o set_uart_1 según corresponda

	// Send response via LoRa
	loRa->txData = dataReceived;
	loRa->txSize = *len;
	startTransmition(loRa);
	startRxContinuous(hw, RECEIVE_PAYLOAD_LENGTH);
	memset(loRa->rxData, 0, 300);
	memset(serialConfig->data, 0, UART_SIZE);
	loRa->rxSize = readReg(hw, LR_RegRxNbBytes);
}
/////////////////////////////////////////////////////////////////////////////////////
void processReceivedSerialLora(Sniffer_t *sniffer) {
	UART_t *serialLora = sniffer->serial_lora;
	uint8_t *dataReceived = serialLora->data;
	uint8_t *len = &(serialLora->len);
	LORA_t *loRa = sniffer->lora;
	SX1276_HW_t *hw = loRa->rxhw;

	if (*len < 2) {
		HAL_Delay(300);
		if (*len < 2) {
			memset(serialLora->data, 0, *len);
			//serialLora->isReceivedDataReady = false;
			return;
		}
	}

	//añadir filtros aca

	*len = packet_message(sniffer, dataReceived, *len, QUERY_UART1); //o set_uart_1 según corresponda

	// Send response via LoRa
	loRa->txData = dataReceived;
	loRa->txSize = *len;
	startTransmition(loRa);
	startRxContinuous(hw, RECEIVE_PAYLOAD_LENGTH);
	memset(loRa->rxData, 0, 300);
	memset(serialLora->data, 0, UART_SIZE);
	loRa->rxSize = readReg(hw, LR_RegRxNbBytes);
}

void processReceivedEthernet(Sniffer_t *sniffer) { ///////////////////////################////////////////////////////////////
	LORA_t *lora = sniffer->lora;
	SX1276_HW_t *hw = lora->rxhw;

	lora->txSize = packet_message(sniffer, sniffer->eth_bufRX,(uint16_t) sniffer->eth_lenRX, QUERY_ETH);

	lora->txData = sniffer->eth_bufRX;
	startTransmition(lora);
	//startRxContinuous(hw, RECEIVE_PAYLOAD_LENGTH);

	memset(lora->rxData, 0, 300);
	lora->rxSize = 0;


	memset(sniffer->eth_bufRX,0,sizeof(sniffer->eth_bufRX));
	sniffer->eth_lenRX = 0;

	lora->txData = NULL;

}




uint8_t dataValidation(uint8_t *len, uint8_t *dataReceived, Sniffer_t *sniffer) {
	// Data validation
	if (*len < MINIMUM_FRAME_LEN || dataReceived[START_INDEX] != RDSS_START_MARK
			|| dataReceived[*len - 1] != RDSS_END_MARK
			|| dataReceived[MODULE_TYPE_INDEX] != SNIFFER
			|| checkCRCValidity(dataReceived, *len) != DATA_OK
			|| dataReceived[MODULE_ID_INDEX] != sniffer->id
			|| dataReceived[CMD_INDEX] == 0) {
		return 0;
	}
	return 1;
}

void processReceivedLoRa(Sniffer_t *sniffer) {
	LORA_t *loRa = sniffer->lora;
	uint8_t *dataReceived = loRa->rxData;
	uint8_t *len = &(loRa->rxSize);
	SX1276_HW_t *hw = loRa->rxhw;

	UART_t *serialLora = sniffer->serial_lora;
	//uint8_t zero = 0x00;                                          //////////////REVISAR///////////////
	//uint8_t base = readReg(hw, LR_RegFifoRxCurrentaddr);

	if (loRa->rxSize > 0) {
		if (loRa->rxData == NULL) {
			return;
		}
		memset(loRa->rxData, 0, sizeof(uint8_t) * loRa->rxSize);
		uint8_t addr = 0x00;
		HAL_GPIO_WritePin(hw->nssPort, hw->nssPin, GPIO_PIN_RESET); // pull the pin low
		HAL_Delay(1);
		HAL_SPI_Transmit(hw->spi, &addr, 1, 100); // send address
		HAL_SPI_Receive(hw->spi, loRa->rxData, loRa->rxSize, 100); // receive 6 bytes data
		HAL_Delay(1);
		HAL_GPIO_WritePin(hw->nssPort, hw->nssPin, GPIO_PIN_SET); // pull the pin high
	}
	// Data validation
	if (dataValidation(len, dataReceived, sniffer) == 0) {
		memset(loRa->rxData, 0, 300);
		*len = 0;
		startRxContinuous(hw, RECEIVE_PAYLOAD_LENGTH);
		return;
	}
	*len = message_package(sniffer, dataReceived);
	if (dataReceived[CMD_INDEX] == QUERY_UART1) {
		//Retransmit  via serial
		HAL_UART_Transmit(serialLora->handler, dataReceived + DATA_START_INDEX,
				*len, 100);
		memset(serialLora->data, 0, UART_SIZE);
	}
	if (dataReceived[CMD_INDEX] == QUERY_ETH) {
		uint8_t size = dataReceived[DATA_LENGHT1_INDEX];
		uint8_t data_enviar[size];
		for (uint8_t i = 0; i < size; i++)
			data_enviar[i] = (uint8_t) dataReceived[DATA_START_INDEX + i];

		eth_transmit(socket_0_register, data_enviar, size);

	} else {
		// Send response via LoRa
		uint8_t size = dataReceived[DATA_LENGHT1_INDEX];
		uint8_t data_enviar[size];
		for (uint8_t i = 0; i < size; i++) {
			data_enviar[i] = (uint8_t) dataReceived[DATA_START_INDEX + i];
		}
		loRa->txData = loRa->rxData;
		loRa->txSize = sizeof(data_enviar) + 9;
		startTransmition(loRa);
		startRxContinuous(hw, RECEIVE_PAYLOAD_LENGTH);
		memset(loRa->rxData, 0, 300);
		loRa->rxSize = 0;

	}
}

void processReceived(Sniffer_t *sniffer) {
	UART_t *serial = sniffer->serial_lora;
	LORA_t *loRa = sniffer->lora;
	if (serial->isReceivedDataReady) {
		HAL_Delay(300);
		if (dataValidation(&(serial->len), serial->data, sniffer) == 1)
			processReceivedSerialConfig(sniffer);
		else
			processReceivedSerialLora(sniffer);
		serial->len = 0;
		serial->isReceivedDataReady = false;
		HAL_UART_AbortReceive_IT(sniffer->serial_lora->handler);
		HAL_UART_Receive_IT(sniffer->serial_lora->handler,
				sniffer->serial_lora->data + sniffer->serial_lora->len, 1);
	} else if (sniffer->eth_lenRX > 0)
		processReceivedEthernet(sniffer);
	else if (loRa->rxSize > 0)
		processReceivedLoRa(sniffer);
}

uint8_t packet_message(Sniffer_t *s, uint8_t *dataReceived, uint16_t dataLen,
		Rs485_cmd_t cmd_rq) {
	uint8_t *responsePtr;
	uint8_t zero[] = { 0x00 };
	uint8_t sniffer_func[] = { SNIFFER };
	uint8_t cmd[] = { cmd_rq }; /////////////$$$$$$$$$$$$$$////////////
	uint8_t start_mark[] = { 0x7e };

	if (dataReceived == NULL)
		return 0;

	uint8_t temp[dataLen];
	uint8_t i = 0;
	while (i < dataLen) {
		temp[i] = dataReceived[START_INDEX + i];
		i++;
	}

	responsePtr = &dataReceived[START_INDEX];
	memcpy(responsePtr, start_mark, sizeof(start_mark));
	responsePtr += sizeof(start_mark);
	memcpy(responsePtr, sniffer_func, sizeof(sniffer_func));
	responsePtr += sizeof(sniffer_func);
	memcpy(responsePtr, &(s->id), sizeof(s->id));
	responsePtr += sizeof(s->id);
	memcpy(responsePtr, cmd, sizeof(cmd));
	responsePtr += sizeof(cmd);

	responsePtr = &dataReceived[DATA_LENGHT1_INDEX];
	memcpy(responsePtr, &dataLen, sizeof(dataLen));
	responsePtr = &dataReceived[DATA_LENGHT2_INDEX];
	memcpy(responsePtr, zero, sizeof(zero));
	responsePtr = &dataReceived[DATA_START_INDEX];
	//copiar mensaje que llego originalmente
	i = 0;
	while (i < dataLen) {
		memcpy(responsePtr, &temp[i], sizeof(temp[i]));
		responsePtr += sizeof(temp[i]);
		i++;
	}

	memset(temp, 0, sizeof(temp));
	uint8_t CRC_INDEX = DATA_START_INDEX + dataLen;
	uint8_t END_INDEX = CRC_INDEX + CRC_SIZE;
	memcpy(dataReceived + DATA_LENGHT1_INDEX, &dataLen, sizeof(dataLen));
	responsePtr += sizeof(dataLen);
	dataReceived[END_INDEX] = LTEL_END_MARK;
	setCrc(dataReceived, DATA_START_INDEX + dataLen);

	return (END_INDEX + 1);
}

uint8_t message_package(Sniffer_t *s, uint8_t *dataReceived) {

	uint16_t dataLen = 0;
	LORA_t *loRa = s->lora;
	uint8_t *response = dataReceived;
	uint8_t *responsePtr;
	switch (response[CMD_INDEX]) {
	case QUERY_RX_FREQ:
		dataLen = sizeof(loRa->dfreq);
		freqEncode(dataReceived + DATA_START_INDEX, loRa->dfreq);
		break;
	case QUERY_TX_FREQ:
		dataLen = sizeof(loRa->ufreq);
		freqEncode(dataReceived + DATA_START_INDEX, loRa->ufreq);
		break;
	case QUERY_SPREAD_FACTOR:
		dataLen = sizeof(loRa->sf);
		dataReceived[DATA_START_INDEX] = loRa->sf - 6;
		break;
	case QUERY_CODING_RATE:
		dataLen = sizeof(loRa->cr);
		dataReceived[DATA_START_INDEX] = loRa->cr;
		break;
	case QUERY_BANDWIDTH:
		dataLen = sizeof(loRa->bw);
		dataReceived[DATA_START_INDEX] = loRa->bw + 1;
		break;
	case QUERY_MODULE_ID:
		dataLen = sizeof(s->function) + sizeof(s->id);
		responsePtr = &dataReceived[DATA_START_INDEX];
		memcpy(responsePtr, &s->function, sizeof(s->function));
		responsePtr += sizeof(s->function);
		memcpy(responsePtr, &s->id, sizeof(s->id));
		break;
	case QUERY_STATUS:
		responsePtr = &dataReceived[DATA_START_INDEX];
		responsePtr = dataReceived + DATA_START_INDEX;
		/*swOut_x_20_mA,
		 swIn_x_20mA,
		 dIn1,
		 dIn2,
		 aOut1_10V,
		 aOut_x_20mA,
		 aIn1_10V,
		 aIn_x_20mA*/
		dataLen = sizeof(s->aIn1_0_10V) + sizeof(s->aOut1_0_10V)
				+ sizeof(s->aIn2_x_20mA) + sizeof(s->aOut2_x_20mA)
				+ sizeof(s->swOut_x_20mA) + sizeof(s->swIn_x_20mA)
				+ sizeof(s->dIn1) + sizeof(s->dIn2) + sizeof(s->swSerial);

		memcpy(responsePtr, &(s->aIn1_0_10V), sizeof(s->aIn1_0_10V));
		responsePtr += sizeof(s->aIn1_0_10V);

		memcpy(responsePtr, &(s->aOut1_0_10V), sizeof(s->aOut1_0_10V));
		responsePtr += sizeof(s->aOut1_0_10V);

		memcpy(responsePtr, &(s->aIn2_x_20mA), sizeof(s->aIn2_x_20mA));
		responsePtr += sizeof(s->aIn2_x_20mA);

		memcpy(responsePtr, &(s->aOut2_x_20mA), sizeof(s->aOut2_x_20mA));
		responsePtr += sizeof(s->aOut2_x_20mA);

		memcpy(responsePtr, &(s->swIn_x_20mA), sizeof(s->swIn_x_20mA));
		responsePtr += sizeof(s->swIn_x_20mA);

		memcpy(responsePtr, &(s->swOut_x_20mA), sizeof(s->swOut_x_20mA));
		responsePtr += sizeof(s->swOut_x_20mA);

		memcpy(responsePtr, &(s->dIn1), sizeof(s->dIn1));
		responsePtr += sizeof(s->dIn1);

		memcpy(responsePtr, &(s->dIn2), sizeof(s->dIn2));
		responsePtr += sizeof(s->dIn2);

		memcpy(responsePtr, &(s->swSerial), sizeof(s->swSerial));
		responsePtr += sizeof(s->swSerial);

		// Set the value of dIn in the response buffer
		//*responsePtr = s->dIn1;
		//responsePtr += 3;
		break;
	case SET_MODULE_ID:
		dataLen = sizeof(s->function) + sizeof(s->id);
		s->function = dataReceived[DATA_START_INDEX];
		s->id = dataReceived[DATA_START_INDEX + sizeof(s->id)];
		saveData(s, ID);
		saveData(s, FUNCTION);
		break;
	case SET_TX_FREQ:
		dataLen = sizeof(loRa->ufreq);
		loRa->ufreq = freqDecode(dataReceived + DATA_START_INDEX);
		saveData(s, UPLINK);
		setFreqReg(loRa->txhw, loRa->ufreq);
		break;
	case SET_RX_FREQ:
		dataLen = sizeof(loRa->dfreq);
		loRa->dfreq = freqDecode(dataReceived + DATA_START_INDEX);
		saveData(s, DOWNLINK);
		setFreqReg(loRa->rxhw, loRa->dfreq);
		startRxContinuous(loRa->rxhw, RECEIVE_PAYLOAD_LENGTH);
		break;
	case SET_BANDWIDTH:
		dataLen = sizeof(loRa->bw);
		loRa->bw = dataReceived[DATA_START_INDEX] - 1;
		saveData(s, BANDWITH);
		writeLoRaParams(loRa);
		startRxContinuous(loRa->rxhw, RECEIVE_PAYLOAD_LENGTH);
		break;
	case SET_SPREAD_FACTOR:
		dataLen = sizeof(loRa->sf);
		loRa->sf = dataReceived[DATA_START_INDEX] + 6;
		saveData(s, SPREAD_FACTOR);
		writeLoRaParams(loRa);
		startRxContinuous(loRa->rxhw, RECEIVE_PAYLOAD_LENGTH);
		break;
	case SET_CODING_RATE:
		dataLen = sizeof(loRa->cr);
		loRa->cr = dataReceived[DATA_START_INDEX];
		saveData(s, CODING_RATE);
		writeLoRaParams(loRa);
		startRxContinuous(loRa->rxhw, RECEIVE_PAYLOAD_LENGTH);
		break;
	case SET_OUT:
		dataLen = sizeof(s->aOut1_0_10V) + sizeof(s->aOut2_x_20mA)
				+ sizeof(s->dOut) + sizeof(s->dOut2);
		s->aOut1_0_10V = dataReceived[DATA_START_INDEX + 0]
				| (dataReceived[DATA_START_INDEX + 1] << 8);
		s->aOut2_x_20mA = dataReceived[DATA_START_INDEX + 2]
				| (dataReceived[DATA_START_INDEX + 3] << 8);
		s->dOut = dataReceived[DATA_START_INDEX + 4];

		s->dOut2 = dataReceived[DATA_START_INDEX + 5];

		s->swSerial = dataReceived[DATA_START_INDEX + 6];
		//	saveData(s, AOUT_0_10V);
		//	saveData(s, AOUT_0_20mA);
		//	saveData(s, AOUT_4_20mA);
		//	saveData(s, DOUT);
		break;
	case SET_AOUT_0_10V:
		dataLen = sizeof(s->aOut1_0_10V);
		s->aOut1_0_10V = dataReceived[DATA_START_INDEX]
				| (dataReceived[DATA_START_INDEX + 1] << 8);
		saveData(s, AOUT_0_10V);
		break;
	case SET_AOUT_4_20mA:
		dataLen = sizeof(s->aOut2_x_20mA);
		s->aOut2_x_20mA = dataReceived[DATA_START_INDEX]
				| (dataReceived[DATA_START_INDEX + 1] << 8);
		saveData(s, AOUT_4_20mA);
		break;
	case SET_AOUT_0_20mA:
		dataLen = sizeof(s->aOut3);
		s->aOut3 = dataReceived[DATA_START_INDEX]
				| (dataReceived[DATA_START_INDEX + 1] << 8);
		saveData(s, AOUT_0_20mA);
		break;
	case SET_DOUT1:
		dataLen = sizeof(s->dIn1);
		s->dIn1 = dataReceived[DATA_START_INDEX];
		saveData(s, DOUT);
		break;
	case QUERY_UART1:
		dataLen = (dataReceived[DATA_LENGHT1_INDEX]
				| dataReceived[DATA_LENGHT2_INDEX] << 8);
		return (dataLen);
		break;
	case SET_UART1:
		//aca va algo del set
		break;
	default:
		// Comando no reconocido, responder con un mensaje de error o realizar otra acción
		return (-1);
	}

	uint8_t CRC_INDEX = DATA_START_INDEX + dataLen;
	uint8_t END_INDEX = CRC_INDEX + CRC_SIZE;
	memcpy(dataReceived + DATA_LENGHT1_INDEX, &dataLen, sizeof(dataLen));
	responsePtr += sizeof(dataLen);
	dataReceived[END_INDEX] = LTEL_END_MARK;
	setCrc(dataReceived, DATA_START_INDEX + dataLen);

	return (END_INDEX + 1);
}

uint8_t readEepromData(Sniffer_t *s, EEPROM_SECTOR_t sector) {
	LORA_t *loRa = s->lora;
	I2C_HandleTypeDef *i2c = s->i2c;

	uint8_t result = 0; // Initialize result to 0 (success).

	switch (sector) {
	case SPREAD_FACTOR:
		loRa->sf = readByte(i2c, K24C02_PAGE_ADDR(0), SF_OFFSET);
		if (loRa->sf < SF_6 || loRa->sf > SF_12) {
			loRa->sf = SF_10;
			result = 1; // Set result to 1 (error).
		}
		break;
	case BANDWITH:
		loRa->bw = readByte(i2c, K24C02_PAGE_ADDR(0), BW_OFFSET);
		if (loRa->bw < BW_7_8KHZ || loRa->bw > BW_500KHZ) {
			loRa->bw = BW_62_5KHZ;
			result = 1; // Set result to 1 (error).
		}
		break;
	case CODING_RATE:
		loRa->cr = readByte(i2c, K24C02_PAGE_ADDR(0), CR_OFFSET);
		if (loRa->cr < CR_4_5 || loRa->cr > CR_4_8) {
			loRa->cr = CR_4_6;
			result = 1; // Set result to 1 (error).
		}
		break;
	case FUNCTION:
		s->function = readByte(i2c, K24C02_PAGE_ADDR(0), FUNCTION_OFFSET);
		if (s->function == -1) {
			s->function = VLADR;
			result = 1; // Set result to 1 (error).
		}
		break;
	case ID:
		s->id = readByte(i2c, K24C02_PAGE_ADDR(0), ID_OFFSET);
		if (s->id == -1) {
			s->id = 0;
			result = 1; // Set result to 1 (error).
		}
		break;
	case DOWNLINK:
		loRa->dfreq = read4Byte(i2c, K24C02_PAGE_ADDR(1), DL_OFFSET);
		if (loRa->dfreq < DOWNLINK_FREQ_MIN || loRa->dfreq > DOWNLINK_FREQ_MAX) {
			loRa->dfreq = DOWNLINK_FREQ;
			result = 1; // Set result to 1 (error).
		}
		break;
	case UPLINK:
		loRa->ufreq = read4Byte(i2c, K24C02_PAGE_ADDR(1), UL_OFFSET);
		if (loRa->ufreq < UPLINK_FREQ_MIN || loRa->ufreq > UPLINK_FREQ_MAX) {
			loRa->ufreq = UPLINK_FREQ;
			result = 1; // Set result to 1 (error).
		}
		break;
	case AOUT_0_10V:
		s->aOut1_0_10V = read2Byte(i2c, K24C02_PAGE_ADDR(2), AOUT_0_10V_OFFSET);
		if (s->aOut1_0_10V == -1) {
			s->aOut1_0_10V = 0;
			result = 1; // Set result to 1 (error).
		}
		break;
	case AOUT_0_20mA:
		s->aOut2_x_20mA = read2Byte(i2c, K24C02_PAGE_ADDR(2),
		AOUT_0_20mA_OFFSET);
		if (s->aOut2_x_20mA == -1) {
			s->aOut2_x_20mA = 0;
			result = 1; // Set result to 1 (error).
		}
		break;
	case AOUT_4_20mA:
		s->aOut2_x_20mA = read2Byte(i2c, K24C02_PAGE_ADDR(2),
		AOUT_4_20mA_OFFSET);
		if (s->aOut2_x_20mA == -1) {
			s->aOut2_x_20mA = 0;
			result = 1; // Set result to 1 (error).
		}
		break;
	case DOUT:
		s->dOut = readByte(i2c, K24C02_PAGE_ADDR(2), DOUT_OFFSET);
		if (s->dOut == -1) {
			s->dOut = GPIO_PIN_RESET;
			result = 1; // Set result to 1 (error).
		}
		break;
	default:
		result = 2; // Set result to 2 (unsupported sector).
		break;
	}

	return (result);
}

HAL_StatusTypeDef saveData(Sniffer_t *s, EEPROM_SECTOR_t sector) {
	SX1278_t *loRa = s->loRa;
	I2C_HandleTypeDef *i2c = s->i2c;
	uint32_t page = 0;
	uint8_t *data;
	uint8_t dataLen = 0;
	HAL_StatusTypeDef res;
	uint8_t offset = 0;
	switch (sector) {
	case SPREAD_FACTOR:
		page = K24C02_PAGE_ADDR(0);
		offset = SF_OFFSET;
		if (loRa->sf < SF_6 || loRa->sf > SF_12)
			loRa->sf = SF_10;
		data = (uint8_t*) &(loRa->sf);
		dataLen = sizeof(loRa->sf);
		break;
	case BANDWITH:
		page = K24C02_PAGE_ADDR(0);
		offset = BW_OFFSET;
		if (loRa->bw < BW_7_8KHZ || loRa->bw > BW_500KHZ)
			loRa->bw = BW_62_5KHZ;
		data = (uint8_t*) &(loRa->bw);
		dataLen = sizeof(loRa->bw);
		break;
	case CODING_RATE:
		page = K24C02_PAGE_ADDR(0);
		offset = CR_OFFSET;
		if (loRa->cr < CR_4_5 || loRa->cr > CR_4_8)
			loRa->cr = CR_4_6;
		data = (uint8_t*) &loRa->cr;
		dataLen = sizeof(loRa->cr);
		break;
	case FUNCTION:
		page = K24C02_PAGE_ADDR(0);
		offset = FUNCTION_OFFSET;
		data = &(s->function);
		dataLen = sizeof(s->function);
		break;
	case ID:
		page = K24C02_PAGE_ADDR(0);
		offset = ID_OFFSET;
		data = &(loRa->id);
		dataLen = sizeof(loRa->id);
		break;
	case DOWNLINK:
		page = K24C02_PAGE_ADDR(1);
		offset = DL_OFFSET;
		if (loRa->dlFreq < DOWNLINK_FREQ_MIN || loRa->dlFreq > DOWNLINK_FREQ_MAX)
			loRa->dlFreq = DOWNLINK_FREQ;

		data = (uint8_t*) &(loRa->dlFreq);
		dataLen = sizeof(loRa->dlFreq);
		break;
	case UPLINK:
		page = K24C02_PAGE_ADDR(1);
		offset = UL_OFFSET;
		if (loRa->ulFreq < UPLINK_FREQ_MIN || loRa->ulFreq > UPLINK_FREQ_MAX)
			loRa->ulFreq = UPLINK_FREQ;
		data = (uint8_t*) &(loRa->ulFreq);
		dataLen = sizeof(loRa->ulFreq);
		break;
	case AOUT_0_10V:
		page = K24C02_PAGE_ADDR(2);
		offset = AOUT_0_10V_OFFSET;
		data = (uint8_t*) &(s->aOut1_0_10V);
		dataLen = sizeof(s->aOut1_0_10V);
		break;
	case AOUT_0_20mA:
		page = K24C02_PAGE_ADDR(2);
		offset = AOUT_0_20mA_OFFSET;
		data = (uint8_t*) &(s->aOut2_x_20mA);
		dataLen = sizeof(s->aOut2_x_20mA);
		break;
	case AOUT_4_20mA:
		page = K24C02_PAGE_ADDR(2);
		offset = AOUT_4_20mA_OFFSET;
		data = (uint8_t*) &(s->aOut3);
		dataLen = sizeof(s->aOut3);
		break;
	case DOUT:
		page = K24C02_PAGE_ADDR(2);
		offset = AOUT_4_20mA_OFFSET;
		data = &(s->dOut);
		dataLen = sizeof(s->dOut);
		break;
	default:
		return (HAL_ERROR);
		break;
	}

	res = savePage(i2c, page, data, offset, dataLen);
	return (res);

}

