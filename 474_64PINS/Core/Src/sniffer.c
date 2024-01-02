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
		s->aOut1 = readEepromData(s, AOUT_0_10V);
		s->aOut2 = readEepromData(s, AOUT_0_20mA);
		s->aOut3 = readEepromData(s, AOUT_4_20mA);
		s->dOut = readEepromData(s, DOUT);
		s->aIn1 = adcBuffer + VOLTAGE0_10V_CH;
		s->aIn2 = adcBuffer + SENSOR0_20mA_CH;
		s->aIn2 = adcBuffer + SENSOR4_20mACH;
	}
	return (s);
}

void processReceivedSerial(Sniffer_t *sniffer) {
	UART_t *serial = sniffer->serial;
	SX1278_t *loRaRx = sniffer->loRaRx;
	uint8_t *dataReceived = serial->data;
	uint8_t *len = &(serial->len);

	// Data validation
	if (*len < MINIMUM_FRAME_LEN || dataReceived[START_INDEX] != RDSS_START_MARK
			|| dataReceived[*len - 1] != RDSS_END_MARK
			|| dataReceived[MODULE_TYPE_INDEX] != VLADR
			|| checkCRCValidity(dataReceived, *len) != DATA_OK
			|| dataReceived[MODULE_ID_INDEX] != sniffer->loRa->id
			|| dataReceived[CMD_INDEX] == 0) {
		memset(dataReceived, 0, *len);
		*len = 0;
		loRaRx->isReceivedDataReady = false;
		serial->isReceivedDataReady = false;
		return;
	}

	*len = exec(sniffer, dataReceived);
	// Send response via UART
	HAL_UART_Transmit(serial->handler, dataReceived, *len, 100);
	serial->isReceivedDataReady = false;
}

void processReceivedLoRa(Sniffer_t *sniffer) {
	LORA_t *loRa = sniffer->lora;
	uint8_t *dataReceived = loRa->rxData;
	uint8_t *len = &(loRa->rxSize);

	// Data validation
	if (*len < MINIMUM_FRAME_LEN || dataReceived[START_INDEX] != RDSS_START_MARK
			|| dataReceived[*len - 1] != RDSS_END_MARK
			|| dataReceived[MODULE_TYPE_INDEX] != VLADR
			|| checkCRCValidity(dataReceived, *len) != DATA_OK
			|| dataReceived[MODULE_ID_INDEX] != 0x08 //sniffer->loRa->id
			|| dataReceived[CMD_INDEX] == 0) {
		memset(loRa->rxData, 0, sizeof(loRa->rxData));
		*len = 0;
		return;
	}

	*len = exec(sniffer, dataReceived);

	// Send response via LoRa
	loRa->txData = dataReceived;
	loRa->txSize = *len;
	HAL_Delay(60);
	startTransmition(loRa);
	//startTransmition(loRa);
	*len = 0;
}

void processReceived(Sniffer_t *sniffer) {
	UART_t *serial = sniffer->serial;
	LORA_t *loRa = sniffer->lora;

	if (serial->isReceivedDataReady) {
		processReceivedSerial(sniffer);
	} else if (loRa->rxSize > 0) {
		processReceivedLoRa(sniffer);
	}
}

uint8_t exec(Sniffer_t *s, uint8_t *dataReceived) {

	uint8_t dataLen = 0;
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
		dataLen = sizeof(s->aIn1) + sizeof(s->aIn2) + sizeof(s->dOut_switch)
				+sizeof(s->dIn_switch)+ sizeof(s->dIn)+sizeof(s->dIn2)+4;
		uint8_t c= 0xAA;
		memcpy(responsePtr, &c, 1);
				responsePtr += 1;
		memcpy(responsePtr, &(s->aIn1), sizeof(s->aIn1));
		responsePtr += sizeof(s->aIn1);

		memcpy(responsePtr, &(s->dOut_switch), sizeof(s->dOut_switch));
		responsePtr += sizeof(s->dOut_switch);

		memcpy(responsePtr, &(s->aIn2), sizeof(s->aIn2));
		responsePtr += sizeof(s->aIn2);

		memcpy(responsePtr, &(s->dIn_switch), sizeof(s->dIn_switch));
		responsePtr += sizeof(s->dIn_switch);
		memcpy(responsePtr, &(s->dIn2), sizeof(s->dIn2));
		responsePtr += sizeof(s->dIn2);
		// Set the value of dIn in the response buffer
		*responsePtr = s->dIn;
		responsePtr += 3;
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
		startRxContinuous(loRa->rxhw,RECEIVE_PAYLOAD_LENGTH);
		break;
	case SET_BANDWIDTH:
		dataLen = sizeof(loRa->bw);
		loRa->bw = dataReceived[DATA_START_INDEX] - 1;
		saveData(s, BANDWITH);
		writeLoRaParams(loRa);
		startRxContinuous(loRa->rxhw,RECEIVE_PAYLOAD_LENGTH);
		break;
	case SET_SPREAD_FACTOR:
		dataLen = sizeof(loRa->sf);
		loRa->sf = dataReceived[DATA_START_INDEX] + 6;
		saveData(s, SPREAD_FACTOR);
		writeLoRaParams(loRa);
		startRxContinuous(loRa->rxhw,RECEIVE_PAYLOAD_LENGTH);
		break;
	case SET_CODING_RATE:
		dataLen = sizeof(loRa->cr);
		loRa->cr = dataReceived[DATA_START_INDEX];
		saveData(s, CODING_RATE);
		writeLoRaParams(loRa);
		startRxContinuous(loRa->rxhw,RECEIVE_PAYLOAD_LENGTH);
		break;
	case SET_OUT:
		dataLen = sizeof(s->aOut1) + sizeof(s->aOut2) + sizeof(s->dOut)
				+ sizeof(s->dOut2);
		s->aOut1 = dataReceived[DATA_START_INDEX + 0]
				| (dataReceived[DATA_START_INDEX + 1] << 8);
		s->aOut2 = dataReceived[DATA_START_INDEX + 2]
				| (dataReceived[DATA_START_INDEX + 3] << 8);
		s->dOut = dataReceived[DATA_START_INDEX + 4];

		s->dOut2 = dataReceived[DATA_START_INDEX + 5];
	//	saveData(s, AOUT_0_10V);
	//	saveData(s, AOUT_0_20mA);
	//	saveData(s, AOUT_4_20mA);
	//	saveData(s, DOUT);
		break;
	case SET_AOUT_0_10V:
		dataLen = sizeof(s->aOut1);
		s->aOut1 = dataReceived[DATA_START_INDEX]
				| (dataReceived[DATA_START_INDEX + 1] << 8);
		saveData(s, AOUT_0_10V);
		break;
	case SET_AOUT_4_20mA:
		dataLen = sizeof(s->aOut2);
		s->aOut2 = dataReceived[DATA_START_INDEX]
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
		dataLen = sizeof(s->dIn);
		s->dIn = dataReceived[DATA_START_INDEX];
		saveData(s, DOUT);
		break;
	default:
		// Comando no reconocido, responder con un mensaje de error o realizar otra acción
		return (-1);
	}

	uint8_t CRC_INDEX = DATA_START_INDEX + dataLen;
	uint8_t END_INDEX = CRC_INDEX + CRC_SIZE;
	dataReceived[DATA_LENGHT2_INDEX] = dataLen;
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
            if (loRa->bw < LORABW_7_8KHZ || loRa->bw > LORABW_500KHZ) {
                loRa->bw = LORABW_62_5KHZ;
                result = 1; // Set result to 1 (error).
            }
            break;
        case CODING_RATE:
            loRa->cr = readByte(i2c, K24C02_PAGE_ADDR(0), CR_OFFSET);
            if (loRa->cr < LORA_CR_4_5 || loRa->cr > LORA_CR_4_8) {
                loRa->cr = LORA_CR_4_6;
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
            s->aOut1 = read2Byte(i2c, K24C02_PAGE_ADDR(2), AOUT_0_10V_OFFSET);
            if (s->aOut1 == -1) {
                s->aOut1 = 0;
                result = 1; // Set result to 1 (error).
            }
            break;
        case AOUT_0_20mA:
            s->aOut2 = read2Byte(i2c, K24C02_PAGE_ADDR(2), AOUT_0_20mA_OFFSET);
            if (s->aOut2 == -1) {
                s->aOut2 = 0;
                result = 1; // Set result to 1 (error).
            }
            break;
        case AOUT_4_20mA:
            s->aOut2 = read2Byte(i2c, K24C02_PAGE_ADDR(2), AOUT_4_20mA_OFFSET);
            if (s->aOut2 == -1) {
                s->aOut2 = 0;
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
		if (loRa->bw < LORABW_7_8KHZ || loRa->bw > LORABW_500KHZ)
			loRa->bw = LORABW_62_5KHZ;
		data = (uint8_t*) &(loRa->bw);
		dataLen = sizeof(loRa->bw);
		break;
	case CODING_RATE:
		page = K24C02_PAGE_ADDR(0);
		offset = CR_OFFSET;
		if (loRa->cr < LORA_CR_4_5 || loRa->cr > LORA_CR_4_8)
			loRa->cr = LORA_CR_4_6;
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
		data = (uint8_t*) &(s->aOut1);
		dataLen = sizeof(s->aOut1);
		break;
	case AOUT_0_20mA:
		page = K24C02_PAGE_ADDR(2);
		offset = AOUT_0_20mA_OFFSET;
		data = (uint8_t*) &(s->aOut2);
		dataLen = sizeof(s->aOut2);
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

