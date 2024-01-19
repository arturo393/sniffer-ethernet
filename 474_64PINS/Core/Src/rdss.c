/*
 * rdss.c
 *
 *  Created on: Sep 28, 2022
 *      Author: sigmadev
 */
#include <rdss.h>

RDSS_t* rdss() {
	RDSS_t *r;
	r = malloc(sizeof(RDSS_t));
	r->len = 0;
	r->status = WAITING;
	r->cmd = NONE;
	r->id = 0;
	return (r);
}

RDSS_t* rdssInit(uint8_t id) {
	RDSS_t *r;
	r = malloc(sizeof(RDSS_t));
	r->len = 0;
	r->status = WAITING;
	r->cmd = NONE;
	r->id = id;
	return (r);
}

void rdssReinit(RDSS_t *rdss) {
	rdss->cmd = NONE;           // Reset the command field
	rdss->crcReceived = 0;      // Reset the received CRC value
	rdss->crcCalculated = 0;    // Reset the calculated CRC value
	rdss->idQuery = 0;          // Reset the query ID
	rdss->status = WAITING;     // Set the status to waiting
	rdss->idReceived = 0;       // Reset the received ID
}

RDSS_status_t rs485_check_CRC_module(UART_tt *uart1) {
	unsigned long crc_cal;
	unsigned long crc_save;
	crc_save = uart1->rxData[7] << 8;
	crc_save |= uart1->rxData[6];
	crc_cal = crc_get(&(uart1->rxData[1]), 5); ///ajustar si se cambia el largo
	if (crc_cal == crc_save)
		return DATA_OK;
	return CRC_ERROR;
}

RDSS_status_t checkModuleValidity(uint8_t *frame, uint8_t lenght) {
	if (frame[1] == VLADR) {
		for (int i = 3; i < lenght; i++)
			if (frame[i] == LTEL_END_MARK)
				return (VALID_MODULE);
	} else
		return (WRONG_MODULE_FUNCTION);
	return (WRONG_MODULE_FUNCTION);
}

RDSS_status_t checkFrameValidity(uint8_t *frame, uint8_t lenght) {

	if (lenght > (MINIMUM_FRAME_LEN)) {
		if (frame[0] == LTEL_START_MARK) {
			if (frame[lenght - 1] == LTEL_END_MARK)
				return (VALID_FRAME);
			else
				return (START_READING);
		} else
			return (NOT_VALID_FRAME);
	} else
		return (WAITING);
}

RDSS_status_t checkCRCValidity(uint8_t *frame, uint8_t len) {
	uint16_t calculatedCrc;
	uint16_t savedCrc;
	savedCrc = ((uint16_t) frame[len - 2] << 8);
	savedCrc |= (uint16_t) frame[len - 3];
	calculatedCrc = crc_get(&frame[1], len - 4);
	return ((calculatedCrc == savedCrc) ? DATA_OK : CRC_ERROR);
}

uint16_t crc_get(uint8_t *buffer, uint8_t buff_len) {
	uint8_t byte_idx;
	uint8_t bit_idx;
	uint16_t generator = 0x1021; // 16-bit divisor
	uint16_t crc = 0;            // 16-bit CRC value

	for (byte_idx = 0; byte_idx < buff_len; byte_idx++) {
		crc ^= ((uint16_t) (buffer[byte_idx] << 8)); // Move byte into MSB of 16-bit CRC

		for (bit_idx = 0; bit_idx < 8; bit_idx++) {
			if ((crc & 0x8000) != 0) { // Test for MSB = bit 15
				crc = ((uint16_t) ((crc << 1) ^ generator));
			} else {
				crc <<= 1;
			}
		}
	}

	return crc;
}

RDSS_status_t validate(uint8_t *buffer, uint8_t length) {
	RDSS_status_t frameStatus = checkFrameValidity(buffer, length);
	if (frameStatus != VALID_FRAME)
		return (frameStatus);
	RDSS_status_t moduleStatus = checkModuleValidity(buffer, length);
	if (moduleStatus != VALID_MODULE)
		return (moduleStatus);
	RDSS_status_t crcStatus = checkCRCValidity(buffer, length);
	if (crcStatus != DATA_OK)
		return (crcStatus);
	return (DATA_OK);
}

void reinit(RDSS_t *rs485) {
	rs485->cmd = NONE;
	rs485->status = WAITING;
}

void encodeVlad(uint8_t *buff) {
	uint16_t lineVoltage = (uint16_t) rand() % 610;
	uint16_t baseCurrent = (uint16_t) rand() % 301;
	uint16_t tunnelCurrent = (uint16_t) rand() % 1001;
	uint16_t unitCurrent = (uint16_t) rand() % 301;
	uint8_t uplinkAgc = (uint8_t) rand() % 43;
	uint8_t downlinkInputPower = (uint8_t) rand() % 130;
	uint8_t downlinkAgc = (uint8_t) rand() % 43;
	uint8_t uplinkOuputPower = (uint8_t) rand() % 130;

	buff[7] = (uint8_t) (lineVoltage >> 8) & 0xFF;
	buff[6] = (uint8_t) lineVoltage & 0xFF;
	buff[9] = (uint8_t) (baseCurrent >> 8) & 0xFF;
	buff[8] = (uint8_t) baseCurrent & 0xFF;
	buff[11] = (uint8_t) (tunnelCurrent >> 8) & 0xFF;
	buff[10] = (uint8_t) tunnelCurrent & 0xFF;
	buff[13] = (uint8_t) (unitCurrent >> 8) & 0xFF;
	buff[12] = (uint8_t) unitCurrent & 0xFF;
	buff[14] = uplinkAgc;
	buff[15] = downlinkInputPower;
	buff[16] = downlinkAgc;
	buff[17] = uplinkOuputPower;
}

uint8_t setCrc(uint8_t *buff, uint8_t size) {
	uint8_t crc_frame[2];
	uint16_t crc;
	crc = crc_get(buff + 1, size - 1);
	memcpy(crc_frame, &crc, 2);
	buff[size++] = crc_frame[0];
	buff[size++] = crc_frame[1];
	return (2);
}

uint8_t setRdssStartData(RDSS_t *rdss, uint8_t *buffer) {
	uint8_t i = 0;
	if (rdss->cmd == 0)
		return i;
	if (rdss->id == 0)
		return i;
	buffer[i++] = LTEL_START_MARK;
	buffer[i++] = VLADR;
	buffer[i++] = rdss->id;
	buffer[i++] = rdss->cmd;
	buffer[i++] = 0x00;
	return i;
}

int freqDecode(uint8_t *buffer) {
	union floatConverter freq;
	freq.i = 0;
	freq.i |= (buffer[0]);
	freq.i |= (buffer[1] << 8);
	freq.i |= (buffer[2] << 16);
	freq.i |= (buffer[3] << 24);
	freq.f = freq.f * 1000000.0f;

	return (int) (freq.f);
}

void freqEncode(uint8_t *buffer, uint32_t freqIn) {
	union floatConverter freqOut;
	freqOut.f = freqIn / 1000000.0f;
	memcpy(buffer, &freqOut.i, sizeof(freqOut.i));
}

void updateRdss(RDSS_t *rdss, uint8_t *buffer, uint8_t bufferSize) {
	if (buffer == NULL)
		return;
	if (bufferSize <= 0)
		return;
	rdss->cmd = buffer[CMD_INDEX]; // Update the command from the received data
	rdss->idReceived = buffer[MODULE_ID_INDEX]; // Update the received ID
	rdss->buffSize = bufferSize;
	rdss->buff = buffer;
}

