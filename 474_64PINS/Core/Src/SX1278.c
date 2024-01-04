/**
 * Author Wojciech Domski <Wojciech.Domski@gmail.com>
 * www: www.Domski.pl
 *
 * work based on DORJI.COM sample code and
 * https://github.com/realspinner/SX1278_LoRa
 */

#include "SX1278.h"

const uint8_t DIOMAPPING1TX = DIO0_TX_DONE | DIO1_RX_TIMEOUT
		| DIO2_FHSS_CHANGE_CHANNEL | DIO3_VALID_HEADER;
const uint8_t FLAGSMOODETX = 0xff & ~(TX_DONE_MASK);
const uint8_t CLEARIRQ = 0xff;
const uint8_t DIOMAPPING1RX = DIO0_RX_DONE | DIO1_RX_TIMEOUT
		| DIO2_FHSS_CHANGE_CHANNEL | DIO3_VALID_HEADER;
const uint8_t FLAGSMOODERX = 0xff & ~(RX_DONE_MASK) & ~(PAYLOAD_CRC_ERROR_MASK);
const uint8_t SYNCWORD = 0x12;
const uint8_t POWER = SX1278_POWER_17DBM;
const uint8_t CRC_SUM = CRC_ENABLE;
const uint8_t OVERCURRENTPROTECT = 0x0B; //Default value;
const uint8_t LNAGAIN = 0x23; //Highest gain & Boost on ;
const uint8_t AGCAUTON = 12;
const uint8_t SYMBTIMEOUTLSB = 0x08;
const uint8_t PREAMBLELENGTHMSB = 0x00;
const uint8_t PREAMBLELENGTHLSB = 12;
const uint8_t FHSSVALUE = 0x07;
const uint8_t LORA_SLEEP_MODE = LORA_MODE_ACTIVATION | LOW_FREQUENCY_MODE
		| SLEEP;
const uint8_t LORA_RX_CONTINUOUS_MODE = LORA_MODE_ACTIVATION
		| LOW_FREQUENCY_MODE | RX_CONTINUOUS;
const uint8_t LORA_TX_MODE = LORA_MODE_ACTIVATION | LOW_FREQUENCY_MODE | TX;
const uint8_t DEFAULT_TX_ADDR = 0x80;

// Configure the registers and their values
const RegisterConfig_t config[] = { { RegSyncWord, SYNCWORD }, { LR_RegPaConfig,
		POWER }, { LR_RegOcp, OVERCURRENTPROTECT }, { LR_RegLna, LNAGAIN }, {
LR_RegModemConfig3, AGCAUTON }, { LR_RegSymbTimeoutLsb, SYMBTIMEOUTLSB }, {
LR_RegPreambleMsb, PREAMBLELENGTHMSB },
		{ LR_RegPreambleLsb, PREAMBLELENGTHLSB }, {
		LR_RegHopPeriod, FHSSVALUE } };

uint8_t readRegister(SX1278_t *loRa, uint8_t address) {
	uint8_t rec = 0;
	HAL_GPIO_WritePin(loRa->nssPort, loRa->nssPin, GPIO_PIN_RESET); // pull the pin low
	HAL_Delay(1);
	HAL_SPI_Transmit(loRa->spi, &address, 1, 100);  // send address
	HAL_SPI_Receive(loRa->spi, &rec, 1, 100);  // receive 6 bytes data
	HAL_Delay(1);
	HAL_GPIO_WritePin(loRa->nssPort, loRa->nssPin, GPIO_PIN_SET); // pull the pin high
	return (rec);
}

void writeRegister(SX1278_t *loRa, uint8_t address, uint8_t *cmd,
		uint8_t lenght) {
	if (lenght > 4)
		return;
	uint8_t tx_data[5] = { 0 };
	tx_data[0] = address | 0x80;
	int j = 0;
	for (int i = 1; i <= lenght; i++) {
		tx_data[i] = cmd[j++];
	}
	HAL_GPIO_WritePin(loRa->nssPort, loRa->nssPin, GPIO_PIN_RESET); // pull the pin low
	HAL_SPI_Transmit(loRa->spi, tx_data, lenght + 1, 1000);
	HAL_GPIO_WritePin(loRa->nssPort, loRa->nssPin, GPIO_PIN_SET); // pull the pin high
	HAL_Delay(10);
}

uint8_t readReg(SX1276_HW_t *hw, uint8_t address) {
	uint8_t rec = 0;
	HAL_StatusTypeDef res;
	HAL_GPIO_WritePin(hw->nssPort, hw->nssPin, GPIO_PIN_RESET); // pull the pin low
	HAL_Delay(1);
	res = HAL_SPI_Transmit(hw->spi, &address, 1, 100);  // send address
	res = HAL_SPI_Receive(hw->spi, &rec, 1, 100);  // receive 6 bytes data
	HAL_Delay(1);
	HAL_GPIO_WritePin(hw->nssPort, hw->nssPin, GPIO_PIN_SET); // pull the pin high
	if (res != HAL_OK)
		Error_Handler();
	return (rec);
}

void writeReg(SX1276_HW_t *hw, uint8_t address, const uint8_t *cmd,
		uint8_t lenght) {
	HAL_StatusTypeDef res;
	if (lenght > 4)
		return;
	uint8_t tx_data[5] = { 0 };
	tx_data[0] = address | 0x80;
	int j = 0;
	for (int i = 1; i <= lenght; i++) {
		tx_data[i] = cmd[j++];
	}
	HAL_GPIO_WritePin(hw->nssPort, hw->nssPin, GPIO_PIN_RESET); // pull the pin low
	res = HAL_SPI_Transmit(hw->spi, tx_data, lenght + 1, 1000);
	HAL_GPIO_WritePin(hw->nssPort, hw->nssPin, GPIO_PIN_SET); // pull the pin high
	if (res != HAL_OK)
		Error_Handler();
	//HAL_Delay(10);
}

void setRFFrequencyReg(SX1278_t *module) {
	uint64_t freq = ((uint64_t) module->frequency << 19) / FXOSC;
	uint8_t freq_reg[3];
	freq_reg[0] = (uint8_t) (freq >> 16);
	freq_reg[1] = (uint8_t) (freq >> 8);
	freq_reg[2] = (uint8_t) (freq >> 0);
	writeRegister(module, LR_RegFrMsb, freq_reg, sizeof(freq_reg));
}

void setOutputPower(SX1278_t *module) {
	writeRegister(module, LR_RegPaConfig, &(module->power), 1);
}

void setLORAWAN(SX1278_t *module) {
	writeRegister(module, RegSyncWord, &(module->syncWord), 1);
}

void setOvercurrentProtect(SX1278_t *module) {
	writeRegister(module, LR_RegOcp, &(module->ocp), 1);
}

void setLNAGain(SX1278_t *module) {
	writeRegister(module, LR_RegLna, &(module->lnaGain), 1);
}

void setPreambleParameters(SX1278_t *module) {

	writeRegister(module, LR_RegSymbTimeoutLsb, &(module->symbTimeoutLsb), 1);
	writeRegister(module, LR_RegPreambleMsb, &(module->preambleLengthMsb), 1);
	writeRegister(module, LR_RegPreambleLsb, &(module->preambleLengthLsb), 1);
}

void setRegModemConfig(SX1278_t *module) {
	uint8_t cmd = 0;
	cmd = module->bw << 4;
	cmd += module->cr << 1;
	cmd += module->headerMode;
	writeRegister(module, LR_RegModemConfig1, &cmd, 1); //Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

	cmd = module->sf << 4;
	cmd += module->LoRa_CRC_sum << 2;
	cmd += module->symbTimeoutMsb;
	writeRegister(module, LR_RegModemConfig2, &cmd, 1);
	writeRegister(module, LR_RegModemConfig3, &(module->AgcAutoOn), 1);
}

void setDetectionParametersReg(SX1278_t *module) {
	uint8_t tmp;
	tmp = readRegister(module, LR_RegDetectOptimize);
	tmp &= 0xF8;
	tmp |= 0x05;
	writeRegister(module, LR_RegDetectOptimize, &tmp, 1);
	tmp = 0x0C;
	writeRegister(module, LR_RegDetectionThreshold, &tmp, 1);
}

void readOperatingMode(SX1278_t *module) {
	module->operatingMode = (0x07 & readRegister(module,
	LR_RegOpMode));
}

void setLoRaLowFreqModeReg(SX1278_t *module, OPERATING_MODE_t mode) {
	uint8_t cmd = LORA_MODE_ACTIVATION | LOW_FREQUENCY_MODE | mode;
	writeRegister(module, LR_RegOpMode, &cmd, 1);
	module->operatingMode = mode;
}

void clearIrqFlagsReg(SX1278_t *module) {
	uint8_t cmd = 0xFF;
	writeRegister(module, LR_RegIrqFlags, &cmd, 1);
}

void writeLoRaParametersReg(SX1278_t *loRa) {
	setLoRaLowFreqModeReg(loRa, SLEEP);
	HAL_Delay(15);
	setRFFrequencyReg(loRa);
	writeRegister(loRa, RegSyncWord, &(loRa->syncWord), 1);
	setOutputPower(loRa);
	setOvercurrentProtect(loRa);
	writeRegister(loRa, LR_RegLna, &(loRa->lnaGain), 1);
	if (loRa->sf == SF_6) {
		loRa->headerMode = IMPLICIT;
		loRa->symbTimeoutMsb = 0x03;
		setDetectionParametersReg(loRa);
	} else {
		loRa->headerMode = EXPLICIT;
		loRa->symbTimeoutMsb = 0x00;
	}

	setRegModemConfig(loRa);
	setPreambleParameters(loRa);
	writeRegister(loRa, LR_RegHopPeriod, &(loRa->fhssValue), 1);
	writeRegister(loRa, LR_RegDioMapping1, &(loRa->dioConfig), 1);
	clearIrqFlagsReg(loRa);
	writeRegister(loRa, LR_RegIrqFlagsMask, &(loRa->flagsMode), 1);
}

void changeMode(SX1278_t *loRa, Lora_Mode_t mode) {

	if (mode == SLAVE_SENDER || mode == MASTER_SENDER) {
		loRa->frequency = (mode == SLAVE_SENDER) ? loRa->ulFreq : loRa->dlFreq;
		loRa->dioConfig = DIO0_TX_DONE | DIO1_RX_TIMEOUT
				| DIO2_FHSS_CHANGE_CHANNEL | DIO3_VALID_HEADER;
		loRa->flagsMode = 0xff;
		CLEAR_BIT(loRa->flagsMode, TX_DONE_MASK);
		loRa->mode = mode;
		loRa->status = TX_MODE;

	} else if (mode == SLAVE_RECEIVER || mode == MASTER_RECEIVER) {
		loRa->frequency =
				(mode == SLAVE_RECEIVER) ? loRa->dlFreq : loRa->ulFreq;

		loRa->dioConfig = DIO0_RX_DONE | DIO1_RX_TIMEOUT
				| DIO2_FHSS_CHANGE_CHANNEL | DIO3_VALID_HEADER;
		loRa->flagsMode = 0xff;
		CLEAR_BIT(loRa->flagsMode, RX_DONE_MASK);
		CLEAR_BIT(loRa->flagsMode, PAYLOAD_CRC_ERROR_MASK);
		loRa->mode = mode;
		loRa->status = RX_MODE;
	}
	setLoRaLowFreqModeReg(loRa, STANDBY);
	HAL_Delay(1);
	setRFFrequencyReg(loRa);
	writeRegister(loRa, LR_RegDioMapping1, &(loRa->dioConfig), 1);
	clearIrqFlagsReg(loRa);
	writeRegister(loRa, LR_RegIrqFlagsMask, &(loRa->flagsMode), 1);
}

void setFreqReg(SX1276_HW_t *hw, uint32_t frequency) {
	uint8_t regOpMode = LORA_MODE_ACTIVATION | LOW_FREQUENCY_MODE | STANDBY;
	uint64_t freq = ((uint64_t) frequency << 19) / FXOSC;
	uint8_t freq_reg[3];
	freq_reg[0] = (uint8_t) (freq >> 16);
	freq_reg[1] = (uint8_t) (freq >> 8);
	freq_reg[2] = (uint8_t) (freq >> 0);
	writeReg(hw, LR_RegOpMode, &regOpMode, 1);
	HAL_Delay(1);
	writeReg(hw, LR_RegFrMsb, freq_reg, sizeof(freq_reg));
}

void setMode(SX1276_HW_t *hw, uint32_t freq, uint8_t dioMapping,
		uint8_t irqFlagsMask) {
	uint8_t regOpMode = LORA_MODE_ACTIVATION | LOW_FREQUENCY_MODE | STANDBY;
	uint64_t freqReal = ((uint64_t) freq << 19) / FXOSC;
	uint8_t freq_reg[3];
	freq_reg[0] = (uint8_t) (freqReal >> 16);
	freq_reg[1] = (uint8_t) (freqReal >> 8);
	freq_reg[2] = (uint8_t) (freqReal >> 0);

	writeReg(hw, LR_RegOpMode, &regOpMode, 1);
	HAL_Delay(1);
	writeReg(hw, LR_RegFrMsb, freq_reg, sizeof(freq_reg));
	writeReg(hw, LR_RegDioMapping1, &dioMapping, 1);
	writeReg(hw, LR_RegIrqFlags, &CLEARIRQ, 1);
	writeReg(hw, LR_RegIrqFlagsMask, &irqFlagsMask, 1);
}

void setTxMode(SX1276_HW_t *hw, uint32_t freq) {
	setMode(hw, freq, DIOMAPPING1TX, FLAGSMOODETX);
}

void setRxMode(SX1276_HW_t *hw, uint32_t freq) {
	setMode(hw, freq, 0x00, FLAGSMOODERX);
}

void writeLoRaParams(LORA_t *loRa) {
	uint8_t headerMode;
	uint8_t symbTimeoutMsb;

	writeReg(loRa->txhw, LR_RegOpMode, &LORA_SLEEP_MODE, 1);
	writeReg(loRa->rxhw, LR_RegOpMode, &LORA_SLEEP_MODE, 1);

	HAL_Delay(15);
	if (loRa->sf == SF_6) {
		headerMode = IMPLICIT;
		symbTimeoutMsb = 0x03;
		uint8_t tmp;
		tmp = readReg(loRa->txhw, LR_RegDetectOptimize);
		tmp = readReg(loRa->rxhw, LR_RegDetectOptimize);
		tmp &= 0xF8;
		tmp |= 0x05;
		writeReg(loRa->txhw, LR_RegDetectOptimize, &tmp, 1);
		writeReg(loRa->rxhw, LR_RegDetectOptimize, &tmp, 1);
		tmp = 0x0C;
		writeReg(loRa->txhw, LR_RegDetectionThreshold, &tmp, 1);
		writeReg(loRa->rxhw, LR_RegDetectionThreshold, &tmp, 1);
	} else {
		headerMode = EXPLICIT;
		symbTimeoutMsb = 0x00;
	}

	uint8_t cmd = 0;
	cmd = loRa->bw << 4;
	cmd += loRa->cr << 1;
	cmd += headerMode;
	writeReg(loRa->txhw, LR_RegModemConfig1, &cmd, 1); //Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)
	writeReg(loRa->rxhw, LR_RegModemConfig1, &cmd, 1);
	cmd = loRa->sf << 4;
	cmd += CRC_SUM << 2;
	cmd += symbTimeoutMsb;
	writeReg(loRa->txhw, LR_RegModemConfig2, &cmd, 1);
	writeReg(loRa->rxhw, LR_RegModemConfig2, &cmd, 1);
}

void writeCommon(SX1276_HW_t *hw) {
	writeReg(hw, LR_RegOpMode, &LORA_SLEEP_MODE, 1);
	HAL_Delay(15);
	for (uint8_t i = 0; i < sizeof(config) / sizeof(config[0]); i++) {
		writeReg(hw, config[i].reg, &config[i].value, 1);
	}
}

void restartRegFifoAddrPtr(SX1276_HW_t *hw) {
	uint8_t zero = 0;
	writeReg(hw, LR_RegOpMode, &LORA_SLEEP_MODE, 1);
	HAL_Delay(15);
	writeReg(hw, LR_RegFifoAddrPtr, &zero, 1);
}

void startRxContinuous(SX1276_HW_t *hw, uint8_t payloadLength) {
	uint8_t addr = 0;
	writeReg(hw, LR_RegOpMode, &(LORA_SLEEP_MODE), 1);
	//writeReg(hw, LR_RegPayloadLength, &(payloadLength), 1); //RegPayloadLength 21byte
	writeReg(hw, LR_RegFifoAddrPtr, &addr, 1); //RegFifoAddrPtr
	writeReg(hw, LR_RegOpMode, &(LORA_RX_CONTINUOUS_MODE), 1);

}
uint8_t flags = 0x40;
uint8_t regflags = 0;
uint8_t readWhenDataArrive(LORA_t *loRa) {
	SX1276_HW_t *hw = loRa->rxhw;

//	if (HAL_GPIO_ReadPin(hw->dio0Port, hw->dio0Pin) == GPIO_PIN_RESET)
//		return (1);
	regflags = readReg(hw,LR_RegIrqFlags );
	if((regflags & flags) != 0x40)
		return(1);
//flags = readReg(hw,LR_RegIrqFlags );
	writeReg(hw, LR_RegIrqFlags, &CLEARIRQ, 1);
	loRa->rxSize = readReg(hw, LR_RegRxNbBytes); //Number for received bytes
	/*
	if (loRa->rxSize > 0 ) {
		uint8_t addr = 0x00;
		HAL_GPIO_WritePin(hw->nssPort, hw->nssPin, GPIO_PIN_RESET); // pull the pin low
		HAL_Delay(1);
		HAL_SPI_Transmit(hw->spi, &addr, 1, 100); // send address
		HAL_SPI_Receive(hw->spi, loRa->rxData, loRa->rxSize, 100); // receive 6 bytes data
		HAL_Delay(1);
		HAL_GPIO_WritePin(hw->nssPort, hw->nssPin, GPIO_PIN_SET); // pull the pin high
	}
	*/ //sacado
	return (0);
}

uint8_t startTransmition(LORA_t *loRa) {
	int timeStart = HAL_GetTick();
	SX1276_HW_t *hw = loRa->txhw;
	if (loRa->txSize == 0)
		return (0);

	writeReg(hw, LR_RegPayloadLength, &(loRa->txSize), 1);
	writeReg(hw, LR_RegFifoAddrPtr, &DEFAULT_TX_ADDR, 1);
	//loRa->txSize = readReg(hw, LR_RegPayloadLength);
	for (int i = 0; i < loRa->txSize; i++)
		writeReg(hw, LR_RegFifo, loRa->txData + i, 1);
	writeReg(hw, LR_RegOpMode, &LORA_TX_MODE, 1);
	int writeTimeEnd = HAL_GetTick();
	loRa->writeTime = writeTimeEnd - timeStart;


	uint8_t irqFlags = 0;
	while (1) {
		irqFlags = readReg(hw, LR_RegIrqFlags);
		if (HAL_GPIO_ReadPin(hw->dio0Port, hw->dio0Pin) || (irqFlags & 0x08)) {
			int timeEnd = HAL_GetTick();
			uint8_t cmd = 0xFF;//agregado
			writeReg(hw, LR_RegIrqFlags, &cmd, 1);//agregado
			loRa->lastTxTime = timeEnd - timeStart;
			return (loRa->txSize);
		}
		if (HAL_GetTick() - timeStart > LORA_SEND_TIMEOUT) {
			return (0);
		}
	}
}

LORA_t* loRa_Init(SPI_HandleTypeDef *spi1,SPI_HandleTypeDef *spi2) {
	LORA_t *l = malloc(sizeof(LORA_t));
	if (l != NULL) {
		l->rxhw = malloc(sizeof(SX1276_HW_t));
		l->txhw = malloc(sizeof(SX1276_HW_t));

		l->rxhw->nssPin = SPI1_NSS_Pin;
		l->rxhw->nssPort = SPI1_NSS_GPIO_Port;
		l->rxhw->nrstPin = NRST_LORA_1_Pin;
		l->rxhw->nrstPort = NRST_LORA_1_GPIO_Port;
		l->rxhw->dio0Port = BUSSY_1_GPIO_Port;
		l->rxhw->dio0Pin = BUSSY_1_Pin;
		l->rxhw->spi = spi1;


		l->txhw->nssPin = SPI2_NSS_Pin;
		l->txhw->nssPort = SPI2_NSS_GPIO_Port;
		l->txhw->nrstPin = NRST_LORA_2_Pin;
		l->txhw->nrstPort = NRST_LORA_2_GPIO_Port;
		l->txhw->dio0Port = BUSSY_2_GPIO_Port;
		l->txhw->dio0Pin = BUSSY_2_Pin;
		l->txhw->spi = spi2;
		HAL_GPIO_WritePin(l->rxhw->nssPort, l->rxhw->nssPin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(l->rxhw->nrstPort, l->rxhw->nrstPin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(l->txhw->nssPort, l->txhw->nssPin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(l->txhw->nrstPort, l->txhw->nrstPin, GPIO_PIN_SET);
	}
	return (l);
}

void initLoRaParameters(SX1278_t *module) {
	module->power = SX1278_POWER_17DBM;
	module->LoRa_CRC_sum = CRC_ENABLE;
	module->ocp = OVERCURRENTPROTECT;
	module->lnaGain = LNAGAIN;
	module->AgcAutoOn = 12; // for L-TEL PROTOCOL
	module->syncWord = 0x12; // for L-TEL PROTOCOL
	module->symbTimeoutLsb = RX_TIMEOUT_LSB;
	module->preambleLengthMsb = PREAMBLE_LENGTH_MSB;
	module->preambleLengthLsb = PREAMBLE_LENGTH_LSB;
	module->preambleLengthLsb = 12; // for L-TEL PROTOCOL
	module->fhssValue = HOPS_PERIOD; // for L-TEL PROTOCOL
	module->len = 9;
}

void sx1278Reset(SX1278_t *loRa) {
	HAL_GPIO_WritePin(loRa->nrstPort, loRa->nrstPin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(loRa->nrstPort, loRa->nrstPin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(loRa->nrstPort, loRa->nrstPin, GPIO_PIN_SET);
	HAL_Delay(100);
}

void waitForTxEnd(SX1278_t *loRa) {
	int timeStart = HAL_GetTick();
	uint8_t irqFlags = 0;
	while (1) {
		irqFlags = readRegister(loRa, LR_RegIrqFlags);
		if (HAL_GPIO_ReadPin(loRa->dio0Port, loRa->dio0Pin)
				|| (irqFlags & 0x08)) {
			int timeEnd = HAL_GetTick();
			loRa->lastTxTime = timeEnd - timeStart;
//			readRegister(loRa, LR_RegIrqFlags);
//			clearIrqFlagsReg(loRa);
			loRa->status = TX_DONE;
			return;
		}
		if (HAL_GetTick() - timeStart > LORA_SEND_TIMEOUT) {
			sx1278Reset(loRa);
			loRa->status = TX_TIMEOUT;
			return;
		}
		//HAL_Delay(1);
	}
}

uint8_t waitForRxDone2(SX1278_t *loRa) {
	uint32_t timeout = HAL_GetTick();
	while ((!HAL_GPIO_ReadPin(loRa->dio0Port, loRa->dio0Pin))) {
		uint8_t flags = readRegister(loRa, LR_RegIrqFlags);
		if (READ_BIT(flags, PAYLOAD_CRC_ERROR_MASK)) {
			uint8_t cmd = flags | (1 << 7);
			writeRegister(loRa, LR_RegIrqFlags, &cmd, 1);
			flags = readRegister(loRa, LR_RegIrqFlags);
		}
		if (HAL_GetTick() - timeout > 2000)
			return (-1);
	}
	return (0);
}

uint8_t waitForRxDone(SX1278_t *loRa) {
	uint32_t timeout = HAL_GetTick();
	while (!HAL_GPIO_ReadPin(loRa->dio0Port, loRa->dio0Pin)) {
		uint8_t flags = readRegister(loRa, LR_RegIrqFlags);
		if (flags & PAYLOAD_CRC_ERROR_MASK) {
			flags |= (1 << 7);
			writeRegister(loRa, LR_RegIrqFlags, &flags, 1);
			flags = readRegister(loRa, LR_RegIrqFlags);
		}
		if (HAL_GetTick() - timeout > 2000) {
			return (-1);
		}
	}
	return (0);
}

void setRxFifoAddr(SX1278_t *module) {
	setLoRaLowFreqModeReg(module, SLEEP); //Change modem mode Must in Sleep mode
	uint8_t cmd = module->rxSize;
	//cmd = 9;
	writeRegister(module, LR_RegPayloadLength, &(cmd), 1); //RegPayloadLength 21byte
	uint8_t addr = readRegister(module, LR_RegFifoRxBaseAddr); //RegFiFoTxBaseAddr
	addr = 0x00;
	writeRegister(module, LR_RegFifoAddrPtr, &addr, 1); //RegFifoAddrPtr
	module->rxSize = readRegister(module, LR_RegPayloadLength);
}

int crcErrorActivation(SX1278_t *module) {
	uint8_t flags;
	flags = readRegister(module, LR_RegIrqFlags);
	SET_BIT(flags, RX_DONE_MASK);
	writeRegister(module, LR_RegIrqFlags, &flags, 1);
	flags = readRegister(module, LR_RegIrqFlags);
	uint8_t errorActivation = READ_BIT(flags, PAYLOAD_CRC_ERROR_MASK);
	return (errorActivation);
}

uint8_t* getRxFifoData(SX1278_t *loRa) {
	loRa->rxSize = readRegister(loRa, LR_RegRxNbBytes); //Number for received bytes
	if (loRa->rxSize > 0) {
		loRa->rxData = malloc(sizeof(uint8_t) * loRa->rxSize);
		uint8_t addr = 0x00;
		HAL_GPIO_WritePin(loRa->nssPort, loRa->nssPin, GPIO_PIN_RESET); // pull the pin low
		HAL_Delay(1);
		HAL_SPI_Transmit(loRa->spi, &addr, 1, 100); // send address
		HAL_SPI_Receive(loRa->spi, loRa->rxData, loRa->rxSize, 100); // receive 6 bytes data
		HAL_Delay(1);
		HAL_GPIO_WritePin(loRa->nssPort, loRa->nssPin, GPIO_PIN_SET); // pull the pin high
		loRa->status = RX_DONE;
	}

	return (loRa->rxData);
}

void setTxFifoAddr(SX1278_t *module) {
	uint8_t cmd = module->len;
	writeRegister(module, LR_RegPayloadLength, &(cmd), 1);
	uint8_t addr = readRegister(module, LR_RegFifoTxBaseAddr);
	addr = 0x80;
	writeRegister(module, LR_RegFifoAddrPtr, &addr, 1);
	module->len = readRegister(module, LR_RegPayloadLength);
}

uint8_t setTxFifoData(SX1278_t *loRa) {
	uint8_t cmd = loRa->txSize;
	if (loRa->txSize > 0) {
		writeRegister(loRa, LR_RegPayloadLength, &(cmd), 1);
		uint8_t addr = readRegister(loRa, LR_RegFifoTxBaseAddr);
		addr = 0x80;
		writeRegister(loRa, LR_RegFifoAddrPtr, &addr, 1);
		loRa->txSize = readRegister(loRa, LR_RegPayloadLength);
		for (int i = 0; i < loRa->txSize; i++)
			writeRegister(loRa, 0x00, loRa->txData + i, 1);
	}
	return (loRa->txSize);
}

void receive(SX1278_t *loRa) {
	setRxFifoAddr(loRa);
	setLoRaLowFreqModeReg(loRa, RX_CONTINUOUS);
	clearRxMemory(loRa);
	waitForRxDone(loRa);
	getRxFifoData(loRa);
}

void transmit(SX1278_t *loRa) {
	setTxFifoData(loRa);
	setLoRaLowFreqModeReg(loRa, TX);
	waitForTxEnd(loRa);
//	memset(loRa->buffer, 0, sizeof(loRa->buffer));
//	loRa->len = 0;
}

SX1278_t* loRaInit(SPI_HandleTypeDef *spi) {
	SX1278_t *loRa;
	loRa = malloc(sizeof(SX1278_t));
	loRa->spi = spi;
	loRa->mode = -1;
	loRa->power = SX1278_POWER_17DBM;
	loRa->LoRa_CRC_sum = CRC_ENABLE;
	loRa->ocp = OVERCURRENTPROTECT;
	loRa->lnaGain = LNAGAIN;
	loRa->AgcAutoOn = 12; // for L-TEL PROTOCOL
	loRa->syncWord = 0x12; // for L-TEL PROTOCOL
	loRa->symbTimeoutLsb = RX_TIMEOUT_LSB;
	loRa->preambleLengthMsb = PREAMBLE_LENGTH_MSB;
	loRa->preambleLengthLsb = PREAMBLE_LENGTH_LSB;
	loRa->preambleLengthLsb = 12; // for L-TEL PROTOCOL
	loRa->fhssValue = HOPS_PERIOD; // for L-TEL PROTOCOL
	loRa->len = 9;
	loRa->rxSize = 0;
	loRa->txSize = 0;
	loRa->sf = SF_10;
	loRa->bw = LORABW_62_5KHZ;
	loRa->cr = LORA_CR_4_6;
	loRa->ulFreq = UPLINK_FREQ;
	loRa->dlFreq = DOWNLINK_FREQ;
	return (loRa);
}

void configureLoRaRx(SX1278_t *loRa, Lora_Mode_t mode) {
	if (loRa->mode != mode)
		return;
	if (loRa->operatingMode == RX_CONTINUOUS)
		return;
	changeMode(loRa, mode);
	setRxFifoAddr(loRa);
	setLoRaLowFreqModeReg(loRa, RX_CONTINUOUS);
}

