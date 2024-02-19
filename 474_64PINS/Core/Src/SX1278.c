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
		| LOW_FREQUENCY_MODE | RX;
const uint8_t LORA_TX_MODE = LORA_MODE_ACTIVATION | LOW_FREQUENCY_MODE | TX;
const uint8_t DEFAULT_TX_ADDR = 0x80;

// Configure the registers and their values
const RegisterConfig_t config[] = { { LR_RegSyncWord, SYNCWORD }, {
LR_RegPaConfig, POWER }, { LR_RegOcp, OVERCURRENTPROTECT },
		{ LR_RegLna, LNAGAIN }, {
		LR_RegModemConfig3, AGCAUTON },
		{ LR_RegSymbTimeoutLsb, SYMBTIMEOUTLSB }, {
		LR_RegPreambleMsb, PREAMBLELENGTHMSB }, { LR_RegPreambleLsb,
				PREAMBLELENGTHLSB }, {
		LR_RegHopPeriod, FHSSVALUE } };

uint8_t readRegister(SX1278_t *modem, uint8_t address) {
	uint8_t rec = 0;
	HAL_GPIO_WritePin(modem->nssPort, modem->nssPin, GPIO_PIN_RESET); // pull the pin low
	HAL_Delay(1);
	HAL_SPI_Transmit(modem->spi, &address, 1, 100);  // send address
	HAL_SPI_Receive(modem->spi, &rec, 1, 100);  // receive 6 bytes data
	HAL_Delay(1);
	HAL_GPIO_WritePin(modem->nssPort, modem->nssPin, GPIO_PIN_SET); // pull the pin high
	return (rec);
}

void writeRegister(SX1278_t *modem, uint8_t address, uint8_t *cmd,
		uint8_t lenght) {
	if (lenght > 4)
		return;
	uint8_t tx_data[5] = { 0 };
	tx_data[0] = address | 0x80;
	int j = 0;
	for (int i = 1; i <= lenght; i++) {
		tx_data[i] = cmd[j++];
	}
	HAL_GPIO_WritePin(modem->nssPort, modem->nssPin, GPIO_PIN_RESET); // pull the pin low
	HAL_SPI_Transmit(modem->spi, tx_data, lenght + 1, 1000);
	HAL_GPIO_WritePin(modem->nssPort, modem->nssPin, GPIO_PIN_SET); // pull the pin high
	HAL_Delay(10);
}

uint8_t readReg(SX1276_HW_t *hw, uint8_t address) { //usar esta
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
	writeRegister(module, RegPaConfig, &(module->power), 1);
}

void setLORAWAN(SX1278_t *module) {
	writeRegister(module, LR_RegSyncWord, &(module->syncWord), 1);
}

void setOvercurrentProtect(SX1278_t *module) {
	writeRegister(module, RegOcp, &(module->ocp), 1);
}

void setLNAGain(SX1278_t *module) {
	writeRegister(module, RegLna, &(module->lnaGain), 1);
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
	cmd += module->CRC_sum << 2;
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
	module->operatingMode = (0x07 & readRegister(module, RegOpMode));
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
	writeRegister(loRa, LR_RegSyncWord, &(loRa->syncWord), 1);
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

//////////////////////Agregado mod FSK////////////////////////////////

void setFSKLowFreqModeReg(SX1278_t *module, OPERATING_MODE_t mode) {
	uint8_t cmd = FSK_OOK_MODE | MODULATIONFSK | LOW_FREQUENCY_MODE | mode; //averiguar lf registers
	writeRegister(module, RegOpMode, &cmd, 1);
	module->operatingMode = mode;

}

void setRFFrequencyReg_FSK(SX1278_t *module) {
	uint64_t freq = ((uint64_t) module->frequency << 19) / FXOSC;
	uint8_t freq_reg[3];
	freq_reg[0] = (uint8_t) (freq >> 16);
	freq_reg[1] = (uint8_t) (freq >> 8);
	freq_reg[2] = (uint8_t) (freq >> 0);
	writeRegister(module, RegFreqMsb, freq_reg, sizeof(freq_reg));
}

void fsk_config(SX1276_HW_t *hw, uint32_t frec) {

	uint8_t cmd = FSK_OOK_MODE | MODULATIONFSK | LOW_FREQUENCY_MODE | SLEEP; //Modo de Operación
	writeReg(hw, RegOpMode, &cmd, 1);

	const uint8_t RXCONFIG = RestartRXONCOLLISION | RESTARTRX_WITHOUT_PLLOCK
			| RESTARTRX_WITH_PLLOCK | AFC_AUTO_ON | AGC_AUTO_ON | RX_TRIGGER;

	static const uint8_t RF96configRegs[] = {
			RegBitRateMsb, 0x02, RegBitRateLsb, 0x8A, // Bit rate: 49230bps
			RegFdevMsb, 0x03, RegFdevLsb, 0x4E, // 51.5kHzFdev -> modulation index = 2.1
			RegPaConfig,0xFB, //(0x00<<7) | (0x04<<4) |(0x0f<<0) , // MAX OUTPUT POWER use PA_BOOST, start at 13dBm
			RegPaRamp, 0x09, // no shaping, 40us TX rise/fall
			RegOcp, 0x32, // Over-current protection @150mA
			RegLna, 0x20, // max LNA gain, no boost
			RegRxConfig, RXCONFIG, // AFC on, AGC on, AGC&AFC on preamble detect
			RegRssiConfig, 0x04, // 32-sample rssi smoothing (5 bit times)
			RegRssiCollision, 0x0A, // 10dB RSSI collision threshold
			//0x10, rssiThres*2, // RSSI threshold
			RegRxBw, 0x52, // RxBW 83kHz
			RegAfcBw, 0x4A, // AfcBw 125kHz
			RegAfcFei, 0x01, // clear AFC at start of RX
			RegPreambleDetect, 0xCA, // 3 byte preamble detector, tolerate 10 chip errors (2.5 bits)
			RegRxTimeout1, 0x00, // No RX timeout if RSSI doesn't happen
			RegRxTimeout2, 0x00, // No RX timeout if no preamble
			RegRxTimeout3, 0x00, // No RX timeout if no sync
			RegRxDelay, 0x02, // delay 8 bits after RX end before restarting RX
			RegOsc, 0x07, // no clock out
			RegPreambleMsb, 0x00, RegPreambleLsb, 0xff, // preamble 255 bytes
			RegSyncConfig, 0x12, // no auto-restart, 0xAA preamble, enable 3 byte sync
			RegSyncValue1, 0xAA, // sync1: same as preamble, gets us additional check
			RegSyncValue2, 0x2D, RegSyncValue3, 0x2A, // sync2 (fixed), and sync3 (network group)
			RegPacketConfig1, 0x00, // whitening, CRC on, no addr filt, CCITT CRC
			RegPacketConfig2, 0x40, // packet mode
			RegPayloadLength, 0,  // max RX packet length
			RegFifoThresh, 0x8F, // start TX when FIFO has 1 byte, FifoLevel intr when 15 bytes in FIFO
			RegDioMapping2, 0xF1, // dio5->mode-ready, dio4->preamble-detect intr
			RegPllHop, 0x00, // no fast-hop
			RegPaDac, 0x07, // enable 20dBm tx power
			0 };

	const uint8_t *ptr;
	ptr = RF96configRegs;
	while (true) {
		uint8_t addr = ptr[0];
		uint8_t cmd = ptr[1];
		if (addr == 0)
			break;
		writeReg(hw, addr, &cmd, 1);
		ptr += 2;
	}

	//Configuración modulación hw:
	/*
	 const uint8_t BR_19_2_KBPS_MSB = 0x06;
	 const uint8_t BR_19_2_KBPS_LSB = 0x83;
	 uint8_t preambleLengthMsb = 0;
	 uint8_t preambleLengthLsb = 0x03;
	 uint8_t cmd = FSK_OOK_MODE | MODULATIONFSK | LOW_FREQUENCY_MODE | SLEEP;
	 writeReg(hw, RegOpMode, &cmd, 1);
	 uint8_t opmode = readReg(hw, RegOpMode);

	 //Configuración bit rate:
	 writeReg(hw, RegBitRateMsb, &BR_19_2_KBPS_MSB, 1);
	 writeReg(hw, RegBitRateLsb, &BR_19_2_KBPS_LSB, 1);

	 //Configuración packet mode:

	 uint8_t CONFIG1 = PACKET_FORMAT | DC_FREE | CRC_ON | CRC_AUTO_CLEAR_OFF
	 | ADDRESS_FILTERING | CRC_WHITHENING_TYPE;
	 writeReg(hw, RegPacketConfig1, &CONFIG1, 1); //Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

	 uint8_t PACKET_MODE = (1 << 6) & 0xff;
	 uint8_t payloadLengthMSb = 0;
	 uint8_t payloadLengthLSb = 0;
	 cmd = PACKET_MODE;
	 cmd += payloadLengthMSb << 0;
	 writeReg(hw, RegPacketConfig2, &cmd, 1);
	 uint8_t statepacketconfig2 = readReg(hw, RegPacketConfig2);

	 cmd = payloadLengthLSb; // igual a cero para unlimited
	 writeReg(hw, RegPayloadLength, &cmd, 1);
	 uint8_t statepacketPayload = readReg(hw, RegPayloadLength);
	 */

	// Configuración frecuencia:
	while (frec < 100000000)
		frec *= 10;
	uint32_t frf = (frec << 2) / (32000000L >> 11);
	uint8_t freq_reg[3];
	freq_reg[0] = (uint8_t) (frf >> 10);
	freq_reg[1] = (uint8_t) (frf >> 2);
	freq_reg[2] = (uint8_t) (frf << 6);

	writeReg(hw, RegFreqMsb, freq_reg, sizeof(freq_reg));
	uint8_t statepacketFreqMSB = readReg(hw, RegFreqMsb);
	uint8_t statepacketFreqMID = readReg(hw, RegFreqMid);
	uint8_t statepacketFreqLSB = readReg(hw, RegFreqLsb);

	/*

	 //Configuración Preámbulo:
	 writeReg(hw, RegPreambleMsb, &(preambleLengthMsb), 1);
	 writeReg(hw, RegPreambleLsb, &(preambleLengthLsb), 1);

	 //Configuración sync word
	 uint8_t SYNC_SIZE = 0 << 0;
	 uint8_t syncword = 0xAA;
	 cmd = (SYNC_ON) | SYNC_SIZE;
	 writeReg(hw, RegSyncConfig, &cmd, 1);
	 writeReg(hw, RegSyncValue1, &(syncword), 1);

	 cmd = 0x07;
	 writeReg(hw, RegPaDac, &cmd, 1);
	 */

}

//////////////////////////////////////////////////////////////////////////

void changeMode(SX1278_t *modem, spi_mode_t mode) {

	if (mode == SLAVE_SENDER || mode == MASTER_SENDER) {
		modem->frequency =
				(mode == SLAVE_SENDER) ? modem->ulFreq : modem->dlFreq;
		modem->dioConfig = DIO0_TX_DONE | DIO1_RX_TIMEOUT
				| DIO2_FHSS_CHANGE_CHANNEL | DIO3_VALID_HEADER;
		modem->flagsMode = 0xff;
		CLEAR_BIT(modem->flagsMode, TX_DONE_MASK);
		modem->spi_mode = mode;
		modem->status = TX_MODE;

	} else if (mode == SLAVE_RECEIVER || mode == MASTER_RECEIVER) {
		modem->frequency =
				(mode == SLAVE_RECEIVER) ? modem->dlFreq : modem->ulFreq;
		modem->dioConfig = DIO0_RX_DONE | DIO1_RX_TIMEOUT
				| DIO2_FHSS_CHANGE_CHANNEL | DIO3_VALID_HEADER;
		modem->flagsMode = 0xff;
		CLEAR_BIT(modem->flagsMode, RX_DONE_MASK);
		CLEAR_BIT(modem->flagsMode, PAYLOAD_CRC_ERROR_MASK);
		modem->spi_mode = mode;
		modem->status = RX_MODE;
	}

	setLoRaLowFreqModeReg(modem, STANDBY);
	HAL_Delay(1);
	setRFFrequencyReg(modem);
	writeRegister(modem, LR_RegDioMapping1, &(modem->dioConfig), 1);
	clearIrqFlagsReg(modem);
	writeRegister(modem, LR_RegIrqFlagsMask, &(modem->flagsMode), 1);
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

uint8_t is_irqflag1_enable(SX1276_HW_t *hw, uint8_t irq_bit) {
	uint32_t PREAMBLE_DETECT_TIMEOUT = 5000;
	uint32_t timeStart = HAL_GetTick();
	uint8_t irgflags = readReg(hw, RegIrqFlags1);

	while ((irgflags & irq_bit) == 0) {
		irgflags = readReg(hw, RegIrqFlags1);
		if ((HAL_GetTick() - timeStart) > PREAMBLE_DETECT_TIMEOUT)
			return (0);

	}

	irgflags = irq_bit;
	writeReg(hw, RegIrqFlags1, &irgflags, 1);
	return (1);

}

uint8_t is_irqflag2_enable(SX1276_HW_t *hw, uint8_t irq_bit) {
	uint32_t PREAMBLE_DETECT_TIMEOUT = 5000;
	uint32_t timeStart = HAL_GetTick();
	uint8_t irgflags = readReg(hw, RegIrqFlags2);

	while ((irgflags & irq_bit) == 0) {
		irgflags = readReg(hw, RegIrqFlags2);
		if ((HAL_GetTick() - timeStart) > PREAMBLE_DETECT_TIMEOUT)
			return (0);

	}

	irgflags = irq_bit;
	writeReg(hw, RegIrqFlags1, &irgflags, 1);
	return (1);

}
void set_fsk_mode(SX1276_HW_t *hw, uint8_t mode) {
	uint8_t OpMode = FSK_OOK_MODE | MODULATIONFSK | HIGH_FREQUENCY_MODE | mode;
	writeReg(hw, RegOpMode, &OpMode, 1);
}

void set_fsk_level(SX1276_HW_t *hw, uint8_t level) {
	uint8_t cmd = 0;
	if (level < 2)
		level = 2;

	if (level > 20)
		level = 20;

	level = FSK_OOK_MODE | MODULATIONFSK | HIGH_FREQUENCY_MODE | STANDBY;
	writeReg(hw, RegOpMode, &level, 1);
	if (level > 17) {
		level = 0x87;
		writeReg(hw, RegPaDac, &(level), 1); // turn 20dBm mode on
		level = (0xf0 + level - 5);
		writeReg(hw, RegPaConfig, &(level), 1);
	} else {
		level = (0xf0 + level - 2);
		writeReg(hw, RegPaConfig, &level, 1);
		level = 0x84;
		writeReg(hw, RegPaDac, &(level), 1); // turn 20dBm mode off
	}
}

void set_fsk_tx_mode(FSK_t *fsk) {
	int timeStart = HAL_GetTick();
	SX1276_HW_t *hw = fsk->txhw;
	uint8_t cmd;
	uint8_t reg_fifo_thresh;

	fsk_config(hw, DOWNLINK_FREQ); //Modo SLEEP para las configuraciones

	set_fsk_level(hw, 20);

	set_fsk_mode(hw, STANDBY); // Para preparar los estados de transmisión

	set_fsk_mode(hw, FSTX); //Sintetizado de la frecuencia de transmisión

	while (!is_irqflag1_enable(hw, IRQ1_MODEREADY));

	uint8_t tx_data[] = { 0xC, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x2D, 0x2A, 0x22,
			0x33, 0x44, 0x55 };
	fsk->txSize = sizeof(tx_data);

	reg_fifo_thresh = readReg(hw, RegFifoThresh);
	uint8_t read_irqFlags = readReg(hw, RegIrqFlags2);

	for (int i = 0; i < fsk->txSize; i++) {
		writeReg(hw, RegFIFO, (tx_data + i), 1);
		uint8_t REGFIFOstate = readReg(hw, RegFIFO);
		read_irqFlags = readReg(hw, RegIrqFlags2);

	}

	uint8_t readFifo = readReg(hw,RegFIFO);

	set_fsk_mode(hw, TX); //Transmisión
}
/////////////////////////////////////////////////////////////////////////
uint8_t set_fsk_rx_mode(FSK_t *fsk){
	SX1276_HW_t *hw = fsk->rxhw;
	uint8_t readFIFO[64] = { 0 };
	uint8_t irqFlags1 = 0;

	fsk_config(hw, DOWNLINK_FREQ);//Modo SLEEP para las configuraciones

	set_fsk_mode(hw, STANDBY); //Para preparar el proceso de recepción

	set_fsk_mode(hw, FSRX); //Sintetizado de la frecuencia de recepción
	set_fsk_afc(hw);

	set_fsk_mode(hw, RX);//Recepción
	if (!is_irqflag1_enable(hw, IRQ1_RSSI))
		return (0);

	if (!is_irqflag1_enable(hw, PREAMBLEDETECT))
		return (0);
	for (uint8_t i = 0; i < 64; i++)
		readFIFO[i] = readReg(hw, RegFIFO); //Se lee FIFO FSK 64 bytes
	return (1);
}

void set_fsk_afc(SX1276_HW_t *hw) {
	static uint8_t RF96lnaMap[] = { 0, 0, 6, 12, 24, 36, 48, 48 };
	uint8_t rssi = readReg(hw, RegRssiValue);
	int16_t thresh = readReg(hw, RegRssiThresh);
	uint8_t snr = rssi > thresh ? 0 : (thresh - rssi) / 2;
	uint8_t lna = RF96lnaMap[(readReg(hw, RegLna) >> 5) & 0x7];
	uint16_t f = (uint16_t) readReg(hw, RegAfcMsb);
	f = (f << 8) | (uint16_t) readReg(hw, RegAfcLsb);
	uint32_t afc = (uint32_t) f * 61;
}



/*
 reg_fifo_thresh = readReg(hw, RegFifoThresh);
 reg_fifo_thresh |= (1 << 7);
 writeReg(hw, RegFifoThresh, &reg_fifo_thresh, 1);
 reg_fifo_thresh = 0;
 reg_fifo_thresh = readReg(hw, RegFifoThresh);
 writeReg(hw, RegFIFO, tx_data, sizeof(tx_data));
 */

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
uint8_t regflagsfsk = 0;

uint8_t readWhenDataArrive(LORA_t *loRa) {
	SX1276_HW_t *hw = loRa->rxhw;
//	if (HAL_GPIO_ReadPin(hw->dio0Port, hw->dio0Pin) == GPIO_PIN_RESET)
//		return (1);
	regflagsfsk = readReg(hw, LR_RegIrqFlags);
	if ((regflagsfsk & flags) != 0x40)
		return (1);
	flags = readReg(hw, LR_RegIrqFlags);
	writeReg(hw, LR_RegIrqFlags, &CLEARIRQ, 1);
	loRa->rxSize = readReg(hw, LR_RegRxNbBytes); //Number for received bytes
	return (0);
}

uint8_t startTransmition(LORA_t *loRa) {
	int timeStart = HAL_GetTick();
	SX1276_HW_t *hw = loRa->txhw;
	if (loRa->txSize == 0)
		return (0);
	uint8_t cmd = (uint8_t) (loRa->txSize);
	writeReg(hw, LR_RegPayloadLength, &(cmd), 1);
	writeReg(hw, LR_RegFifoAddrPtr, &DEFAULT_TX_ADDR, 1);

	//uint8_t regPayloadLength = 	readReg(hw, LR_RegPayloadLength);
	//uint8_t regFifoAddrPtr = readReg(hw, LR_RegFifoAddrPtr);

	//loRa->txSize = readReg(hw, LR_RegPayloadLength);
	for (int i = 0; i < loRa->txSize; i++)
		writeReg(hw, LR_RegFifo, loRa->txData + i, 1);
	writeReg(hw, LR_RegOpMode, &LORA_TX_MODE, 1);
	int writeTimeEnd = HAL_GetTick();
	loRa->writeTime = writeTimeEnd - timeStart;

	uint8_t irqFlags = 0;
	while (1) {
		irqFlags = readReg(hw, LR_RegIrqFlags);
		if (irqFlags & TX_DONE_MASK) {
			int timeEnd = HAL_GetTick();
			uint8_t cmd = 0xFF;	//agregado
			writeReg(hw, LR_RegIrqFlags, &cmd, 1);	//agregado
			loRa->lastTxTime = timeEnd - timeStart;
			return (loRa->txSize);
		}
		if (HAL_GetTick() - timeStart > LORA_SEND_TIMEOUT) {
			return (0);
		}
	}
}

LORA_t* loRa_Init(SPI_HandleTypeDef *spi1, SPI_HandleTypeDef *spi2) {
	LORA_t *l = malloc(sizeof(LORA_t));
	if (l != NULL) {
		l->rxhw = malloc(sizeof(SX1276_HW_t));
		l->txhw = malloc(sizeof(SX1276_HW_t));

		l->txSize = 0;

		l->txhw->spi = spi1;
		l->txhw->nssPin = SPI1_NSS_Pin;
		l->txhw->nssPort = SPI1_NSS_GPIO_Port;
		l->txhw->nrstPin = NRST_LORA_1_Pin;
		l->txhw->nrstPort = NRST_LORA_1_GPIO_Port;
		l->txhw->dio0Port = BUSSY_1_GPIO_Port;
		l->txhw->dio0Pin = BUSSY_1_Pin;

		l->rxhw->spi = spi2;
		l->rxhw->nssPin = SPI2_NSS_Pin;
		l->rxhw->nssPort = SPI2_NSS_GPIO_Port;
		l->rxhw->nrstPin = NRST_LORA_2_Pin;
		l->rxhw->nrstPort = NRST_LORA_2_GPIO_Port;
		l->rxhw->dio0Port = BUSSY_2_GPIO_Port;
		l->rxhw->dio0Pin = BUSSY_2_Pin;

		HAL_GPIO_WritePin(l->rxhw->nssPort, l->rxhw->nssPin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(l->rxhw->nrstPort, l->rxhw->nrstPin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(l->txhw->nssPort, l->txhw->nssPin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(l->txhw->nrstPort, l->txhw->nrstPin, GPIO_PIN_SET);
	}
	return (l);
}

FSK_t* fsk_Init(SPI_HandleTypeDef *spi1, SPI_HandleTypeDef *spi2) {
	FSK_t *f = malloc(sizeof(FSK_t));
	if (f != NULL) {
		f->rxhw = malloc(sizeof(SX1276_HW_t));
		f->txhw = malloc(sizeof(SX1276_HW_t));

		f->txSize = 0;

		f->txhw->spi = spi1;
		f->txhw->nssPin = SPI1_NSS_Pin;
		f->txhw->nssPort = SPI1_NSS_GPIO_Port;
		f->txhw->nrstPin = NRST_LORA_1_Pin;
		f->txhw->nrstPort = NRST_LORA_1_GPIO_Port;  //config led
		f->txhw->dio0Port = BUSSY_1_GPIO_Port;
		f->txhw->dio0Pin = BUSSY_1_Pin;

		f->rxhw->spi = spi2;
		f->rxhw->nssPin = SPI2_NSS_Pin;
		f->rxhw->nssPort = SPI2_NSS_GPIO_Port;
		f->rxhw->nrstPin = NRST_LORA_2_Pin;
		f->rxhw->nrstPort = NRST_LORA_2_GPIO_Port; //config led
		f->rxhw->dio0Port = BUSSY_2_GPIO_Port;
		f->rxhw->dio0Pin = BUSSY_2_Pin;

		HAL_GPIO_WritePin(f->rxhw->nssPort, f->rxhw->nssPin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(f->rxhw->nrstPort, f->rxhw->nrstPin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(f->txhw->nssPort, f->txhw->nssPin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(f->txhw->nrstPort, f->txhw->nrstPin, GPIO_PIN_SET);
	}
	return (f);
}

void initLoRaParameters(SX1278_t *module) {
	module->power = SX1278_POWER_17DBM;
	module->CRC_sum = CRC_ENABLE;
	module->ocp = OVERCURRENTPROTECT;
	module->lnaGain = LNAGAIN;
	module->AgcAutoOn = 12; // for L-TEL PROTOCOL
	module->syncWord = 0x12; // for L-TEL PROTOCOL
	module->symbTimeoutLsb = RX_TIMEOUT_LSB;
	module->preambleLengthMsb = PREAMBLE_LENGTH_MSB;
	module->preambleLengthLsb = PREAMBLE_LENGTH_LSB;
	module->preambleLengthLsb = 6; // for L-TEL PROTOCOL
	module->fhssValue = HOPS_PERIOD; // for L-TEL PROTOCOL
	module->len = 0;
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
	setLoRaLowFreqModeReg(loRa, RX);
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
	loRa->spi_mode = -1;
	loRa->power = SX1278_POWER_17DBM;
	loRa->CRC_sum = CRC_ENABLE;
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
	loRa->bw = BW_62_5KHZ;
	loRa->cr = CR_4_6;
	loRa->ulFreq = UPLINK_FREQ;
	loRa->dlFreq = DOWNLINK_FREQ;
	return (loRa);
}

void configureLoRaRx(SX1278_t *loRa, spi_mode_t mode) {
	if (loRa->spi_mode != mode)
		return;
	if (loRa->operatingMode == RX)
		return;
	changeMode(loRa, mode);
	setRxFifoAddr(loRa);
	setLoRaLowFreqModeReg(loRa, RX);
}

