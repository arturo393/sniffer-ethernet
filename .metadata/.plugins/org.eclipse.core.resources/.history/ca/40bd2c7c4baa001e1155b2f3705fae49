/*
 * sniffer.h
 *
 *  Created on: Jul 20, 2023
 *      Author: artur
 */

#ifndef INC_SNIFFER_H_
#define INC_SNIFFER_H_

#include "main.h"
#include "SX1278.h"
#include "rdss.h"
#include "k24c02.h"

#define SF_OFFSET 0
#define BW_OFFSET 1
#define CR_OFFSET 2
#define FUNCTION_OFFSET 3
#define ID_OFFSET 4
#define UL_OFFSET 0
#define DL_OFFSET 4
#define AOUT_0_10V_OFFSET 0
#define AOUT_4_20mA_OFFSET 2
#define AOUT_0_20mA_OFFSET 4
#define DOUT_OFFSET 6
#define ADC_WINDOW_SIZE 50

typedef enum {
	VOLTAGE0_10V_CH, SENSOR0_20mA_CH, ADC_CHANNELS, SENSOR4_20mACH} ADC_CHANNEL_T;

extern uint32_t adcValues[ADC_CHANNELS];
extern uint8_t adcCounter[ADC_CHANNELS];
extern uint16_t adcReadings[ADC_CHANNELS][ADC_WINDOW_SIZE];
extern uint16_t adcMA[ADC_CHANNELS];
extern uint32_t adcSum[ADC_CHANNELS];

typedef enum {
	SPREAD_FACTOR,
	BANDWITH,
	CODING_RATE,
	FUNCTION,
	ID,
	UPLINK,
	DOWNLINK,
	AOUT_0_10V,
	AOUT_4_20mA,
	AOUT_0_20mA,
	DOUT
} EEPROM_SECTOR_t;

typedef struct {
	GPIO_PinState dOut;
	GPIO_PinState dIn;
	GPIO_PinState dOut2;
	GPIO_PinState dIn2;
	GPIO_PinState dOut_switch;
	GPIO_PinState dIn_switch;
	uint16_t aIn1;
	uint16_t aIn2;
	uint16_t aIn3;
	uint16_t aOut1;
	uint16_t aOut2;
	uint16_t aOut3;
	Function_t function;
	uint8_t id;
	I2C_HandleTypeDef *i2c;
	SX1278_t *loRaTx;
	SX1278_t *loRaRx;
	SX1278_t *loRa;
	UART_t *serial;
	LORA_t *lora;
} Sniffer_t;

Sniffer_t* sniffer(I2C_HandleTypeDef *i2c,uint16_t *adcBuffer);
void processReceived(Sniffer_t *sniffer);
void slaveProcessRdss(RDSS_t *rdss, SX1278_t *loRa, Sniffer_t *sniffer);
void transmitStatus(SX1278_t *loRa, RDSS_t *rdss);
void processCommand(SX1278_t *loRa, RDSS_t *rdss, Sniffer_t *sniffer);
uint8_t exec(Sniffer_t *s, uint8_t *dataReceived);
HAL_StatusTypeDef saveData(Sniffer_t *s, EEPROM_SECTOR_t offset);
HAL_StatusTypeDef readData(Sniffer_t *s, EEPROM_SECTOR_t sector);
HAL_StatusTypeDef readEepromData(Sniffer_t *s, EEPROM_SECTOR_t sector);
void getLoRaParamsFromEeprom(Sniffer_t *s);

#endif /* INC_SNIFFER_H_ */
