/*
 * rx_ethernet_master.h
 *
 *  Created on: Feb 1, 2024
 *      Author: artur
 */

#ifndef SRC_RX_ETHERNET_MASTER_H_
#define SRC_RX_ETHERNET_MASTER_H_

#include "main.h"
#include "module.h"
#include "SX1278.h"
#include "rs485.h"


#define LORA_RX_LED_ON() HAL_GPIO_WritePin(LORA_RX_OK_GPIO_Port, LORA_RX_OK_Pin, GPIO_PIN_SET)
#define LORA_RX_LED_OFF() HAL_GPIO_WritePin(LORA_RX_OK_GPIO_Port, LORA_RX_OK_Pin, GPIO_PIN_RESET)

void process_lora_rx(SX1278_t *loRa);
void send_valid_data_to_uart(uint8_t *buffer, uint8_t length);

#endif /* SRC_RX_ETHERNET_MASTER_H_ */
