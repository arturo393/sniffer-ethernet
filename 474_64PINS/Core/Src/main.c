/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "sniffer.h"
#include "wizchip_conf.h"
#include "socket.h"
#include "w5500_spi.h"
#include "ethernet.h"
#include <string.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define funciones_main  0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DAC_HandleTypeDef hdac2;

I2C_HandleTypeDef hi2c3;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint16_t adc_result[2];
uint8_t dato[5];
uint8_t dato2[5];
uint8_t re[50];
uint8_t re2[50];
GPIO_PinState sw;    // PB9
uint8_t eth = 72; // H
uint16_t offset_address;
uint8_t BSB;
uint8_t RWB;
uint8_t OM;
uint8_t DATA[240];
uint8_t control_phase;
uint8_t buffer[243] = { 0 };
uint8_t buffer_t[3];
uint8_t *p;
uint8_t buffer2[3000];

uint8_t sn[4];
uint8_t buffer3[3000];
uint8_t *rt;
GPIO_PinState NSHD;	 // PC0
GPIO_PinState S485_N232; // PC15
GPIO_PinState DE485_F232; // PC13
GPIO_PinState NRE485; // PA12
GPIO_PinState NTE485; // PA1
GPIO_PinState LB; // PA5

uint8_t c = 0;
uint16_t RxXferdif = 0;
uint8_t set1 = 0;
uint8_t set2 = 0;
uint8_t time1, time2, timeNow, timeDiff;

uint8_t gar[] = { 192, 168, 60, 1 };
uint8_t sub_r[] = { 255, 255, 255, 0 };
uint8_t shar[] = { 0x00, 0x08, 0xDC, 0x01, 0x02, 0x03 };
uint8_t sipr[] = { 192, 168, 60, 101 };
uint8_t mode = 0x012;
int8_t socket_mode = 0x01; //TCP
uint8_t S_MR = 0x01;
uint8_t S_CR_open = 0x01;
uint8_t S_CR_discon = 0x08;
uint8_t S_CR_con = 0x04;
uint8_t S_CR_recv = 0x40;
uint8_t S_SR = 0x13;
uint8_t S_PORT[2] = { 0x0B, 0xB8 }; //PUERTO 3000
uint8_t S_DPORT[2] = { 0x03, 0xE8 }; // PUERTO 1000
uint8_t S_MMS[2] = { 0x05, 0xB4 };
uint8_t S_RXBUF_SIZE = 0x01; // 1KB
uint8_t S_TXBUF_SIZE = 0x01; // 1KB
uint8_t S_CR_listen = 0x02;
uint8_t _PHYCFGR_RST = 0x1D;
uint8_t _PHYCFGR_NRST = 0X9D;
uint8_t read_reg = 1;
uint8_t S_DHAR[6] = { 0x08, 0xDC, 0x00, 0x01, 0x02, 0x0A };
uint8_t S_TTL = 0x40;

uint8_t s_MR; //0
uint8_t s_CR; //1
uint8_t s_IR; //2
uint8_t s_SR; //3
uint8_t s_PORT[2]; //4
uint8_t s_DHAR[6]; //6
uint8_t s_DIPR[4]; //C
uint8_t s_DPORT[2]; //10
uint8_t s_MSS[2]; //12
uint8_t s_TOS; //15
uint8_t s_TTL; //16
uint8_t s_RXBUF_SIZE; //1E
uint8_t s_TXBUF_SIZE; //1F
uint8_t s_TX_FS[2]; //20
uint8_t s_TX_RD[2]; //22
uint8_t s_TX_WR[2]; //24

uint8_t s_RX_WR[2]; //2A
uint8_t s_IMR; //2C
uint8_t s_FRAG[2]; //2D
uint8_t s_KPALVTR; //2F

uint8_t read_IR = 0;

uint8_t socket_status = 0; // 1 CON, 2 DISCN, 3 RECV, 4 TIMEOUT, 5 SENDOK
uint8_t tx_data[4] = { 84, 69, 83, 84 }; // TEST
uint8_t send_socket0 = 0x20;
uint16_t point = 0;
uint16_t q = 0;

uint8_t s_send_ok = 0; //0x10
uint8_t s_timeout = 0; //0x08
uint8_t s_recv = 0; //0x04
uint8_t s_discon = 0; //0x02
uint8_t s_con = 0; //0x01
uint16_t len_rx = 0;
uint16_t offset_read = 0;
uint8_t data_transmition[3000];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_DAC2_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

Sniffer_t *s;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	for (int adcIdx = 0; adcIdx < ADC_CHANNELS; adcIdx++) {

		adcSum[adcIdx] -= adcReadings[adcIdx][adcCounter[adcIdx]];
		if ((int32_t) adcSum[adcIdx] < 0)
			adcSum[adcIdx] = 0;

		// Store the new value in the buffer
		adcReadings[adcIdx][adcCounter[adcIdx]] = adcValues[adcIdx];

		// Add new value to the sumh
		adcSum[adcIdx] += adcValues[adcIdx];

		// Calculate and return the average
		adcMA[adcIdx] = (adcSum[adcIdx] / ADC_WINDOW_SIZE);

		// Increment the current index and wrap around if necessary
		adcCounter[adcIdx]++;
		if (adcCounter[adcIdx] >= ADC_WINDOW_SIZE) {
			adcCounter[adcIdx] = 0;
		}

	}

	HAL_ADC_Stop_DMA(&hadc1);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// Read received data from UART1
	/*
	 c=1;
	 HAL_UART_Receive_IT(huart,re, 50);
	 */
	//uint8_t *data = s->serial_lora->data;     ///////////////REVISAR///////////////////
	//uint8_t len;
	//&len =&( s->serial->len);
	s->serial_lora->len++;
	/*
	 if (len >= UART_SIZE-1) {
	 s->serial->len = 0;
	 s->serial_lora->isReceivedDataReady = true;
	 memset(data, 0, UART_SIZE);
	 }
	 HAL_UART_Receive_IT(s->serial->handler, s->serial->data+s->serial->len, 1);
	 */
	//processReceived(s);
	//	HAL_UART_Receive_IT(huart,re, 50);
	HAL_UART_Receive_IT(s->serial_lora->handler,
			s->serial_lora->data + s->serial_lora->len, 1);
	s->serial_lora->isReceivedDataReady = true;

}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	 uint32_t keep_alive_counter = HAL_GetTick();
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */

	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_DAC1_Init();
	MX_DAC2_Init();
	MX_I2C3_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_SPI2_Init();
	MX_TIM1_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */


	s = sniffer(&hi2c3, adcMA);

	s->fsk = fsk_Init(&hspi1, &hspi2);




	// lora config
	// writeCommon(s->lora->txhw);
	//writeCommon(s->lora->rxhw);
	// setTxMode(s->lora->txhw, s->lora->ufreq);
	// setRxMode(s->lora->rxhw, s->lora->dfreq);
	// writeLoRaParams(s->lora);
	// startRxContinuous(s->lora->rxhw, RECEIVE_PAYLOAD_LENGTH);
	//
     /*
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1); // KA
	// RS485/N232 LTC2873
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0); // NRESET W5500
	HAL_Delay(2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1); // NRESET W5500
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1); // NSHD
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1); // S485_N232
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, 0); // DE485_F232
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0); // NRE485
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0); // NTE485
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0); // LB
	//HAL_UART_Receive_IT(&huart1, re, 50); sacado
	sw = HAL_GPIO_ReadPin(DIN2_GPIO_Port, DIN2_Pin);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, sw); // S485_N232

	//SPI ETHERNET
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1); // CS HIGH DISABLE
    */
	//-----------------------------------------------------------

	//configuración fisicas del chip//
	/*
	eth_write_reg(COMMON_REG_OFFSET, PHYCFGR_RST_OFFSET,(uint8_t*) &_PHYCFGR_RST, sizeof(_PHYCFGR_RST));
	HAL_Delay(500);
	eth_write_reg(COMMON_REG_OFFSET, PHYCFGR_RST_OFFSET,(uint8_t*) &_PHYCFGR_NRST, sizeof(_PHYCFGR_NRST));
	HAL_Delay(200);


	//configuración de registros comunes
	common_reg_config(buffer, mode, gar, sub_r, shar, sipr);

	socket_reg_config(buffer, S_MR, S_PORT, S_DHAR, S_DPORT, S_MMS, S_TTL,
			S_RXBUF_SIZE, S_TXBUF_SIZE, S_CR_open, S_CR_listen);
   */



	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {

		//readWhenDataArrive(s->lora);
		//startTransmition(loRa);
		//processReceived(s);


		set_fsk_tx_mode(s->fsk);
		set_fsk_rx_mode(s->fsk);


		/*
		// LEER REGISTRO IR
		eth_read_reg(socket_0_register,  S_IR_OFFSET, &s_IR, sizeof(s_IR));
		if (s_IR & Sn_IR_MASK) {
			uint8_t ir_reset;
			if (s_IR & Sn_CONNECT){  // SOCK_ESTABLISHED
				socket_cmd_cfg(socket_0_register, S_CR_CONNECT);
				ir_reset = Sn_CONNECT;
			}
			if (s_IR & Sn_DISCONNECT) { //FIN/ACK
				socket_cmd_cfg(socket_0_register, S_CR_DISCONECT);
				socket_cmd_cfg(socket_0_register, S_CR_OPEN);
				socket_cmd_cfg(socket_0_register, S_CR_LISTEN);
				ir_reset = Sn_DISCONNECT;
			}
			if ((s_IR & Sn_RECEIVE)) {
					/* readDataFromEthernet
					s->eth_lenRX = read_socket_n_rx_buffer(socket_0_register,s->eth_bufRX);
					socket_cmd_cfg(socket_0_register, S_CR_RECV);
					ir_reset = Sn_RECEIVE;
			}
			eth_write_reg(socket_0_register, S_IR_OFFSET, (uint8_t*) &ir_reset,
					sizeof(ir_reset));

		}




        // lectura status del socket
		eth_read_reg(socket_0_register,  S_SR_OFFSET, &s_SR, sizeof(s_SR));


		if (HAL_GetTick() - keep_alive_counter > 1000)
		              keep_alive_counter = HAL_GetTick();
		          else {
		              if (HAL_GetTick() - keep_alive_counter > 50)
		                HAL_GPIO_WritePin(KA_GPIO_Port, KA_Pin, GPIO_PIN_RESET);
		              else
		                HAL_GPIO_WritePin(KA_GPIO_Port, KA_Pin, GPIO_PIN_SET);
		          }

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3

	*/
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.GainCompensation = 0;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC1_Init(void) {

	/* USER CODE BEGIN DAC1_Init 0 */

	/* USER CODE END DAC1_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC1_Init 1 */

	/* USER CODE END DAC1_Init 1 */

	/** DAC Initialization
	 */
	hdac1.Instance = DAC1;
	if (HAL_DAC_Init(&hdac1) != HAL_OK) {
		Error_Handler();
	}

	/** DAC channel OUT1 config
	 */
	sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
	sConfig.DAC_DMADoubleDataMode = DISABLE;
	sConfig.DAC_SignedFormat = DISABLE;
	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC1_Init 2 */

	/* USER CODE END DAC1_Init 2 */

}

/**
 * @brief DAC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC2_Init(void) {

	/* USER CODE BEGIN DAC2_Init 0 */

	/* USER CODE END DAC2_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC2_Init 1 */

	/* USER CODE END DAC2_Init 1 */

	/** DAC Initialization
	 */
	hdac2.Instance = DAC2;
	if (HAL_DAC_Init(&hdac2) != HAL_OK) {
		Error_Handler();
	}

	/** DAC channel OUT1 config
	 */
	sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
	sConfig.DAC_DMADoubleDataMode = DISABLE;
	sConfig.DAC_SignedFormat = DISABLE;
	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac2, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC2_Init 2 */

	/* USER CODE END DAC2_Init 2 */

}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void) {

	/* USER CODE BEGIN I2C3_Init 0 */

	/* USER CODE END I2C3_Init 0 */

	/* USER CODE BEGIN I2C3_Init 1 */

	/* USER CODE END I2C3_Init 1 */
	hi2c3.Instance = I2C3;
	hi2c3.Init.Timing = 0x00303D5B;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C3_Init 2 */

	/* USER CODE END I2C3_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 7;
	hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 16000 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 99;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, H_F_Pin | TE485_Pin | DOUT2_Pin | BUSSY_2_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,
	GPIO_PIN_1 | KA_Pin | SWSERIAL_Pin | GPIO_PIN_12 | SPI1_NSS_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
	GPIO_PIN_1 | KA_Pin | GPIO_PIN_5 | GPIO_PIN_9 | GPIO_PIN_12 | SPI1_NSS_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			DOUT_Pin | BUSSY_1_Pin | LORA_TX_Pin | NRST_LORA_1_Pin | DIO1_1_Pin
					| DIO3_1_Pin | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(NRST_LORA_2_GPIO_Port, NRST_LORA_2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : H_F_Pin TE485_Pin DOUT2_Pin BUSSY_2_Pin */
	GPIO_InitStruct.Pin = H_F_Pin | TE485_Pin | DOUT2_Pin | BUSSY_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI2_NSS_Pin */
	GPIO_InitStruct.Pin = SPI2_NSS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI2_NSS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : NINT_W5500_Pin PC1 DIN2_Pin current_out_select_Pin
	 current_in_select_Pin DIO1_2_Pin DIO3_2_Pin */
	GPIO_InitStruct.Pin = NINT_W5500_Pin | GPIO_PIN_1 | DIN2_Pin
			| current_out_select_Pin | current_in_select_Pin | DIO1_2_Pin
			| DIO3_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA1 KA_Pin PA9 PA12
	 SPI1_NSS_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | KA_Pin | GPIO_PIN_9 | GPIO_PIN_12
			| SPI1_NSS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA5 */
	//GPIO_InitStruct.Pin = GPIO_PIN_5; sacado
	/*Configure GPIO pin : SWSERIAL_Pin */
	GPIO_InitStruct.Pin = SWSERIAL_Pin;

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(SWSERIAL_GPIO_Port, &GPIO_InitStruct); //GPIOA

	/*Configure GPIO pin : DIN_Pin1 */
	GPIO_InitStruct.Pin = DIN1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DIN1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : DOUT_Pin BUSSY_1_Pin LORA_TX_Pin NRST_LORA_1_Pin
	 DIO1_1_Pin DIO3_1_Pin PB6 PB7 */
	GPIO_InitStruct.Pin = DOUT_Pin | BUSSY_1_Pin | LORA_TX_Pin | NRST_LORA_1_Pin
			| DIO1_1_Pin | DIO3_1_Pin | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : NRST_LORA_2_Pin */
	GPIO_InitStruct.Pin = NRST_LORA_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(NRST_LORA_2_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
