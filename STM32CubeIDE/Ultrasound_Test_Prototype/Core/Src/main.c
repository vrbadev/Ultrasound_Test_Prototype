/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

#define ADC_BUFFER_LEN 1000
#define USART_RX_BUFFER_LEN 256

#define MAX9939_SHDN (0x01 << 7)
#define MAX9939_MEAS (0x01 << 6)

#define MAX9939_VOS_NEG (0x01 << 5)
#define MAX9939_VOS_0_0MV (0x00 << 1)
#define MAX9939_VOS_1_3MV (0x01 << 1)
#define MAX9939_VOS_2_5MV (0x02 << 1)
#define MAX9939_VOS_3_8MV (0x03 << 1)
#define MAX9939_VOS_4_9MV (0x04 << 1)
#define MAX9939_VOS_6_1MV (0x05 << 1)
#define MAX9939_VOS_7_3MV (0x06 << 1)
#define MAX9939_VOS_8_4MV (0x07 << 1)
#define MAX9939_VOS_10_6MV (0x08 << 1)
#define MAX9939_VOS_11_7MV (0x09 << 1)
#define MAX9939_VOS_12_7MV (0x0A << 1)
#define MAX9939_VOS_13_7MV (0x0B << 1)
#define MAX9939_VOS_14_7MV (0x0C << 1)
#define MAX9939_VOS_15_7MV (0x0D << 1)
#define MAX9939_VOS_16_7MV (0x0E << 1)
#define MAX9939_VOS_17_6MV (0x0F << 1)

#define MAX9939_GAIN_0_25X ((0x09 << 1) | 0x01)
#define MAX9939_GAIN_1X ((0x00 << 1) | 0x01)
#define MAX9939_GAIN_10X ((0x01 << 1) | 0x01)
#define MAX9939_GAIN_20X ((0x02 << 1) | 0x01)
#define MAX9939_GAIN_30X ((0x03 << 1) | 0x01)
#define MAX9939_GAIN_40X ((0x04 << 1) | 0x01)
#define MAX9939_GAIN_60X ((0x05 << 1) | 0x01)
#define MAX9939_GAIN_80X ((0x06 << 1) | 0x01)
#define MAX9939_GAIN_120X ((0x07 << 1) | 0x01)
#define MAX9939_GAIN_157X ((0x08 << 1) | 0x01)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
uint32_t millis = 0;
uint32_t delay_end = 0;

uint8_t usart_rx_buffer[USART_RX_BUFFER_LEN];
uint32_t usart_rx_len = 0;

struct {
	uint8_t delimeter[5];
	uint8_t pga_gain : 8;
	int8_t pga_offset : 8;
	uint8_t adc_channel : 8;
	uint32_t id : 32;
	uint8_t adc_buffer[(3 * ADC_BUFFER_LEN) / 4];
	uint8_t dummy[2];
} usart_tx_packet;

uint32_t adc_num_conversions = 0;
uint16_t adc_buffer[ADC_BUFFER_LEN];
bool adc_dma_half_complete = false;
bool adc_dma_complete = false;

struct {
	uint8_t shdn;
	uint8_t meas;
	int gain;
	int offset;
} max9939_settings;

struct {
	char shdn_ch;
	char meas_ch;
	char* gain_str;
	char offset_sign;
	char* offset_str;
} max9939_info;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

void delay(uint32_t ms) {
	delay_end = millis + ms;
	while (millis < delay_end);
}

void adc_set_channel(uint32_t ch)
{
	HAL_ADC_Stop_DMA(&hadc1);
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ch;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer, ADC_BUFFER_LEN);
}

void compress_12bit_data(bool first_half)
{
	uint16_t* start = first_half ? (uint16_t*) adc_buffer : (uint16_t*) &(adc_buffer[ADC_BUFFER_LEN/2]);
	uint32_t o = 0;
	for (uint32_t i = 0; i < ADC_BUFFER_LEN / 2; i+=2) {
		usart_tx_packet.adc_buffer[o] = (uint8_t) (start[i] & 0x00FF);
		o++;
		usart_tx_packet.adc_buffer[o] = (uint8_t) ((start[i] & 0x0F00) >> 8) | ((start[i+1] & 0x0F00) >> 4);
		o++;
		usart_tx_packet.adc_buffer[o] = (uint8_t) (start[i+1] & 0x00FF);
		o++;
	}
	usart_tx_packet.id++;
}

void max9939_write(uint8_t reg)
{
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
	delay(1);
	HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
	while (hspi1.State == HAL_SPI_STATE_BUSY);
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
	delay(1);
}

void max9939_apply_settings()
{
	max9939_write((((uint8_t) max9939_settings.gain) << 1) | 0x01);

	int tmp = max9939_settings.offset;
	if (tmp < 0) {
		tmp = -tmp;
		if (tmp >= 0 && tmp <= 15) {
			max9939_write(max9939_settings.shdn | max9939_settings.meas | ((((uint8_t) tmp) << 1) | MAX9939_VOS_NEG));
		}
	} else {
		if (tmp >= 0 && tmp <= 15) {
			max9939_write(max9939_settings.shdn | max9939_settings.meas | (((uint8_t) tmp) << 1));
		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //HAL_ADC_Start_IT(&hadc1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  HAL_UART_Receive_DMA(&huart1, usart_rx_buffer, USART_RX_BUFFER_LEN);

  max9939_settings.shdn = 0x00;
  max9939_settings.meas = 0x00;
  max9939_settings.gain = 8;
  max9939_settings.offset = 10;
  max9939_apply_settings();

  for(int i = 0; i < sizeof(usart_tx_packet.delimeter); i++) { usart_tx_packet.delimeter[i] = 0xFF; }
  usart_tx_packet.id = 0;
  usart_tx_packet.adc_channel = 2;
  usart_tx_packet.pga_gain = (uint8_t) max9939_settings.gain;
  usart_tx_packet.pga_offset = (int8_t) max9939_settings.offset;

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer, ADC_BUFFER_LEN);

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int tmp;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (usart_rx_len > 0) {
		//printf("Received message: %lu\n", usart_rx_len);
		//HAL_UART_Transmit_DMA(&huart1, usart_rx_buffer, usart_rx_len); // echo
		switch (usart_rx_buffer[0]) {
		case 'S':
			max9939_settings.shdn = (usart_rx_buffer[1] == '1') ? MAX9939_SHDN : 0x00;
			max9939_apply_settings();
			break;
		case 'M':
			max9939_settings.meas = (usart_rx_buffer[1] == '1')? MAX9939_MEAS : 0x00;
			max9939_apply_settings();
			break;
		case 'G':
			usart_rx_buffer[usart_rx_len] = 0;
			tmp = atoi((char*) &(usart_rx_buffer[1]));
			if (tmp >= 0 && tmp <= 9) {
				max9939_settings.gain = tmp;
				usart_tx_packet.pga_gain = (uint8_t) max9939_settings.gain;
				max9939_apply_settings();
			}
			break;
		case 'O':
			usart_rx_buffer[usart_rx_len] = 0;
			tmp = atoi((char*) &(usart_rx_buffer[1]));
			if (tmp >= -15 && tmp <= 15) {
				max9939_settings.offset = tmp;
				usart_tx_packet.pga_offset = (int8_t) max9939_settings.offset;
				max9939_apply_settings();
			}
			break;
		case 'C':
			usart_rx_buffer[usart_rx_len] = 0;
			tmp = atoi((char*) &(usart_rx_buffer[1]));
			switch (tmp) {
			case 0:
				usart_tx_packet.adc_channel = 0;
				adc_set_channel(ADC_CHANNEL_0);
				break;
			case 1:
				usart_tx_packet.adc_channel = 1;
				adc_set_channel(ADC_CHANNEL_1);
				break;
			case 2:
				usart_tx_packet.adc_channel = 2;
				adc_set_channel(ADC_CHANNEL_2);
				break;
			}
			break;
		}
		usart_rx_len = 0;
	}

	if (adc_dma_half_complete) {
			compress_12bit_data(true);
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*) &usart_tx_packet, sizeof(usart_tx_packet));
			//HAL_UART_Transmit_DMA(&huart1, (uint8_t*) adc_buffer, ADC_BUFFER_LEN);

		adc_dma_half_complete = false;
	}

	if (adc_dma_complete) {
			compress_12bit_data(false);
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*) &usart_tx_packet, sizeof(usart_tx_packet));
			//HAL_UART_Transmit_DMA(&huart1, (uint8_t*) (&adc_buffer[ADC_BUFFER_LEN/2]), ADC_BUFFER_LEN);

		adc_dma_complete = false;
	}


	adc_num_conversions = 0;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_8;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_NVIC_SetPriority(ADC1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_IRQn);
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 399;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 4000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_CTS;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adc_num_conversions++;
	adc_dma_complete = true;
}


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	adc_dma_half_complete = true;
}
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_UART_Receive_DMA(&huart1, usart_rx_buffer, 1);
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	usart_rx_buffer_size++;
}*/


void USART1_RxDMA_Reset(void)
{
	HAL_UART_DMAStop(&huart1);
	hdma_usart1_rx.Instance->CNDTR = 0;
	HAL_UART_Receive_DMA(&huart1, usart_rx_buffer, USART_RX_BUFFER_LEN);
}

void USART1_IDLECallback(void)
{
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	usart_rx_len = sizeof(usart_rx_buffer) - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
	USART1_RxDMA_Reset();
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
