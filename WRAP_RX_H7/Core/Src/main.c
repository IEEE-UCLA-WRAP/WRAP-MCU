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
#include <math.h>
#include "constants.h"
#include "receiver.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int demodulate(const uint16_t * samples, int * symbs, params_r * params);
void costas_loop(float * norm_samples, float * samples_d, params_r * params);
uint8_t find_packet(float * symbs, uint8_t * bits, const int num_symbs);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

// sampling buffers
uint32_t adc_buf[ADC_BUF_LEN];			// adc buffer
uint16_t buffer_1[ADC_BUF_LEN];			// adc buffer
uint16_t buffer_2[ADC_BUF_LEN];			// adc buffer

float temp_symbs[2*NUM_SYMBS];				// 2x N_BUFF for slack
float symbol_buffer[SYMBOL_BUFF];			// symbol buffer
uint8_t bits[BITS];
uint8_t parity_check;

volatile uint8_t buff_flag_1 = RESET;
volatile uint8_t buff_flag_2 = RESET;
volatile uint8_t buff_process = RESET;

uint32_t start, end, demod_time, num_symbs, total_symbs, adc_start, adc_end, adc_time;
uint8_t t_str[NUM_CHARS];
float norm_samples[ADC_BUF_LEN];
float samples_d[ADC_BUF_LEN] = {0, 0, 0, 0, 0, 0};
float Quad[ADC_BUF_LEN] = {0, 0, 0, 0, 0, 0};
float filtered_samps[ADC_BUF_LEN + RRC_LEN - 1];

uint8_t packet_found;
uint8_t result;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Start timers
  HAL_TIM_Base_Start(&htim2);

  // Starts slave ADC (ADC2); this must be started before ADC1. It won't do anything until triggered by ADC1 anyways.
  HAL_ADC_Start(&hadc2);

  // Starts master ADC (ADC1) with fancy multi DMA command. Here is where we specify which buffer the DMA should store values in and how large the buffer is
  HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*) adc_buf, ADC_BUF_LEN);

  uint16_t * samples;
  uint8_t packet_found;

  // setup params
  params_r params = {.CL_phase = 0,
  					 .CL_integrator = 0,
					 .TR_phase = 0,
					 .TR_integrator = 0,
					 .sps = SPS};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// execute one buffer at a time. Look at SWV console to see if computation time is too long
	// alias buffer for ease
	if (buff_flag_1) {
	  samples = buffer_1;
	}
	if (buff_flag_2) {
	  samples = buffer_2;
	}

	if (buff_flag_1 || buff_flag_2) {
	  packet_found = 0;
	  // demodulate buffer
	  start = __HAL_TIM_GET_COUNTER(&htim2);
	  num_symbs = demodulate(samples, temp_symbs, &params);
	  end = __HAL_TIM_GET_COUNTER(&htim2);

	  demod_time = end - start;

	  total_symbs += num_symbs;
	  // add temp_symbs to running buffer for correlation
	  // shift latest entries
	  for (int j = 0; j < SYMBOL_BUFF-num_symbs; j++) {
		  symbol_buffer[j] = symbol_buffer[j+num_symbs];
	  }
	  for (int j = 0; j < num_symbs; j++) {
		  symbol_buffer[SYMBOL_BUFF-1-num_symbs+j] = temp_symbs[j];
	  }

	  if (total_symbs >= NUM_SYMBS) {
			packet_found = find_packet(symbol_buffer, bits, SYMBOL_BUFF);
			if (packet_found) {
				#if HAMMING_CODE_FLAG
					for (int i = 0; i < NUM_SYMBS - (NUM_PACKET_H * PACKET_HEADER_LEN); i = i+BITS_PER_CHAR) {
						result = 0;
						parity_check = 0;

						parity_check += bits[i+7] ^ bits[i+8] ^ bits[i+9] ^ bits[i+10] ^ bits[i+11];
						parity_check <<= 1;

						parity_check += bits[i+3] ^ bits[i+4] ^ bits[i+5] ^ bits[i+6] ^ bits[i+11];
						parity_check <<= 1;

						parity_check += bits[i+1] ^ bits[i+2] ^ bits[i+5] ^ bits[i+6] ^ bits[i+9] ^ bits[i+10];
						parity_check <<= 1;

						parity_check += bits[i+0] ^ bits[i+2] ^ bits[i+4] ^ bits[i+6] ^ bits[i+8] ^ bits[i+10];

						for(int j = 0; j < BITS_PER_CHAR; j++)
						{
							if(((j+1) & j) != 0){
								result <<= 1;
								if(parity_check != j+1){
									result += bits[i + j];
								}
								else{
									result += -1*bits[i + j] + 1;
								}
							}
						}

						t_str[i/BITS_PER_CHAR] = result;
					}

				#elif REPETITION_CODE_FLAG
					for (int i = 0; i < NUM_SYMBS - (NUM_PACKET_H * PACKET_HEADER_LEN); i = i+BITS_PER_CHAR) {
						result = 0;
						for(int j = 0; j < 8; j++)
						{
							uint8_t total_one_bits = 0;
							for(int k = 0; k < REPETITION_INVERSE_CODERATE; k++){
								total_one_bits += bits[i+REPETITION_INVERSE_CODERATE*j+k];
							}
							uint8_t decoded_bit = (uint8_t) (total_one_bits >= (REPETITION_INVERSE_CODERATE>>1) + 1);

							result <<= 1;
							result += decoded_bit;
						}
						t_str[i/BITS_PER_CHAR] = result;
					}
				#else
					for (int i = 0; i < NUM_SYMBS - (NUM_PACKET_H * PACKET_HEADER_LEN); i = i+8) {
						result = 0;
						for(int j = 0; j < 8; j++)
						{
							result <<= 1;
							result += bits[i + j];
						}
						t_str[i>>3] = result;
					}

				#endif
				HAL_UART_Transmit(&huart3, (uint8_t *)t_str, sizeof(t_str), 100);
			}

			total_symbs = 0;
	  }
	  buff_process = RESET;
	  buff_flag_1 = RESET;
	  buff_flag_2 = RESET;
	}

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
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /*AXI clock gating */
  RCC->CKGAENR = 0xFFFFFFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 70;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_1);

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_LSE;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,32768);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 18;
  PeriphClkInitStruct.PLL2.PLL2P = 3;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 6144;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_INTERL;
  multimode.DualModeData = ADC_DUALMODEDATAFORMAT_32_10_BITS;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_4CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 280;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  // toggles buffer status pin so sampling rate can be measured
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  adc_start = __HAL_TIM_GET_COUNTER(&htim2);

  // copies ADC/DMA temp buffer into sample buffer
  if (!buff_process){
	  buff_process = SET;
	  buff_flag_1 = SET;
	  buff_flag_2 = RESET;
	  for(int j = 0; j < ADC_BUF_LEN/2; j++)
	  {
		  buffer_1[2*j] = (uint16_t)(adc_buf[j]&0x0000FFFF);
		  buffer_1[2*j+1] = (uint16_t)(adc_buf[j]>>16);
	  }
  }
}

// Called when buffer is completely filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  // toggles buffer status pin so sampling rate can be measured
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  adc_end = __HAL_TIM_GET_COUNTER(&htim2);
  adc_time = adc_end - adc_start;

  // copies ADC/DMA temp buffer into sample buffer
  if (!buff_process){
	  buff_process = SET;
	  buff_flag_2 = SET;
	  buff_flag_1 = RESET;
	  for(int j = 0; j < ADC_BUF_LEN/2; j++) {
		  buffer_2[2*j] = (uint16_t)(adc_buf[j+ADC_BUF_LEN/2]&0x0000FFFF);
		  buffer_2[2*j+1] = (uint16_t)(adc_buf[j+ADC_BUF_LEN/2]>>16);
	  }
  }
}

int demodulate(const uint16_t * samples, int * symbs, params_r * params) {

    normalize(samples, norm_samples);

//     Costas Loop
    costas_loop(norm_samples, samples_d, params);
    // filter w SRRC
    arm_conv_f32(samples_d, ADC_BUF_LEN, RRC, RRC_LEN, filtered_samps);
    // readjust window
    float shift = RRC_LEN/2. - 0.5;
    int k;
    for (int i = shift ; i < ADC_BUF_LEN+RRC_LEN-1-shift; i++) {
        k = i - shift;
        filtered_samps[k] = filtered_samps[i];
    }

    // timing recovery
    int bit_len = timing_recovery(filtered_samps, symbs, params);

    return bit_len;
}

void costas_loop(float * norm_samples, float * samples_d, params_r * params) {
    float phase = params->CL_phase;
    float inph[2*ORDER+1] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    float quad[2*ORDER+1] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    float inph_[ORDER+1] = {0, 0, 0, 0, 0, 0};
    float quad_[ORDER+1] = {0, 0, 0, 0, 0, 0};
    double error = 0;
    float integrator = 0; //params->CL_integrator;

    float kp = 8.5;
    float ki = 0.1;
    float dt = (float)FC / (float)FS;

    for (int i = ORDER; i < ADC_BUF_LEN+ORDER; i++) {
        // define t from microcontroller
        int k = i - ORDER;
        inph_[ORDER] = norm_samples[k]*2*cos(2*M_PI*dt*k + phase);
        quad_[ORDER] = norm_samples[k]*-2*sin(2*M_PI*dt*k + phase);

        arm_conv_f32(inph_, ORDER+1, lp, ORDER+1, inph);
        arm_conv_f32(quad_, ORDER+1, lp, ORDER+1, quad);

        samples_d[k] = inph[ORDER];
        Quad[k] = quad[ORDER];

        error = inph[ORDER] * quad[ORDER];
        integrator += ki*error;
        phase = phase + kp*error + integrator;

        // shift the values of inph_ and quad_
        for (int jx = 1; jx < ORDER+1; jx++) {
            inph_[jx-1] = inph_[jx];
            quad_[jx-1] = quad_[jx];
        }
    }
    params->CL_phase = remainder(phase, 2*M_PI);
    params->CL_integrator = remainder(integrator, 2*M_PI);
}

uint8_t find_packet(float * symbs, uint8_t * bits, const int num_symbs) {
    // take cross correlation
    float xcorr_out[SYMBOL_BUFF+PACKET_HEADER_LEN-1];
    packet_found = 0;
    arm_correlate_f32(packet_header, PACKET_HEADER_LEN, symbs, num_symbs, xcorr_out);

    // find packet
    int shift = 0;
    for (int i = num_symbs-(NUM_PACKET_H-1)*PACKET_HEADER_LEN - 1; i >= 0; i--) {
        if (fabs(xcorr_out[i]) > PACKET_HEADER_LEN-1) {
            shift = SYMBOL_BUFF+PACKET_HEADER_LEN-1-i;
            packet_found = 1;
            if (xcorr_out[i] < 0) {
				for (int j = 0; j < BITS; j++) {
					symbs[shift + j] = symbs[shift+ j]*-1;
				}
            }
            break;
        }
    }

    if (!packet_found)
        return 0;

    // convert symbols to bits
    for (int i = 0; i < BITS; i++) {
        bits[i] = (symbs[shift+i]+1)*0.5;
    }
    return 1;
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
