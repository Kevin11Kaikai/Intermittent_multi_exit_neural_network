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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "tensorflow/lite/micro/kernels/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
#include "./TF/main_functions1.h"
#include "./TF/main_functions2.h"
// pre-build steps: arm-none-eabi-gcc -P -E ../STM32H743VITX_FLASH.c -o ../GENERATED.ld
// link script: ${workspace_loc:/${ProjName}/GENERATED.ld}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define VREFINT_CAL_ADDR 0x1FF1E860
#define ADDR_VREFINT_CAL ((uint16_t*) VREFINT_CAL_ADDR)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
__IO uint16_t uhADCxConvertedValue = 0;
uint16_t ADC_max_binary = ((1<<16)-1);
float ADC_ref_voltage = 3.3;
float ADC_sys_voltage = 0.0;
RTC_HandleTypeDef RTCHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC3_Init(void);
static void MX_DCMI_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void stop_mode1(void);
void stop_mode2(void);

#ifdef __cplusplus
extern "C" {
#endif
extern int sensor_init(void);
#ifdef __cplusplus
}
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* Enable the CPU Cache */
  CPU_CACHE_Enable();
  /* USER CODE END 1 */

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
  MX_TIM3_Init();
  MX_RTC_Init();
  MX_ADC3_Init();
  MX_DCMI_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
//  stop_mode1();
//  stop_mode2();

//  printf("start OV init\r\n");
//  fb_alloc_init0();
//  framebuffer_init0();
//  sensor_init0();
//  dma_alloc_init0();

//  int ret = sensor_init();
//  printf("finished OV init, return=%d\r\n", ret);

  float target_voltage = 3.9;
  float sys_vol;
  stop_mode2_duration(200);
  sys_vol = adc_ref_internal_read();
  stop_mode2_duration(200);
  sys_vol = adc_ref_internal_read();
  if (sys_vol < target_voltage)
  {
	printf("System start FAILED, voltage=%.2f, target=%.2f, going to deep sleep\r\n", sys_vol, target_voltage);
	// will restart after this sleep
	stop_mode1_duration(60000);
  }

  // NN
  setup1();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t  start1;
  uint32_t  stop1;
  uint32_t  delta1;

  int sample_idx = 3;
  while (1)
  {
//  start = DWT->CYCCNT;
	start1 = HAL_GetTick();
// Code to measure
//	main_loop1();
//  main_loop2();

	sample_idx = 2;
	main_single_event(sample_idx, 0.0);
//	sample_idx += 1;
//	sample_idx %= 4;
//	if (sample_idx == 0)
//	{
//		sample_idx = 3;
//	}

//  stop = DWT->CYCCNT;
	stop1 = HAL_GetTick();
//	delta = stop - start;
	delta1 = stop1 - start1;
//	d_ms = delta / (SYSCLK / 1000);

//	fprintf(stderr, "start: %ld, end: %ld, delta: %ld, d_ms: %ld, delta1 %ld (ms)\r\n", start, stop, delta, d_ms, delta1);
//	fprintf(stderr, "delta1 %ld (ms)\r\n", delta1);
	fprintf(stderr, "\r\n");

//	HAL_Delay(10000);
//	stop_mode1();
	stop_mode2();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 25;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 9;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 3072;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_19;
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
  /* Run the ADC calibration in single-ended mode */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }

//  while (1)
//  {
//	  adc_read();
//  }
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */
  /* ### - 4 - Start conversion ############################################# */
  if (HAL_ADC_Start(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /* Infinite Loop */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_EMBEDDED;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_FALLING;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.SyncroCode.FrameEndCode = 0;
  hdcmi.Init.SyncroCode.FrameStartCode = 0;
  hdcmi.Init.SyncroCode.LineStartCode = 0;
  hdcmi.Init.SyncroCode.LineEndCode = 0;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00C0EAFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 3276, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 5;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  /*##-2- Start the transmission process #####################################*/
  /* While the UART in reception process, user can transmit data through
     "aTxBuffer" buffer */
  /* Buffer used for transmission */
  uint8_t aTxBuffer[] = "**** UART_TwoBoards_ComPolling ****\r\n";

  /* Buffer used for reception */
//  uint8_t aRxBuffer[RXBUFFERSIZE];
//  while (1)
  {
//	  if(HAL_UART_Transmit(&huart3, (uint8_t*)aTxBuffer, sizeof(aTxBuffer), 5000)!= HAL_OK)
//	  {
//	    Error_Handler();
//	  }
//	  printf("123 %f\r\n", 4.56);
	  printf("USART init\r\n");
//	  HAL_Delay(1000);
  }

  /*##-3- Put UART peripheral in reception process ###########################*/
//  if(HAL_UART_Receive(&huart3, (uint8_t *)aRxBuffer, RXBUFFERSIZE, 5000) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pins : PC0 PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void adc_read(void)
{
	/*##-3- Start the conversion process #######################################*/
	if (HAL_ADC_Start(&hadc1) != HAL_OK)
	{
		/* Start Conversation Error */
		Error_Handler();
	}

	/*##-4- Wait for the end of conversion #####################################*/
	/*  For simplicity reasons, this example is just waiting till the end of the
conversion, but application may perform other tasks while conversion
operation is ongoing. */
	if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
	{
		/* End Of Conversion flag not set on time */
		Error_Handler();
	}
	else
	{
		/* ADC conversion completed */
		/*##-5- Get the converted value of regular channel  ########################*/
		uhADCxConvertedValue = HAL_ADC_GetValue(&hadc1);
		ADC_sys_voltage = (uhADCxConvertedValue / (float)ADC_max_binary) * ADC_ref_voltage;
	}

	printf("ADC: %d, V: %.4f\r\n", uhADCxConvertedValue, ADC_sys_voltage);
}

float adc_ref_internal_read(void)
{
	uint16_t VREFINT_CAL, VREFINT_DATA;
	VREFINT_CAL = *ADDR_VREFINT_CAL;
	float vref_int_voltage = 0.0;

	/*##-3- Start the conversion process #######################################*/
	if (HAL_ADC_Start(&hadc3) != HAL_OK)
	{
		/* Start Conversation Error */
		Error_Handler();
	}

	/*##-3- Start the conversion process #######################################*/
	if (HAL_ADC_Start(&hadc1) != HAL_OK)
	{
		/* Start Conversation Error */
		Error_Handler();
	}

	/*##-4- Wait for the end of conversion #####################################*/
	/*  For simplicity reasons, this example is just waiting till the end of the
conversion, but application may perform other tasks while conversion
operation is ongoing. */
	if (HAL_ADC_PollForConversion(&hadc3, 10) != HAL_OK)
	{
		/* End Of Conversion flag not set on time */
		Error_Handler();
	}
	else
	{
		/* Read the converted value */
		VREFINT_DATA = HAL_ADC_GetValue(&hadc3);

		/* Convert the result from 16 bit value to the voltage dimension (mV unit) */
		/* Vref = 3.3 V */
//		uwInputVoltage = (uwConvertedValue * 3300) / 0xFFFF;
		vref_int_voltage = (VREFINT_DATA / (float)ADC_max_binary) * ADC_ref_voltage;
	}

	// measure channel voltage

	/*##-4- Wait for the end of conversion #####################################*/
	/*  For simplicity reasons, this example is just waiting till the end of the
	conversion, but application may perform other tasks while conversion
	operation is ongoing. */
	if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
	{
		/* End Of Conversion flag not set on time */
		Error_Handler();
	}
	else
	{
		/* ADC conversion completed */
		/*##-5- Get the converted value of regular channel  ########################*/
		uhADCxConvertedValue = HAL_ADC_GetValue(&hadc1);
		ADC_sys_voltage = 3.3f * (float)VREFINT_CAL * uhADCxConvertedValue / ((float)VREFINT_DATA * ADC_max_binary);
		// resistor divide
		ADC_sys_voltage *= 4.0f;
		// compensate
		ADC_sys_voltage *= (5.11/4.01f);
	}

//	printf("VREF ADC: %d, V: %.4f, VREFINT_CAL: %d, ADC: %d, V: %.4f\r\n", VREFINT_DATA, vref_int_voltage, VREFINT_CAL, uhADCxConvertedValue, ADC_sys_voltage);
//	printf("ADC V: %.4f\r\n", ADC_sys_voltage);

	return ADC_sys_voltage;
}

#define D3SRAM_ADDRESS        0x38000000
#define D3SRAM_DATA           0x30313233
#define D3SRAM_ERASE_DATA     0xFFFFFFFF
#define D3SRAM_SIZE           0x4000 /* 16 KB */
#define PWR_WAKEUP_PIN_FLAGS  (PWR_WAKEUP_FLAG1 | PWR_WAKEUP_FLAG2 | PWR_WAKEUP_FLAG3 | \
                               PWR_WAKEUP_FLAG4 | PWR_WAKEUP_FLAG5 | PWR_WAKEUP_FLAG6)
static void SRAM_WriteData()
{
  uint32_t SRAM_Index;

  /* Erase  D3 SRAM */
  for( SRAM_Index = 0; SRAM_Index < D3SRAM_SIZE ; SRAM_Index++)
  {
   *(__IO uint32_t*)(D3SRAM_ADDRESS + (SRAM_Index * 4)) = D3SRAM_ERASE_DATA ;
  }

  /* Fill  D3 SRAM */
  for( SRAM_Index = 0; SRAM_Index < D3SRAM_SIZE ; SRAM_Index++)
  {
   *(__IO uint32_t*)(D3SRAM_ADDRESS + (SRAM_Index * 4)) = D3SRAM_DATA;
  }
}

/**
  * @brief  Check data from D3 SRAM memory
  * @param  None
  * @retval None
  */
static void SRAM_CheckData()
{
  uint32_t SRAM_Index;

  /* Chedk  D3 SRAM data */
  for( SRAM_Index = 0; SRAM_Index < D3SRAM_SIZE ; SRAM_Index++)
  {
    if ( *(__IO uint32_t*)(D3SRAM_ADDRESS + (SRAM_Index * 4)) != D3SRAM_DATA)
    {
      Error_Handler();
    }
  }
}

static void CPU_CACHE_Clean(void)
{
  /* Clean D-Cache */
  SCB_CleanDCache();
}

#define RTC_ASYNCH_PREDIV    0x7F
#define RTC_SYNCH_PREDIV     0x0130
void RTC_Config(void)
{
  /* Configure RTC */
  RTCHandle.Instance = RTC;

  /* Set the RTC time base to 1s */
  /* Configure RTC prescaler and RTC data registers as follow:
    - Hour Format     = Format 24
    - Asynch Prediv   = Value according to source clock
    - Synch Prediv    = Value according to source clock
    - OutPut          = Output Disable
    - OutPutPolarity  = High Polarity
    - OutPutType      = Open Drain */
  RTCHandle.Init.HourFormat = RTC_HOURFORMAT_24;
  RTCHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
  RTCHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
  RTCHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
  RTCHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RTCHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if(HAL_RTC_Init(&RTCHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

void stop_mode1(void)
{
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
//	HAL_Delay(1);
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

	/* Check and handle if D1 domain was resumed from StandBy mode */
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB_D1) != RESET)
	{
		/* Check D3SRAM data */
		SRAM_CheckData();

		/* Clear Standby flag */
		__HAL_PWR_CLEAR_FLAG(PWR_CPU_FLAGS);
	}

	/* Configure RTC */
	RTC_Config();

	/* Insert 3 seconds delay */
//	HAL_Delay(3000);

	/* The Following Wakeup sequence is highly recommended prior to each Standby mode entry
	mainly  when using more than one wakeup source this is to not miss any wakeup event.
	- Disable all used wakeup sources,
	- Clear all related wakeup flags,
	- Re-enable all used wakeup sources,
	- Enter the Standby mode.
	 */
	/* Disable all used wakeup sources*/
	HAL_RTCEx_DeactivateWakeUpTimer(&RTCHandle);

	/* Clear all related wakeup flags */
	HAL_PWREx_ClearWakeupFlag(PWR_WAKEUP_PIN_FLAGS);

	/* Re-enable all used wakeup sources*/
	/* ## Setting the Wake up time ############################################*/
	/* RTC Wakeup Interrupt Generation:
	the wake-up counter is set to its maximum value to yield the longuest
	stand-by time to let the current reach its lowest operating point.
	The maximum value is 0xFFFF, corresponding to about 33 sec. when
	RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16

	Wakeup Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSI))
	Wakeup Time = Wakeup Time Base * WakeUpCounter
	  = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSI)) * WakeUpCounter
	  ==> WakeUpCounter = Wakeup Time / Wakeup Time Base

	To configure the wake up timer to 5s the WakeUpCounter is set to 0x2710:
	Wakeup Time Base = 16 /(~32.000KHz) = ~0.5 ms
	Wakeup Time = 0.5 ms  * WakeUpCounter
	Therefore, with wake-up counter =  0x2710  = 10,000
	   Wakeup Time =  0,5 ms *  10,000 = 5 sec. */
	HAL_RTCEx_SetWakeUpTimer_IT(&RTCHandle, 0x2710, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB_D1) == RESET)
	{
		/* Write D3SRAM data */
		SRAM_WriteData();

		/* Before entering Stanby mode make CPU cache clean operation to ensure that data are written in D3SRAM */
//		CPU_CACHE_Clean();

		/* Check written data */
		SRAM_CheckData();

		/* Set LED1 Off to inform the user that the system will enter STOP mode */
//		BSP_LED_Off(LED1);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

		/* Enter D2 Domain to Standby mode */
		HAL_PWREx_EnterSTANDBYMode(PWR_D2_DOMAIN);

		/* Enter D1 to Standby mode. In this case the system will enter STOP mode since D1
	   is in DSTANDBY mode, D2 is in DSTANDBY and D3 is in DSTOP mode */
		HAL_PWREx_EnterSTANDBYMode(PWR_D1_DOMAIN);
	}

	while(1)
	{
		printf("123\r\n");
	}
}

// sleep ms
void stop_mode1_duration(uint32_t ms)
{
	/* Check and handle if D1 domain was resumed from StandBy mode */
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB_D1) != RESET)
	{
		/* Check D3SRAM data */
//		SRAM_CheckData();

		/* Clear Standby flag */
		__HAL_PWR_CLEAR_FLAG(PWR_CPU_FLAGS);
	}

	/* Configure RTC */
	RTC_Config();

	/* Insert 3 seconds delay */
//	HAL_Delay(3000);

	/* The Following Wakeup sequence is highly recommended prior to each Standby mode entry
	mainly  when using more than one wakeup source this is to not miss any wakeup event.
	- Disable all used wakeup sources,
	- Clear all related wakeup flags,
	- Re-enable all used wakeup sources,
	- Enter the Standby mode.
	 */
	/* Disable all used wakeup sources*/
	HAL_RTCEx_DeactivateWakeUpTimer(&RTCHandle);

	/* Clear all related wakeup flags */
	HAL_PWREx_ClearWakeupFlag(PWR_WAKEUP_PIN_FLAGS);

	/* Re-enable all used wakeup sources*/
	/* ## Setting the Wake up time ############################################*/
	/* RTC Wakeup Interrupt Generation:
	the wake-up counter is set to its maximum value to yield the longuest
	stand-by time to let the current reach its lowest operating point.
	The maximum value is 0xFFFF, corresponding to about 33 sec. when
	RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16

	Wakeup Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSI))
	Wakeup Time = Wakeup Time Base * WakeUpCounter
	  = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSI)) * WakeUpCounter
	  ==> WakeUpCounter = Wakeup Time / Wakeup Time Base

	To configure the wake up timer to 5s the WakeUpCounter is set to 0x2710:
	Wakeup Time Base = 16 /(~32.000KHz) = ~0.5 ms
	Wakeup Time = 0.5 ms  * WakeUpCounter
	Therefore, with wake-up counter =  0x2710  = 10,000
	   Wakeup Time =  0,5 ms *  10,000 = 5 sec. */
	uint32_t counts = ms * (0x2710 / 5000);
	HAL_RTCEx_SetWakeUpTimer_IT(&RTCHandle, counts, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB_D1) == RESET)
	{
		/* Write D3SRAM data */
//		SRAM_WriteData();

		/* Before entering Stanby mode make CPU cache clean operation to ensure that data are written in D3SRAM */
//		CPU_CACHE_Clean();

		/* Check written data */
//		SRAM_CheckData();

		/* Set LED1 Off to inform the user that the system will enter STOP mode */
//		BSP_LED_Off(LED1);
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

		/* Enter D2 Domain to Standby mode */
		HAL_PWREx_EnterSTANDBYMode(PWR_D2_DOMAIN);

		/* Enter D1 to Standby mode. In this case the system will enter STOP mode since D1
	   is in DSTANDBY mode, D2 is in DSTANDBY and D3 is in DSTOP mode */
		HAL_PWREx_EnterSTANDBYMode(PWR_D1_DOMAIN);
	}

	while(1)
	{
		printf("123\r\n");
	}
}

void stop_mode2(void)
{
	/* Configure the RTC */
	RTC_Config();

//	while (1)
//	{
		// read ADC voltage
//		adc_ref_internal_read();
		/* Turn LED1 on */
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

		/* Insert 5 second delay */
		HAL_Delay(1);

		/* Disable Wakeup Counter */
		HAL_RTCEx_DeactivateWakeUpTimer(&RTCHandle);

		/* ## Setting the Wake up time ############################################*/
		/*  RTC Wakeup Interrupt Generation:
		  Wakeup Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSE or LSI))
		  Wakeup Time = Wakeup Time Base * WakeUpCounter
		  = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSE or LSI)) * WakeUpCounter
			==> WakeUpCounter = Wakeup Time / Wakeup Time Base

		  To configure the wake up timer to 20s the WakeUpCounter is set to 0x9C40:
		  RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16
		  Wakeup Time Base = 16 /(32.000KHz) = 0,5 ms
		  Wakeup Time = 20s = 0,5ms  * WakeUpCounter
		==> WakeUpCounter = 20s/0,5ms = 40000 = 0x9C40 */
		// 5s
//		HAL_RTCEx_SetWakeUpTimer_IT(&RTCHandle, 0x2710, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
		HAL_RTCEx_SetWakeUpTimer_IT(&RTCHandle, 0x9C40, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

		/* Turn LED1 OFF  */
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

		/* Enter Stop Mode */
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
		/* Configures system clock after wake-up from STOP: enable HSE, PLL and select
			PLL as system clock source (HSE and PLL are disabled in STOP mode) */
		//	SYSCLKConfig_STOP();
		/* Configure the system clock */
		SystemClock_Config();

		/* Configure the peripherals common clocks */
		PeriphCommonClock_Config();
//	}
}

// sleep ms
void stop_mode2_duration(uint32_t ms)
{
	/* Configure the RTC */
	RTC_Config();

	/* Disable Wakeup Counter */
	HAL_RTCEx_DeactivateWakeUpTimer(&RTCHandle);

	/* ## Setting the Wake up time ############################################*/
	/*  RTC Wakeup Interrupt Generation:
	  Wakeup Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSE or LSI))
	  Wakeup Time = Wakeup Time Base * WakeUpCounter
	  = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSE or LSI)) * WakeUpCounter
		==> WakeUpCounter = Wakeup Time / Wakeup Time Base

	  To configure the wake up timer to 20s the WakeUpCounter is set to 0x9C40:
	  RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16
	  Wakeup Time Base = 16 /(32.000KHz) = 0,5 ms
	  Wakeup Time = 20s = 0,5ms  * WakeUpCounter
	==> WakeUpCounter = 20s/0,5ms = 40000 = 0x9C40 */
	// 5s
	uint32_t counts = ms * (0x2710 / 5000);
//	HAL_RTCEx_SetWakeUpTimer_IT(&RTCHandle, 0x2710, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
	HAL_RTCEx_SetWakeUpTimer_IT(&RTCHandle, counts, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

	/* Enter Stop Mode */
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	/* Configures system clock after wake-up from STOP: enable HSE, PLL and select
		PLL as system clock source (HSE and PLL are disabled in STOP mode) */
	//	SYSCLKConfig_STOP();
	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
//	HAL_Delay(10);
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
//	adc_read();

//	adc_ref_internal_read();
//}
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
