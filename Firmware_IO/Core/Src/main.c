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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LCD.h"

#include "fatfs_sd.h"

#include "string.h"
#include "stdio.h"
//DHCP
#include "socket.h"
#include "wizchip_conf.h"
#include "dhcp.h"
#include "mb.h"
#include "W5500_config.h"

#include "structer.h"
#include "io.h"
#include "screen.h"
#include "timer.h"
#include "math.h"
#include "rtc.h"
#include "flash.h"
#include "unittest.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define CDC_DEBUG
#define TIME_WAIT 100

#define MEASUREMENT_1		1
#define MEASUREMENT_2		2

#define SHOW_HIS			1
#define NOT_SHOW_HIS		0

#define MEASUREMENT_1_FILE_NAME		"measurement1.csv"
#define MEASUREMENT_2_FILE_NAME		"measurement2.csv"

#define EMPTY			-1 //this is the value return when at address of flash is empty data, this must be modify by other compiler



#define CDC_LCD_DEBUG
#ifdef CDC_LCD_DEBUG
#define	DBG_LCD(x,format,variable) 	 do(uint8_t buf[100]; sprintf(buf,x format,variable);  CDC_Transmit_FS(buf,sizeof(buf));)while(0);
#else
	DBG_LCD(x)
#endif //CDC_LCD_DEBUG
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define false 	0
#define true 	1
/*----------------------------DHCP - MODBUS-----------------------------*/
volatile uint8_t ip_assigned = false;

uint16_t usRegInputBuf[REG_INPUT_NREGS] = { 0 };
uint16_t usRegHoldingBuf[REG_HOLDING_NREGS] = { 0 };
uint8_t ucRegCoilsBuf[REG_COILS_SIZE] = { 0 };

/*---------------------------------------------------*/

/*----------------app.c----------------*/
static dataMeasure mdata = { { 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0 } };

uint8_t menuScreenFlag = 0;
wiz_NetInfo net_info = { .mac = { 0xEA, 0x11, 0x22, 0x33, 0x44, 0xEA }, .dhcp =
		NETINFO_DHCP };

volatile static uint8_t mainScreenFlag = 0; //use for reset data calib
volatile static button mbutton;
volatile static input minput;
volatile static sensor msensor;
volatile static output moutput;
volatile static ledStatus mledStatus;
MeasureValue mcalibValue;
MeasureValue mmeasureValue;

volatile static setCalibValue msetCalibValue_1; //for measurement1
volatile static setCalibValue msetCalibValue_2; //for measurement2
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void Callback_IPAssigned(void) {
	ip_assigned = true;
}
void Callback_IPConflict(void) {

}


static void write_SDCard(dataMeasure data, char *fileName);
static dataMeasure read_SDCard(char *fileName, uint8_t lineIndex);

static void W5500_init();

/*---------------app.c------------*/
static void app_SettingRtc(void);
//static void app_SettingData(void);
//static void app_GetEeprom(void);
static void app_Measurement_1(void);
static void app_Measurement_2(void);
static void app_SetCalibValue(uint8_t measurementIndex);
static void app_GetCalibValue(uint8_t measurementIndex);

static void app_HisValue(uint8_t measurementIndex);
static void app_ClearAllOutput(void);

static optionScreen_e_t app_optionMenu(void);
static void app_processOptionMenu(optionScreen_e_t optionMenu);
static void app_ShowIP(void);
static void app_Init(void);
static void app_GotoMainScreen(uint8_t option, uint8_t measurementIndex);
static void app_SettingVDLRZ(void);
static void updateMBRegister(void);

/*----------cycle measurement function---------*/
CycleMeasure meas_checkSensor(CycleMeasure cycleMeasure, uint8_t measurementIndex);
CycleMeasure meas_measurementZ(CycleMeasure cycleMeasure, uint8_t measurementIndex);
CycleMeasure meas_measurementX1Y1(CycleMeasure cycleMeasure, uint8_t measurementIndex);
CycleMeasure meas_measurementX2Y2(CycleMeasure cycleMeasure, uint8_t measurementIndex);

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
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim6); //use for delay_us
	HAL_TIM_Base_Start_IT(&htim1); //timer interrupt every 100us
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);

#if 0
	for (int  i = 0;  i < 10; ++ i) {


	dataMeasure _data,__data;
	_data.coordinates.R = 100;
	_data.coordinates.X = -100;
	_data.coordinates.Y = -1000;
	_data.coordinates.Z = -2000;
	_data.coordinates.aX = 2000;
	_data.coordinates.aY = 4000;
	_data.time.day = 10;
	_data.time.hour = 20;
	_data.time.minute = 40;
	_data.time.month = 9;
	_data.time.year = 20;
	_data.mode = 4;

	write_SDCard(_data, MEASUREMENT_1_FILE_NAME);
//	process_SD_Card(_data, MEASUREMENT_1_FILE_NAME);
	__data = read_SDCard(MEASUREMENT_1_FILE_NAME, 0);
//	app_GotoMainScreen(CALIBSET, MEASUREMENT_1);
	screen_DataMeasureType1(__data, CALIBSET, MEASUREMENT_1, NOT_SHOW_HIS);
	HAL_Delay(500);
	dataMeasure ___data,____data;
	___data.coordinates.R = 100;
	___data.coordinates.X = -56;
	___data.coordinates.Y = -14;
	___data.coordinates.Z = -2000;
	___data.coordinates.aX = 2000;
	___data.coordinates.aY = 1200;
	___data.time.day = 10;
	___data.time.hour = 21;
	___data.time.minute = 40;
	___data.time.month = 9;
	___data.time.year = 30;
	___data.mode = 4;

	write_SDCard(___data, MEASUREMENT_1_FILE_NAME);
//	process_SD_Card(__data, MEASUREMENT_1_FILE_NAME);
	____data = read_SDCard(MEASUREMENT_1_FILE_NAME, 0);
	screen_DataMeasureType1(____data, CALIBSET, MEASUREMENT_1, NOT_SHOW_HIS);
	}
	while(1);
#endif


#if 1
	unitTestZ();
#endif
	app_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		minput = io_getInput();
		mbutton = io_getButton();
		uint32_t _time = 0;
		if (menuScreenFlag) {
			menuScreenFlag = 0;
			app_processOptionMenu(app_optionMenu());
		}

		if (_ON == minput.in0) {
			minput.in0 = _OFF;
			app_Measurement_1(); //measurement 1
		}

		if (_ON == minput.in1) {
			minput.in1 = _OFF;
			app_Measurement_2(); //measurement 2
		}

		if (_ON == mbutton.reset) { //reset datacalib
			mbutton.reset = _OFF;
			timer_Start(TIMER_CLEARCALIB, TIMERMAXVALUE);
			do {
				/* code */
			} while (_ON == io_getButton().reset);
			_time = time_Stop(TIMER_CLEARCALIB);

			if ((_time >= 100000 /*10sec*/)) {
				MeasureValue vl = { 0, 0, 0, 0, 0 };
				DBG("Clear DataCalib by RESET button\n");
				if(mainScreenFlag == MEASUREMENT_1)
				{
					FLASH_WriteDataCalib(&vl, MEASUREMENT_1);
					mledStatus.led1= _OFF;
					io_setLedStatus(mledStatus, ucRegCoilsBuf);
					msetCalibValue_1 = CALIBRESET;
					app_GotoMainScreen(CALIBRESET, MEASUREMENT_1);
				}
				else
				{
					FLASH_WriteDataCalib(&vl, MEASUREMENT_2);
					mledStatus.led2= _OFF;
					io_setLedStatus(mledStatus, ucRegCoilsBuf);
					msetCalibValue_2 = CALIBRESET;
					app_GotoMainScreen(CALIBRESET, MEASUREMENT_2);
				}

			}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

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
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
#if 0
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x10;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x1;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x1;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
	sTime.Hours = mtime.hour;
	sTime.Minutes = mtime.minute;
	sTime.Seconds = 0x0;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
		Error_Handler();
	}
	sDate.WeekDay = RTC_WEEKDAY_MONDAY; //dont care
	sDate.Month = mtime.month;
	sDate.Date = mtime.day;
	sDate.Year = mtime.year;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK) {
		Error_Handler();
	}
#endif
  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
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
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_FORCED_INACTIVE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 47;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|OUT3_Pin
                          |OUT4_Pin|OUT5_Pin|OUT6_Pin|OUT7_Pin
                          |GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, W5500_RS_Pin|LED1_Pin|LED2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUT0_Pin|OUT1_Pin|OUT2_Pin|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE4 PE5 PE6
                           OUT3_Pin OUT4_Pin OUT5_Pin OUT6_Pin
                           OUT7_Pin PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |OUT3_Pin|OUT4_Pin|OUT5_Pin|OUT6_Pin
                          |OUT7_Pin|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : W5500_CS_Pin */
  GPIO_InitStruct.Pin = W5500_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(W5500_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : W5500_INT_Pin BT_SET_Pin BT_RESERVED_Pin */
  GPIO_InitStruct.Pin = W5500_INT_Pin|BT_SET_Pin|BT_RESERVED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : W5500_RS_Pin */
  GPIO_InitStruct.Pin = W5500_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(W5500_RS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT0_Pin OUT1_Pin OUT2_Pin PB8
                           PB9 */
  GPIO_InitStruct.Pin = OUT0_Pin|OUT1_Pin|OUT2_Pin|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR0_Pin SENSOR1_Pin */
  GPIO_InitStruct.Pin = SENSOR0_Pin|SENSOR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : IN7_Pin IN6_Pin IN5_Pin IN4_Pin */
  GPIO_InitStruct.Pin = IN7_Pin|IN6_Pin|IN5_Pin|IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IN3_Pin BT_PREV_Pin BT_NEXT_Pin BT_MENU_Pin
                           BT_RESET_Pin */
  GPIO_InitStruct.Pin = IN3_Pin|BT_PREV_Pin|BT_NEXT_Pin|BT_MENU_Pin
                          |BT_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : IN2_Pin IN1_Pin IN0_Pin */
  GPIO_InitStruct.Pin = IN2_Pin|IN1_Pin|IN0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void W5500_init() {
	uint8_t dns[4];
	uint8_t dhcp_buffer[1024];
	static UCHAR Vendor[11] = "Modbus TCP";
	uint8_t rx_tx_buff_sizes[] = { 2, 2, 2, 2, 2, 2, 2, 2 };

	reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
	reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
	reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);
	wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);

	setSHAR(net_info.mac);

	DHCP_init(DHCP_SOCKET, dhcp_buffer);

	reg_dhcp_cbfunc(Callback_IPAssigned, Callback_IPAssigned,
			Callback_IPConflict);

	uint32_t current = HAL_GetTick();
	//get IP assigned
	while (HAL_GetTick() - current < 1000) {
		DHCP_run();
	}

	if (!ip_assigned) {
		return;
	}

	getIPfromDHCP(net_info.ip);
	getGWfromDHCP(net_info.gw);
	getSNfromDHCP(net_info.sn);
	getDNSfromDHCP(dns);

	wizchip_setnetinfo(&net_info);
	HAL_Delay(200);

	if (eMBTCPInit(MBTCP_PORT) != MB_ENOERR) {
		return;
	}

	if (eMBSetSlaveID(SLAVE_ID, true, Vendor, sizeof(Vendor)) != MB_ENOERR) {
		return;
	}

	if (eMBEnable() != MB_ENOERR) {
		return;
	}
}
eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress,
		USHORT usNRegs, eMBRegisterMode eMode) {
	eMBErrorCode eStatus = MB_ENOERR;
	int iRegIndex;

	if ((usAddress >= REG_HOLDING_START)
			&& (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS)) {
		iRegIndex = (int) (usAddress - REG_HOLDING_START);
		switch (eMode) {
		/* Pass current register values to the protocol stack. */
		case MB_REG_READ:
			while (usNRegs > 0) {
				*pucRegBuffer++ = (unsigned char) (usRegHoldingBuf[iRegIndex]
						>> 8);
				*pucRegBuffer++ = (unsigned char) (usRegHoldingBuf[iRegIndex]
						& 0xFF);
				iRegIndex++;
				usNRegs--;
			}
			break;
			/* Update current register values with new values from the protocol stack. */
		case MB_REG_WRITE:
			while (usNRegs > 0) {
				usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
				usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
				iRegIndex++;
				usNRegs--;
			}
		}
	} else {
		eStatus = MB_ENOREG;
	}
	return eStatus;
}

eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress,
		USHORT usNRegs) {
	eMBErrorCode eStatus = MB_ENOERR;
	int iRegIndex;

	if ((usAddress >= REG_INPUT_START)
			&& (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS)) {
		iRegIndex = (int) (usAddress - REG_INPUT_START);
		while (usNRegs > 0) {
			*pucRegBuffer++ = (unsigned char) (usRegInputBuf[iRegIndex] >> 8);
			*pucRegBuffer++ = (unsigned char) (usRegInputBuf[iRegIndex] & 0xFF);
			iRegIndex++;
			usNRegs--;
		}
	} else {
		eStatus = MB_ENOREG;
	}

	return eStatus;
}

eMBErrorCode eMBRegCoilsCB(UCHAR *pucRegBuffer, USHORT usAddress,
		USHORT usNCoils, eMBRegisterMode eMode) {
	eMBErrorCode eStatus = MB_ENOERR;
	int iNCoils = (int) usNCoils;
	unsigned short usBitOffset;

	/* Check if we have registers mapped at this block. */
	if ((usAddress >= REG_COILS_START)
			&& (usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE)) {
		usBitOffset = (unsigned short) (usAddress - REG_COILS_START);
		switch (eMode) {
		/* Read current values and pass to protocol stack. */
		case MB_REG_READ:
			while (iNCoils > 0) {
				*pucRegBuffer++ = xMBUtilGetBits(ucRegCoilsBuf, usBitOffset,
						(unsigned char) (iNCoils > 8 ? 8 : iNCoils));
				iNCoils -= 8;
				usBitOffset += 8;
			}
			break;
			/* Update current register values. */
		case MB_REG_WRITE:
			while (iNCoils > 0) {
				xMBUtilSetBits(ucRegCoilsBuf, usBitOffset,
						(unsigned char) (iNCoils > 8 ? 8 : iNCoils),
						*pucRegBuffer++);
				iNCoils -= 8;
				usBitOffset += 8;
			}
			break;
		}
	} else {
		eStatus = MB_ENOREG;
	}
	return eStatus;
}

eMBErrorCode eMBRegDiscreteCB(UCHAR *pucRegBuffer, USHORT usAddress,
		USHORT usNDiscrete) {
#if 0 ////dont use this register
	eMBErrorCode eStatus = MB_ENOERR;
	short iNDiscrete = (short) usNDiscrete;
	unsigned short usBitOffset;

	/* Check if we have registers mapped at this block. */
	if ((usAddress >= REG_DISCRETE_START)
			&& (usAddress + usNDiscrete
					<= REG_DISCRETE_START + REG_DISCRETE_SIZE)) {
		usBitOffset = (unsigned short) (usAddress - REG_DISCRETE_START);
		while (iNDiscrete > 0) {
			*pucRegBuffer++ = xMBUtilGetBits(ucRegDiscreteBuf, usBitOffset,
					(unsigned char) (iNDiscrete > 8 ? 8 : iNDiscrete));
			iNDiscrete -= 8;
			usBitOffset += 8;
		}
	} else {
		eStatus = MB_ENOREG;
	}
	return eStatus;
#else
	return MB_ENOERR;
#endif
}

static void write_SDCard(dataMeasure data, char *fileName) {
	FATFS FatFs;
	FIL fil;
	char buff[110];
	unsigned int BytesWr;
	if(f_mount(&FatFs, "", 0) != FR_OK)//mount SD card
		return;
#if 0	//turn on this macro if you want to check the free space of SD card
		DWORD fre_clust;
		uint32_t totalSpace;
		uint32_t freeSpace;

		if (f_getfree("", &fre_clust, &FatFs) != FR_OK) {
			break;
		}
		totalSpace = (uint32_t) ((FatFs->n_fatent - 2) * FatFs->csize * 0.5);
		freeSpace = (uint32_t) (fre_clust * FatFs->csize * 0.5);
#endif
	if (f_open(&fil, fileName, FA_READ | FA_OPEN_ALWAYS | FA_WRITE) == FR_OK) //In this mode, it will create the file if file not existed
			{
		sprintf(buff,
				"20%02u/%02u/%02u - %02u:%02u,%hi,%hi,%hi,%hi,%hi,%hi,%u\n",
				data.time.year, data.time.month, data.time.day, data.time.hour,
				data.time.minute, data.coordinates.R, data.coordinates.X,
				data.coordinates.Y, data.coordinates.Z, data.coordinates.aX,
				data.coordinates.aY, data.mode);
		f_lseek(&fil, f_size(&fil));
		f_write(&fil, buff, strlen(buff), &BytesWr);
		f_close(&fil);

	}
	f_mount(NULL, "", 0);
}

static dataMeasure read_SDCard(char *fileName, uint8_t lineIndex) {
	FATFS FatFs;
	FIL fil;
	char buff[80];
	dataMeasure data = {0};
	
	unsigned int totalLines = 0;

	if(f_mount(&FatFs, "", 1) != FR_OK) //mount SD card
		return data;
	if(f_open(&fil, fileName, FA_READ)!= FR_OK)
		return data;

	// Count the total number of lines in the file
    while (f_gets(buff, 100, &fil) != FR_OK) {
    	totalLines++;
    }

    f_lseek(&fil, 0); // move pointer to beginning of file

    while(totalLines - lineIndex > 0)
    {
    	totalLines --;
    	f_gets(buff, sizeof(buff), &fil);
    }
    f_close(&fil);
    f_mount(NULL, "", 0);
//    uint8_t itemparse = sscanf(buff, "20%hhi-%hhi-%hhi %hhi:%hhi,%hi,%hi,%hi,%hi,%hi,%hi,%hhi\n", &data->time.year, &data->time.month, &data->time.day, &data->time.hour,
//			&data->time.minute, &data->coordinates.R, &data->coordinates.X,
//			&data->coordinates.Y, &data->coordinates.Z, &data->coordinates.aX,
//			&data->coordinates.aY, &data->mode);
    unsigned short int  year,month,day,hour,minute;
//    char _buff[] = "2021-7-24 12:34,100,200,300,400,500,600,1";
    char *token = strtok(buff, ",");
    sscanf(token, "20%hu/%hu/%hu - %hu:%hu", &year, &month, &day, &hour,
    			&minute);

    data.time.day = (uint8_t)day;
    data.time.hour = (uint8_t)hour;
    data.time.minute = (uint8_t)minute;
    data.time.month = (uint8_t)month;
    data.time.year = (uint8_t)year;

	token = strtok(NULL, ",");

	sscanf(token, "%hi", &data.coordinates.R);
	token = strtok(NULL, ",");

	sscanf(token, "%hi", &data.coordinates.X);
	token = strtok(NULL, ",");

	sscanf(token, "%hi", &data.coordinates.Y);
	token = strtok(NULL, ",");

	sscanf(token, "%hi", &data.coordinates.Z);
	token = strtok(NULL, ",");

	sscanf(token, "%hi", &data.coordinates.aX);
	token = strtok(NULL, ",");

	sscanf(token, "%hi", &data.coordinates.aY);
	token = strtok(NULL, ",");

	sscanf(token, "%hhu", &data.mode);
	return data;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) { //should check
	if (htim == &htim1) {
		uint8_t i;
		for (i = 0; i < MAX_TIME; i++) {
			if (TIME_RUN == mtimer[i].status) {
				mtimer[i].inc += 1;
				if (mtimer[i].inc >= mtimer[i].count) {
					mtimer[i].status = TIME_FINISH;
					mtimer[i].flat = 1;
				}
			}
		}

//		updateMBRegister(); //update modbus register every 100us
	}
}
static void app_SettingVDLRZ(void) {
	VDRLZ_CycleSet cycle = V_set;
	uint8_t exit = 0;
	VDRLZ_Input buffer = FLASH_ReadVDRLZ();

	do {
		if (cycle == V_set) {
			screen_setVDRLZ(buffer, V_set);
			do {
				mbutton = io_getButton();
				if (_ON == mbutton.set) {
					while (_ON == io_getButton().set)
						;
					buffer.V += 1;
					screen_setVDRLZ(buffer, V_set);
					HAL_Delay(TIME_WAIT);
				}
				if (_ON == mbutton.reset) {
					while (_ON == io_getButton().reset)
						;
					buffer.V -= 1;
					screen_setVDRLZ(buffer, V_set);
					HAL_Delay(TIME_WAIT);
				}
				if (_ON == mbutton.next) {
					cycle = D_set;
					while (_ON == io_getButton().next)
						;
				}
				if (_ON == mbutton.menu) {
					while (_ON == io_getButton().menu)
						;
					exit = 1;
					break;
				}
			} while (cycle == V_set);
		}
		if (cycle == D_set) {
			screen_setVDRLZ(buffer, D_set);
			do {
				mbutton = io_getButton();
				if (_ON == mbutton.set) {
					while (_ON == io_getButton().set)
						;
					buffer.D += 1;
					screen_setVDRLZ(buffer, D_set);
					HAL_Delay(TIME_WAIT);
				}
				if (_ON == mbutton.reset) {
					while (_ON == io_getButton().reset)
						;
					buffer.D -= 1;
					screen_setVDRLZ(buffer, D_set);
					HAL_Delay(TIME_WAIT);
				}
				if (_ON == mbutton.next) {
					cycle = L_set;
					while (_ON == io_getButton().next)
						;
				}
				if (_ON == mbutton.prev) {
					cycle = V_set;
					while (_ON == io_getButton().prev)
						;
				}
				if (_ON == mbutton.menu) {
					while (_ON == io_getButton().menu)
						;
					exit = 1;
					break;
				}
			} while (cycle == D_set);
		}
		if (cycle == R_set) {
			screen_setVDRLZ(buffer, R_set);
			do {
				mbutton = io_getButton();
				if (_ON == mbutton.set) {
					while (_ON == io_getButton().set)
						;
					buffer.R += 0.1;
					screen_setVDRLZ(buffer, R_set);
					HAL_Delay(TIME_WAIT);
				}
				if (_ON == mbutton.reset) {
					while (_ON == io_getButton().reset)
						;
					buffer.R -= 0.1;
					screen_setVDRLZ(buffer, R_set);
					HAL_Delay(TIME_WAIT);
				}
				if (_ON == mbutton.next) {
					cycle = Z_set;
					while (_ON == io_getButton().next)
						;
				}
				if (_ON == mbutton.prev) {
					cycle = L_set;
					while (_ON == io_getButton().prev)
						;
				}
				if (_ON == mbutton.menu) {
					while (_ON == io_getButton().menu)
						;
					exit = 1;
					break;
				}
			} while (cycle == R_set);
		}
		if (cycle == L_set) {
			screen_setVDRLZ(buffer, L_set);
			do {
				mbutton = io_getButton();
				if (_ON == mbutton.set) {
					while (_ON == io_getButton().set)
						;
					buffer.L += 1;
					screen_setVDRLZ(buffer, L_set);
					HAL_Delay(TIME_WAIT);
				}
				if (_ON == mbutton.reset) {
					while (_ON == io_getButton().reset)
						;
					buffer.L -= 1;
					screen_setVDRLZ(buffer, L_set);
					HAL_Delay(TIME_WAIT);
				}
				if (_ON == mbutton.next) {
					cycle = R_set;
					while (_ON == io_getButton().next)
						;
				}
				if (_ON == mbutton.prev) {
					cycle = D_set;
					while (_ON == io_getButton().prev)
						;
				}
				if (_ON == mbutton.menu) {
					while (_ON == io_getButton().menu)
						;
					exit = 1;
					break;
				}
			} while (cycle == L_set);
		}
		if (cycle == Z_set) {
			screen_setVDRLZ(buffer, Z_set);
			do {
				mbutton = io_getButton();
				if (_ON == mbutton.set) {
					while (_ON == io_getButton().set)
						;
					buffer.Z += 0.1;
					screen_setVDRLZ(buffer, Z_set);
					HAL_Delay(TIME_WAIT);
				}
				if (_ON == mbutton.reset) {
					while (_ON == io_getButton().reset)
						;
					buffer.Z -=0.1;
					screen_setVDRLZ(buffer, Z_set);
					HAL_Delay(TIME_WAIT);
				}
				if (_ON == mbutton.prev) {
					cycle = R_set;
					while (_ON == io_getButton().prev)
						;
				}
				if (_ON == mbutton.menu) {
					while (_ON == io_getButton().menu)
						;
					exit = 1;
					break;
				}
			} while (cycle == Z_set);
		}
	} while (exit == 0);
	FLASH_WriteVDRLZ(&buffer);
	app_GotoMainScreen(msetCalibValue_1, MEASUREMENT_1); // main screen
}

static void app_SettingRtc(void) {
	CycleTime cycle = SET_YEAR;
	uint8_t exit = 0;
	Time mtime = rtc_Now();
	LCD_Clear();
	do {
		/*set year*/
		if (SET_YEAR == cycle) {
			screen_setDateTime(mtime, SET_YEAR);
			do {
				mbutton = io_getButton();
				if (_ON == mbutton.set) {
					while (_ON == io_getButton().set)
						;
					mtime.year++;
					if (99 < mtime.year)
						mtime.year = 0;
					screen_setDateTime(mtime, SET_YEAR);
					HAL_Delay(TIME_WAIT);
				}
				if (_ON == mbutton.next) {
					cycle = SET_MONTH;
					while (_ON == io_getButton().next)
						;
				}
				if (_ON == mbutton.reset) {
					while (_ON == io_getButton().reset)
						;
					mtime.year--;
					if (99 < mtime.year)
						mtime.year = 99;
					screen_setDateTime(mtime, SET_YEAR);
					HAL_Delay(TIME_WAIT);
				}

				if (_ON == mbutton.menu) {
					while (_ON == io_getButton().menu)
						;
					exit = 1;
					break;
				}
			} while (SET_YEAR == cycle);
		}
		/*set month*/
		if (SET_MONTH == cycle) {
			screen_setDateTime(mtime, SET_MONTH);
			do {
				mbutton = io_getButton();
				if (_ON == mbutton.set) {
					while (_ON == io_getButton().set)
						;
					mtime.month++;
					if (12 < mtime.month)
						mtime.month = 1;
					screen_setDateTime(mtime, SET_MONTH);
					HAL_Delay(TIME_WAIT);
				}
				if (_ON == mbutton.next) {
					cycle = SET_DAY;
					while (_ON == io_getButton().next)
						;
				}
				if (_ON == mbutton.reset) {
					while (_ON == io_getButton().reset)
						;
					mtime.month--;
					if (12 < mtime.month)
						mtime.month = 12;
					screen_setDateTime(mtime, SET_MONTH);
					HAL_Delay(TIME_WAIT);
				}
				if (_ON == mbutton.prev) {
					while (_ON == io_getButton().prev)
						;
					cycle = SET_YEAR;
				}
				if (_ON == mbutton.menu) {
					while (_ON == io_getButton().menu)
						;
					exit = 1;
					break;
				}
			} while (SET_MONTH == cycle);
		}
		/*set day*/
		if (SET_DAY == cycle) {
			screen_setDateTime(mtime, SET_DAY);
			do {
				mbutton = io_getButton();
				if (_ON == mbutton.set) {
					while (_ON == io_getButton().set)
						;
					mtime.day ++;
					if (31 < mtime.day)
						mtime.day = 1;
					screen_setDateTime(mtime, SET_DAY);
					HAL_Delay(TIME_WAIT);
				}
				if (_ON == mbutton.next) {
					cycle = SET_HOUR;
					while (_ON == io_getButton().next)
						;
				}
				if (_ON == mbutton.reset) {
					while (_ON == io_getButton().reset)
						;
					mtime.day--;
					if (31 < mtime.day)
						mtime.day = 31;
					screen_setDateTime(mtime, SET_DAY);
					HAL_Delay(TIME_WAIT);
				}
				if (_ON == mbutton.prev) {
					while (_ON == io_getButton().prev)
						;
					cycle = SET_MONTH;
				}
				if (_ON == mbutton.menu) {
					while (_ON == io_getButton().menu)
						;
					exit = 1;
					break;
				}
			} while (SET_DAY == cycle);
		}
		/*set hour*/
		if (SET_HOUR == cycle) {
			screen_setDateTime(mtime, SET_HOUR);
			do {
				mbutton = io_getButton();
				if (_ON == mbutton.set) {
					while (_ON == io_getButton().set)
						;
					mtime.hour += 1;
					if (23 < mtime.hour)
						mtime.hour = 0;

					screen_setDateTime(mtime, SET_HOUR);
					HAL_Delay(TIME_WAIT);
				}
				if (_ON == mbutton.next) {
					cycle = SET_MINUTE;
					while (_ON == io_getButton().next)
						;
				}

				if (_ON == mbutton.reset) {
					while (_ON == io_getButton().reset)
						;
					mtime.hour--;
					if (23 < mtime.hour)
						mtime.hour = 23;
					screen_setDateTime(mtime, SET_HOUR);
					HAL_Delay(TIME_WAIT);
				}
				if (_ON == mbutton.prev) {
					while (_ON == io_getButton().prev)
						;
					cycle = SET_DAY;
				}
				if (_ON == mbutton.menu) {
					while (_OFF == io_getButton().menu)
						;
					exit = 1;
					break;
				}
			} while (SET_HOUR == cycle);
		}
		/*set minute*/
		if (SET_MINUTE == cycle) {
			screen_setDateTime(mtime, SET_MINUTE);
			do {
				mbutton = io_getButton();
				if (_ON == mbutton.set) {
					while (_ON == io_getButton().set)
						;
					mtime.minute++;
					if (59 < mtime.minute)
						mtime.minute = 1;
					mtime.second = 0;
					screen_setDateTime(mtime, SET_MINUTE);
					HAL_Delay(TIME_WAIT);
				}
				if (_ON == mbutton.prev) {
					cycle = SET_HOUR;
					while (_ON == io_getButton().prev)
						;
				}
				if (_ON == mbutton.reset) {
					while (_ON == io_getButton().reset)
						;
					mtime.minute--;
					if (59 < mtime.minute)
						mtime.minute = 59;
					mtime.second = 0;
					screen_setDateTime(mtime, SET_MINUTE);
					HAL_Delay(TIME_WAIT);
				}
				if (_ON == mbutton.menu) {
					while (_ON == io_getButton().menu)
						;
					exit = 1;
					break;
				}
			} while (SET_MINUTE == cycle);
		}
	} while (0 == exit);
	rtc_SetDateTime(mtime);
//	dataMeasure temp = read_SDCard(MEASUREMENT_1_FILE_NAME, 0);
	app_GotoMainScreen(msetCalibValue_1, MEASUREMENT_1); // main screen
}

static void app_Measurement_1(void) {
	volatile CycleMeasure cycleMeasure = STOP;

	mdata.coordinates.X = 0;
	mdata.coordinates.Y = 0;
	mdata.coordinates.Z = 0;
	mdata.coordinates.aX = 0;
	mdata.coordinates.aY = 0;
	mdata.mode = NONE;

	mmeasureValue.X1 = 0;
	mmeasureValue.Y1 = 0;
	mmeasureValue.X2 = 0;
	mmeasureValue.Y2 = 0;
	mmeasureValue.Z = 0;

	DBG("START MEASUREMENT 1\r\n");

	cycleMeasure = meas_checkSensor(cycleMeasure, MEASUREMENT_1);
	if (cycleMeasure == _ERROR) {
		DBG("cycleMeasure == _ERROR");
	}
	cycleMeasure = meas_measurementZ(cycleMeasure, MEASUREMENT_1);
	cycleMeasure = meas_measurementX1Y1(cycleMeasure, MEASUREMENT_1);
	cycleMeasure = meas_measurementX2Y2(cycleMeasure, MEASUREMENT_1);

	while (cycleMeasure == _ERROR_XY) {
		DBG("Sensor X/Y Error");
		uint32_t _time = 0;
		mledStatus.led1 = _ON;
		mledStatus.led2 = _ON;
		io_setLedStatus(mledStatus, ucRegCoilsBuf);
		HAL_Delay(500);
		mledStatus.led1 = _OFF;
		mledStatus.led2 = _OFF;
		io_setLedStatus(mledStatus, ucRegCoilsBuf);

		moutput.out2 = _ON;
		moutput.out3 = _ON;
		io_setOutput(moutput, ucRegCoilsBuf);

		//TODO: man hinh hien thi ERROR

		if (io_getButton().reset == _ON) {
			timer_Start(TIMER_CLEARSENSOR, TIMERMAXVALUE);
			while (io_getButton().reset == _ON)
				;
			_time = time_Stop(TIMER_CLEARSENSOR);

		}
		if (_time > 20000) //time press reset button longer than 2sec
				{
			if(msetCalibValue_1 == CALIBSET)
				mledStatus.led1 = _ON;
			else
				mledStatus.led1 = _OFF;

			if(msetCalibValue_2 == CALIBSET)
				mledStatus.led2 = _ON;
			else
				mledStatus.led2 = _OFF;
			io_setLedStatus(mledStatus, ucRegCoilsBuf);

			moutput.out2 = _OFF;
			moutput.out3 = _OFF;
			io_setOutput(moutput, ucRegCoilsBuf);
			cycleMeasure = ERROR;
		}

	}

	while (CALCULATORVALUE == cycleMeasure && (0 == GET_IN0)) {
//		minput = io_getInput();
		if (0 == GET_IN2)  //C = 5
				{
			minput.in2 = _OFF;
			cycleMeasure = FINISH;
			DBG("cycleMeasure = FINISH");

			if (ZONLY == mdata.mode) {
				mdata.mode = MEASUREALL;
			} else if (ZERROR1 == mdata.mode) {
				mdata.mode = ZERROR2; // macro for LCD to print Z =... (Z cannot measure)
			}
			app_CalculatorValue(cycleMeasure, mdata.mode, MEASUREMENT_1);

			if ((NONE != mdata.mode) && (CALIBSET == msetCalibValue_1)) {
				write_SDCard(mdata, MEASUREMENT_1_FILE_NAME);
			}
			screen_DataMeasureType1(mdata, msetCalibValue_1, MEASUREMENT_1, NOT_SHOW_HIS);
			while (0 == GET_IN2 && (0 == GET_IN0))
				;
		}
	}

	while (cycleMeasure == FINISH && (0 == GET_IN0)) {
		//waiting for GET_IN0 = 1 to set calib
	}

	if (GET_IN0 == 1 /*end of cycle measurement */) {
		moutput.out0 = _OFF;
		moutput.out1 = _OFF;
		moutput.out2 = _OFF;
		moutput.out3 = _OFF;
		moutput.out4 = _OFF;
		moutput.out5 = _OFF;
		moutput.out6 = _OFF;
		moutput.out7 = _OFF;
		io_setOutput(moutput, ucRegCoilsBuf);

		if(cycleMeasure == WAITMEASUREX1Y1 && CALIBSET == msetCalibValue_1) // write to SD card if only measure Z
			write_SDCard(mdata, MEASUREMENT_1_FILE_NAME);

		if (cycleMeasure == FINISH && CALIBRESET == msetCalibValue_1) {
			/*-----SET value -----*/
			timer_Start(TIMER_CLEARCALIB, TIMERCLEARCALIB); //wait SET button in 6sec
			do {
				mbutton = io_getButton();
			} while (TIME_FINISH != timer_Status(TIMER_CLEARCALIB)
					&& mbutton.set != _ON);
			if (_ON == mbutton.set) {
				app_GetCalibValue(MEASUREMENT_1);
				if ((0 == mcalibValue.X1) && (0 == mcalibValue.X2)
						&& (0 == mcalibValue.Y1) && (0 == mcalibValue.Y2)
						&& (0 == mcalibValue.Z) && (ZERROR1 != mdata.mode)) {
					app_SetCalibValue(MEASUREMENT_1);
					mledStatus.led1 = _ON;
					msetCalibValue_1 = CALIBSET;
					io_setLedStatus(mledStatus, ucRegCoilsBuf);
					DBG("cycleMeasure = SET_DONE\n");
					app_CalculatorValue(FINISH, mdata.mode, MEASUREMENT_1);
					mainScreenFlag = MEASUREMENT_1;
					screen_DataMeasureType1(mdata, msetCalibValue_1, MEASUREMENT_1, NOT_SHOW_HIS);
					//in man hinh 0
				}
			}
		}
		else
			app_GotoMainScreen(msetCalibValue_1, MEASUREMENT_1);

	}
}
static void app_Measurement_2(void) {

	volatile CycleMeasure cycleMeasure = STOP;

	mdata.coordinates.X = 0;
	mdata.coordinates.Y = 0;
	mdata.coordinates.Z = 0;
	mdata.coordinates.aX = 0;
	mdata.coordinates.aY = 0;
	mdata.mode = NONE;

	mmeasureValue.X1 = 0;
	mmeasureValue.Y1 = 0;
	mmeasureValue.X2 = 0;
	mmeasureValue.Y2 = 0;
	mmeasureValue.Z = 0;

	DBG("===START MEASUREMENT 2===\r\n");

	cycleMeasure = meas_checkSensor(cycleMeasure, MEASUREMENT_2);
	if (cycleMeasure == _ERROR) {
		DBG("cycleMeasure == _ERROR");
	}
	
	cycleMeasure = meas_measurementZ(cycleMeasure, MEASUREMENT_2);
	cycleMeasure = meas_measurementX1Y1(cycleMeasure, MEASUREMENT_2);
	cycleMeasure = meas_measurementX2Y2(cycleMeasure, MEASUREMENT_2);

	while (cycleMeasure == _ERROR_XY) {
		DBG("Sensor X/Y Error");
		uint32_t _time = 0;
		mledStatus.led1 = _ON;
		mledStatus.led2 = _ON;
		io_setLedStatus(mledStatus, ucRegCoilsBuf);
		HAL_Delay(500);
		mledStatus.led1 = _OFF;
		mledStatus.led2 = _OFF;
		io_setLedStatus(mledStatus, ucRegCoilsBuf);

		moutput.out2 = _ON;
		moutput.out3 = _ON;
		io_setOutput(moutput, ucRegCoilsBuf);

		//TODO: man hinh hien thi ERROR

		if (io_getButton().reset == _ON) {
			timer_Start(TIMER_CLEARSENSOR, TIMERMAXVALUE);
			while (io_getButton().reset == _ON)
				;
			_time = time_Stop(TIMER_CLEARSENSOR);

		}
		if (_time > 20000) //time press reset button longer than 2sec
				{
			if (msetCalibValue_1 == CALIBSET)
				mledStatus.led1 = _ON;
			else
				mledStatus.led1 = _OFF;

			if (msetCalibValue_2 == CALIBSET)
				mledStatus.led2 = _ON;
			else
				mledStatus.led2 = _OFF;
			io_setLedStatus(mledStatus, ucRegCoilsBuf);

			moutput.out2 = _OFF;
			moutput.out3 = _OFF;
			io_setOutput(moutput, ucRegCoilsBuf);
			cycleMeasure = ERROR;
		}

	}

	while (CALCULATORVALUE == cycleMeasure && (0 == GET_IN1)) {
		minput = io_getInput();
		if (_ON == minput.in2)  //C = 5
				{
			minput.in2 = _OFF;
			cycleMeasure = FINISH;
			DBG("cycleMeasure = FINISH");

			if (ZONLY == mdata.mode) {
				mdata.mode = MEASUREALL;
			} else if (ZERROR1 == mdata.mode) {
				mdata.mode = ZERROR2; // macro for LCD to print Z =... (Z cannot measure)
			}
			app_CalculatorValue(cycleMeasure, mdata.mode, MEASUREMENT_2);

			if ((NONE != mdata.mode) && (CALIBSET == msetCalibValue_2)) {
				write_SDCard(mdata, MEASUREMENT_2_FILE_NAME);
			}
			screen_DataMeasureType1(mdata, msetCalibValue_2, MEASUREMENT_2, NOT_SHOW_HIS);
			while (_ON == io_getInput().in2 && (0 == GET_IN1))
				;
		}
	}

	while (cycleMeasure == FINISH && (0 == GET_IN1)) {
		//waiting for GET_IN1 = 1 to set calib
	}

	if (GET_IN1 == 1 /*end of cycle measurement */) {
		moutput.out0 = _OFF;
		moutput.out1 = _OFF;
		moutput.out2 = _OFF;
		moutput.out3 = _OFF;
		moutput.out4 = _OFF;
		moutput.out5 = _OFF;
		moutput.out6 = _OFF;
		moutput.out7 = _OFF;
		io_setOutput(moutput, ucRegCoilsBuf);

		if(cycleMeasure == WAITMEASUREX1Y1 && CALIBSET == msetCalibValue_2 ) // write to SD card if only measure Z
			write_SDCard(mdata, MEASUREMENT_2_FILE_NAME);

		if (cycleMeasure == FINISH && CALIBRESET == msetCalibValue_2) {
			/*-----SET value -----*/
			timer_Start(TIMER_CLEARCALIB, TIMERCLEARCALIB); //wait SET button in 6sec
			do {
				mbutton = io_getButton();
			} while (TIME_FINISH != timer_Status(TIMER_CLEARCALIB)
					&& mbutton.set != _ON);
			if (_ON == mbutton.set) {
				app_GetCalibValue(MEASUREMENT_2);
				if ((0 == mcalibValue.X1) && (0 == mcalibValue.X2)
						&& (0 == mcalibValue.Y1) && (0 == mcalibValue.Y2)
						&& (0 == mcalibValue.Z) && (ZERROR1 != mdata.mode)) {
					app_SetCalibValue(MEASUREMENT_2);
					mledStatus.led2 = _ON;
					msetCalibValue_2 = CALIBSET;
					io_setLedStatus(mledStatus, ucRegCoilsBuf);
					DBG("cycleMeasure = SET_DONE\n");
					app_CalculatorValue(FINISH, mdata.mode, MEASUREMENT_2);
					mainScreenFlag = MEASUREMENT_2;
					screen_DataMeasureType1(mdata, msetCalibValue_2, MEASUREMENT_2, NOT_SHOW_HIS);
				}
			}
		}
		else
		{
			app_GotoMainScreen(msetCalibValue_2, MEASUREMENT_2);
		}
	}

}

static void app_ClearAllOutput(void) {
	moutput.out0 = _OFF;
	moutput.out1 = _OFF;
	moutput.out2 = _OFF;
	moutput.out3 = _OFF;
	moutput.out4 = _OFF;
	moutput.out5 = _OFF;
	moutput.out6 = _OFF;
	moutput.out7 = _OFF;
	io_setOutput(moutput, ucRegCoilsBuf);
}


void app_CalculatorValue(CycleMeasure lcycleMeasures, uint8_t mode,
		uint8_t measurementIndex) {
	double db_DetaTX1 = 0;
	double db_DetaTY1 = 0;
	double db_DetaTX2 = 0;
	double db_DetaTY2 = 0;
	double db_DetaTZ = 0;

	double db_anpha1 = 0;
	double db_beta1 = 0;
	double db_anpha2 = 0;
	double db_beta2 = 0;

	VDRLZ_Input buffer;
	buffer = FLASH_ReadVDRLZ();

	double db_DetaXSS1 = 0;
	double db_DetaYSS1 = 0;
	double db_DetaXSS2 = 0;
	double db_DetaYSS2 = 0;
	double db_DetaXRB1 = 0;
	double db_DetaYRB1 = 0;
	double db_DetaXRB2 = 0;
	double db_DetaYRB2 = 0;
	double db_DetaZ = 0;
	double db_r1 = 0;
	double db_r2 = 0;
	double db_LXSS = 0;
	double db_LYSS = 0;
	double db_LXRB = 0;
	double db_LYRB = 0;

	uint8_t temp = 0;
	if (measurementIndex == 1) {
		temp = (uint8_t) msetCalibValue_1;
//		app_GetCalibValue(MEASUREMENT_1);
	} else {
		temp = (uint8_t) msetCalibValue_2;
		app_GetCalibValue(MEASUREMENT_2);
	}
	if (CALIBSET == temp) {
		mdata.time = rtc_Now();
//        DBG("**********************************************************\n")
		/*calculator Z*/
		if ((ZONLY == mode) || (MEASUREALL == mode)) {
			db_DetaTZ = ((double) (mmeasureValue.Z - mcalibValue.Z)) * 10.0
					/ 1000000.0; /* us/1000000 => s */
			db_DetaZ = buffer.V * db_DetaTZ;
			mdata.coordinates.Z = (int16_t) (db_DetaZ * 100.0);
//			char buf[100];
//			sprintf(buf,"mmeasureValue.Z =%ld mcalibValue.Z=%ld\n",mmeasureValue.Z,mcalibValue.Z);
			DBG(buf);
			if ((db_DetaZ > buffer.Z) || (db_DetaZ < (-buffer.Z))) {
				moutput.out3 = _ON;
			} else {
				moutput.out3 = _OFF;
			}
		}

		/*calculator X Y*/
		if (FINISH == lcycleMeasures) {
//			char buf[100];
			db_DetaTX1 = ((double) (mmeasureValue.X1 - mcalibValue.X1)) * 10.0
					/ 1000000.0; /* us/1000000 => s */
			db_DetaTY1 = ((double) (mmeasureValue.Y1 - mcalibValue.Y1)) * 10.0
					/ 1000000.0; /* us/1000000 => s */
			db_DetaTX2 = ((double) (mmeasureValue.X2 - mcalibValue.X2)) * 10.0
					/ 1000000.0; /* us/1000000 => s */
			db_DetaTY2 = ((double) (mmeasureValue.Y2 - mcalibValue.Y2)) * 10.0
					/ 1000000.0; /* us/1000000 => s */
//			sprintf(buf, "mmeasureValue.X1=%ld mcalibValue.X1=%ld\n",
//					mmeasureValue.X1, mcalibValue.X1);
//			DBG(buf);
//			sprintf(buf, "mmeasureValue.Y1=%ld mcalibValue.Y1=%ld\n",
//					mmeasureValue.Y1, mcalibValue.Y1);
//			DBG(buf);
//			sprintf(buf, "mmeasureValue.X2=%ld mcalibValue.X2=%ld\n",
//					mmeasureValue.X2, mcalibValue.X2);
//			DBG(buf);
//			sprintf(buf, "mmeasureValue.Y2=%ld mcalibValue.Y2=%ld\n",
//					mmeasureValue.Y2, mcalibValue.Y2);
//			DBG(buf);

			db_anpha1 = (buffer.V * db_DetaTX1) / buffer.D;
			db_beta1 = (buffer.V * db_DetaTY1) / buffer.D;
			db_anpha2 = (buffer.V * db_DetaTX2) / buffer.D;
			db_beta2 = (buffer.V * db_DetaTY2) / buffer.D;
//            DBG("db_anpha1 = %lf\n",db_anpha1);
//            DBG("db_beta1 = %lf\n",db_beta1);
//            DBG("db_anpha2 = %lf\n",db_anpha2);
//            DBG("db_beta2 = %lf\n",db_beta2);

			db_DetaXSS1 = (2 * buffer.D) * sin(db_anpha1 / 2)
					* cos(db_anpha1 / 2);
			db_DetaYSS1 = (2 * buffer.D) * sin(db_beta1 / 2)
					* cos(db_beta1 / 2);
			db_DetaXRB1 = db_DetaXSS1 * cos(3.142 / 4);
			db_DetaYRB1 = db_DetaYSS1 * cos(3.142 / 4);
//            DBG("db_DetaXSS1 = %lf\n",db_DetaXSS1);
//            DBG("db_DetaYSS1 = %lf\n",db_DetaYSS1);
//            DBG("db_DetaXRB1 = %lf\n",db_DetaXRB1);
//            DBG("db_DetaYRB1 = %lf\n",db_DetaYRB1);

			db_DetaXSS2 = (2 * buffer.D) * sin(db_anpha2 / 2)
					* cos(db_anpha2 / 2);
			db_DetaYSS2 = (2 * buffer.D) * sin(db_beta2 / 2)
					* cos(db_beta2 / 2);
			db_DetaXRB2 = db_DetaXSS2 * cos(3.142 / 4);
			db_DetaYRB2 = db_DetaYSS2 * cos(3.142 / 4);
//            DBG("db_DetaXSS2 = %lf\n",db_DetaXSS2);
//            DBG("db_DetaYSS2 = %lf\n",db_DetaYSS2);
//            DBG("db_DetaXRB2 = %lf\n",db_DetaXRB2);
//            DBG("db_DetaYRB2 = %lf\n",db_DetaYRB2);

			db_r1 = sqrt(
					(db_DetaYSS1 * db_DetaYSS1) + (db_DetaXSS1 * db_DetaXSS1));
			db_r2 = sqrt(
					(db_DetaYSS2 * db_DetaYSS2) + (db_DetaXSS2 * db_DetaXSS2));
//            DBG("db_r1 = %lf\n",db_r1);
//            DBG("db_r2 = %lf\n",db_r2);

			db_LXSS = (atan((db_DetaXSS2 - db_DetaXSS1) / buffer.L) * 180)
					/ 3.142;
			db_LYSS = (atan((db_DetaYSS2 - db_DetaYSS1) / buffer.L) * 180)
					/ 3.142;
			db_LXRB = (atan((db_DetaXRB2 - db_DetaXRB1) / buffer.L) * 180)
					/ 3.142;
			db_LYRB = (atan((db_DetaYRB2 - db_DetaYRB1) / buffer.L) * 180)
					/ 3.142;
//            DBG("db_LXSS = %lf\n",db_LXSS);
//            DBG("db_LYSS = %lf\n",db_LYSS);
//            DBG("db_LXRB = %lf\n",db_LXRB);
//            DBG("db_LYRB = %lf\n",db_LYRB);

			mdata.coordinates.X = (int16_t) (db_DetaXRB2 * 100.0);
			mdata.coordinates.Y = (int16_t) (db_DetaYRB2 * 100.0);

			mdata.coordinates.R = (int16_t) (db_r2 * 100.0);
			mdata.coordinates.aX = (int16_t) (db_LXRB * 10.0);
			mdata.coordinates.aY = (int16_t) (db_LYRB * 10.0);
//            DBG("X (db_DetaXRB2) = %lf\n",db_DetaXRB2);
//            DBG("Y (db_DetaYRB2)= %lf\n",db_DetaYRB2);
//            DBG("R (db_r2)= %lf\n",db_r2);
//            DBG("A (db_LXRB) = %lf\n",db_LXRB);
//            DBG("B (db_LYRB)= %lf\n",db_LYRB);
//            DBG("X1 (db_DetaXRB1)= %lf\n",db_DetaXRB1);
//            DBG("Y1 (db_DetaYRB1)= %lf\n",db_DetaYRB1);
			if (db_r2 > buffer.R) {
				moutput.out2 = _ON;
			} else {
				moutput.out2 = _OFF;
			}
		}
		io_setOutput(moutput, ucRegCoilsBuf);
	}
}

static void app_SetCalibValue(uint8_t measurementIndex) {
	mcalibValue.X1 = mmeasureValue.X1;
	mcalibValue.Y1 = mmeasureValue.Y1;
	mcalibValue.X2 = mmeasureValue.X2;
	mcalibValue.Y2 = mmeasureValue.Y2;
	mcalibValue.Z = mmeasureValue.Z;
	FLASH_WriteDataCalib(&mcalibValue, measurementIndex);
//    DBG("mcalibValue.X1 = %d\n",mcalibValue.X1);
//    DBG("mcalibValue.Y1 = %d\n",mcalibValue.Y1);
//    DBG("mcalibValue.X2 = %d\n",mcalibValue.X2);
//    DBG("mcalibValue.Y2 = %d\n",mcalibValue.Y2);
//    DBG("mcalibValue.Z = %d\n",mcalibValue.Z);
}

static void app_GetCalibValue(uint8_t measurementIndex) {
	mcalibValue = FLASH_ReadDataCalib(measurementIndex);
	if (mcalibValue.X1 == EMPTY && mcalibValue.X2 == EMPTY
			&& mcalibValue.Y1 == EMPTY && mcalibValue.Y2 == EMPTY
			&& mcalibValue.Z == EMPTY) {
		mcalibValue.X1 = 0;
		mcalibValue.X2 = 0;
		mcalibValue.Y1 = 0;
		mcalibValue.Y2 = 0;
		mcalibValue.Z = 0;
	}
//    DBG("mcalibValue.X1 = %d\n",mcalibValue.X1);
//    DBG("mcalibValue.Y1 = %d\n",mcalibValue.Y1);
//    DBG("mcalibValue.X2 = %d\n",mcalibValue.X2);
//    DBG("mcalibValue.Y2 = %d\n",mcalibValue.Y2);
//    DBG("mcalibValue.Z = %d\n",mcalibValue.Z);
}


static optionScreen_e_t app_optionMenu(void) {
	optionScreen_e_t optionIndex = measurement1Setting;
	uint8_t exit = 0;
	uint8_t inMeasHis = 0; //flag for measurement screen history

	screen_OptionMenu(&optionIndex);
	do {
		mbutton = io_getButton();
		if (mbutton.next == _ON) {
			while (_ON == io_getButton().next)
				;
			if (inMeasHis) {
				optionIndex = measurement2HisList;
			} else {
				optionIndex++;
			}
			screen_OptionMenu(&optionIndex);
		}
		if (mbutton.prev == _ON) {
			while (_ON == io_getButton().prev)
				;
			if (inMeasHis) {
				optionIndex = measurement1HisList;
			} else {
				optionIndex--;
			}
			screen_OptionMenu(&optionIndex);
		}

		if (mbutton.set == _ON) {
			while (_ON == io_getButton().set)
				;
			if (optionIndex == measurementHis) {
				optionIndex = measurement1HisList;
				screen_OptionMenu(&optionIndex);
				inMeasHis = 1;
			} else {
				exit = 1;
			}
		}
		if (mbutton.menu == _ON) {
			while (_ON == io_getButton().menu)
				;
			optionIndex = measurement1Setting; //go to mainscreen
			exit = 1;
		}

	} while (exit == 0);
	return optionIndex;
}

static void app_processOptionMenu(optionScreen_e_t optionMenu) {
	switch (optionMenu) {
	case measurement1Setting:

		app_GotoMainScreen(msetCalibValue_1, MEASUREMENT_1);
		break;
	case measurement2Setting:

		app_GotoMainScreen(msetCalibValue_2, MEASUREMENT_2);
		break;
	case measurement1HisList:
		app_HisValue(MEASUREMENT_1);
		break;
	case measurement2HisList:
		app_HisValue(MEASUREMENT_2);
		break;
	case VDLRZinput:
		app_SettingVDLRZ();
		break;
	case timeSetting:
		app_SettingRtc();
		break;
	case showIP:
		app_ShowIP();
		break;
	default:
		break;
	}
}

static void app_ShowIP(void) {
	uint8_t exit = 0;
	screen_showIP(&net_info);
	do {
		mbutton = io_getButton();
		if (mbutton.menu == _ON) {
			while (_ON == io_getButton().menu)
				;
			exit = 1;
		}
	} while (exit == 0);

	app_GotoMainScreen(msetCalibValue_1, MEASUREMENT_1);
}

static void app_HisValue(uint8_t measurementIndex) {
	uint8_t index = 0;
	uint8_t exit = 0;
	dataMeasure ldata;
	char fileName[17];
//	uint8_t u8_Led3 = mledStatus.led3;
	if(measurementIndex == MEASUREMENT_1)
	{
		strcpy(fileName, MEASUREMENT_1_FILE_NAME);
	}
	else
	{
		strcpy(fileName, MEASUREMENT_2_FILE_NAME);
	}
	LCD_Clear();
	ldata = read_SDCard(fileName, index);
	screen_DataMeasureType1(ldata, CALIBSET, measurementIndex, SHOW_HIS);
	do {
		mbutton = io_getButton();
		if (_ON == mbutton.next) {
			while (_ON == io_getButton().next)
				;
			index--;
			if (index > 9)
				index = 0;

			ldata = read_SDCard(fileName, index);
			screen_DataMeasureType1(ldata, CALIBSET, measurementIndex,
			SHOW_HIS);
		}
		if (_ON == mbutton.prev) {
			while (_ON == io_getButton().prev)
				;
			index++;
			if (index > 9)
				index = 9;

			ldata = read_SDCard(fileName, index);
			screen_DataMeasureType1(ldata, CALIBSET, measurementIndex,
			SHOW_HIS);
		}
		if (_ON == mbutton.set) {
			while (_ON == io_getButton().set)
				;
			uint8_t tempExit = 0;
			do {
				mbutton = io_getButton();
				if (mbutton.next == _ON) {
					while (_ON == io_getButton().next)
						;
					screen_DataMeasureType2(ldata, CALIBSET, measurementIndex,
					SHOW_HIS);
				}
				if (mbutton.prev == _ON) {
					while (_ON == io_getButton().prev)
						;
					screen_DataMeasureType1(ldata, CALIBSET, measurementIndex,
					SHOW_HIS);
				}
				if (mbutton.reset == _ON) {
					while (_ON == io_getButton().reset)
						;
					screen_DataMeasureType1(ldata, CALIBSET, measurementIndex,
					SHOW_HIS);
					tempExit = 1;
				}
				if (_ON == mbutton.menu) {
					while (_ON == io_getButton().menu)
						;
					tempExit = 1;
					exit = 1;
				}
			} while (tempExit == 0);
		}

		if (_ON == mbutton.menu) {
			while (_ON == io_getButton().menu)
				;
			exit = 1;
		}
	} while (exit == 0);
	app_GotoMainScreen(msetCalibValue_1, MEASUREMENT_1); // main screen
}

static void app_Init(void) {
	LCD_Init();
	W5500_init(); //if this line error -> check power of ethernet
	LCD_Clear();

	VDRLZ_Input temp;
	temp = FLASH_ReadVDRLZ();
	if(temp.D == 0xFFFFFFFF && temp.L == 0xFFFFFFFF && isnanf(temp.R) && temp.V == 0xFFFFFFFF && isnanf(temp.Z))
	{
		temp.V = 20;
		temp.D = 50;
		temp.R = 1.2;
		temp.L = 15;
		temp.Z = 1.2;
		FLASH_WriteVDRLZ(&temp);
	}

	app_GetCalibValue(MEASUREMENT_1);
	if ((mcalibValue.X1 == 0) && (mcalibValue.X2 == 0)
			&& (mcalibValue.Y1 == 0) && (mcalibValue.Y2 == 0)
			&& (mcalibValue.Z == 0)) {
		mledStatus.led1 = _OFF;
		msetCalibValue_1 = CALIBRESET;
	} else {
		mledStatus.led1 = _ON;
		msetCalibValue_1 = CALIBSET;
	}

	app_GetCalibValue(MEASUREMENT_2);
    if((mcalibValue.X1 == 0) && (mcalibValue.X2 == 0) && (mcalibValue.Y1 == 0) && (mcalibValue.Y2 == 0) && (mcalibValue.Z == 0))
    {
        mledStatus.led2 = _OFF;
        msetCalibValue_2 = CALIBRESET;
    }
    else
    {
        mledStatus.led2 = _ON;
        msetCalibValue_2 = CALIBSET;
    }

	io_setLedStatus(mledStatus, ucRegCoilsBuf);

	app_GotoMainScreen(msetCalibValue_1, MEASUREMENT_1);
}
static void app_GotoMainScreen(uint8_t option, uint8_t measurementIndex) {
	uint8_t index = 0;
	volatile dataMeasure data = {0};
	uint8_t exit = 0;

	mainScreenFlag = measurementIndex; //use for reset calib

	data = read_SDCard(MEASUREMENT_1_FILE_NAME, index);
	screen_DataMeasureType1(data, option, measurementIndex, NOT_SHOW_HIS);

	do {
		mbutton = io_getButton();
		minput = io_getInput();
		if (mbutton.next == _ON) {
			while (_ON == io_getButton().next)
				;
			screen_DataMeasureType2(data, option, measurementIndex,
			NOT_SHOW_HIS);
		}
		if (mbutton.prev == _ON) {
			while (_ON == io_getButton().prev)
				;
			screen_DataMeasureType1(data, option, measurementIndex,
			NOT_SHOW_HIS);
		}
		if (_ON == mbutton.menu) {
			while (_ON == io_getButton().menu)
				;
			menuScreenFlag = 1;
			exit = 1;
			break;
		}

	} while (exit == 0 && _OFF == minput.in0 && _OFF == minput.in1
			&& _OFF == mbutton.reset);
}

uint32_t now= 0;
CycleMeasure meas_checkSensor(CycleMeasure cycleMeasure, uint8_t measurementIndex) {

	while ((STOP == cycleMeasure) && (0 == GET_INPUT(measurementIndex)))
	{
		app_ClearAllOutput();
		minput.in0 = _OFF;
		cycleMeasure = CLEARSENSOR;
		timer_Start(TIMER_CLEARSENSOR, TIMERCLEARSENSOR);
		moutput.out4 = _ON; // O7 ON start clear sensor
		io_setOutput(moutput, ucRegCoilsBuf);
		DBG("Clearing sensor \r\n");

	}
	/********************************************## 2 ##*******************************************/
	/********************************************## 3 ##*******************************************/
	/*Waiting clear Sensor*/
	while (CLEARSENSOR == cycleMeasure) {
		if (TIME_FINISH == timer_Status(TIMER_CLEARSENSOR)) {
			msensor = io_getSensor();
			if ((_OFF == msensor.s0) && (_OFF == msensor.s1)) {
				now = HAL_GetTick();
				moutput.out0 = _ON;
				moutput.out4 = _OFF; //turn off O7
				io_setOutput(moutput, ucRegCoilsBuf);

				cycleMeasure = WAITMEASUREZ;
				DBG("Check done - SENSOR OK\r\n");

			} else {
				moutput.out0 = _OFF;
				moutput.out4 = _OFF; //turn off O7
				io_setOutput(moutput, ucRegCoilsBuf);
				cycleMeasure = _ERROR;
				DBG("Check done - SENSOR [NOT] OK\r\n");
			}
		}

	}
	return cycleMeasure;
}

CycleMeasure meas_measurementZ(CycleMeasure cycleMeasure, uint8_t measurementIndex)
{
	setCalibValue mcalibValue;
	char fileName[17];
	uint8_t trigger_in2 = 0;
	CycleMeasure cycleMeasureZ = MEASUREZ;

	if(measurementIndex == MEASUREMENT_1)
	{
		mcalibValue = msetCalibValue_1;
		strcpy(fileName, MEASUREMENT_1_FILE_NAME);
	}
	else
	{
		mcalibValue = msetCalibValue_2;
		strcpy(fileName, MEASUREMENT_2_FILE_NAME);
	}
	while ((WAITMEASUREZ == cycleMeasure) && (0 == GET_INPUT(measurementIndex))) {
		if(0 == GET_IN2)//if (_ON == minput.in2) //C=1
				{
			timer_Start(TIMER_Z, TIMERMAXVALUE); //start timer to measure Z
			trigger_in2 = 1;
//			msensor.s0 = _OFF;
//			msensor.s1 = _OFF;
			cycleMeasure = MEASUREZ;
			cycleMeasureZ = Z_NOT_OK;
			DBG("Measure Z start and wait stable z\r\n");
//			while (0 == GET_IN2 && (0 == GET_INPUT(measurementIndex)))
//				;
			while(trigger_in2 == 1)
			{
				if(GET_IN2 == 1)
				{
					trigger_in2 = 0;
				}
				if((0 == GET_SENSOR0) && (0 == GET_SENSOR1) && cycleMeasureZ == Z_NOT_OK)
				{
					mmeasureValue.Z = time_Stop(TIMER_Z);
					cycleMeasureZ = Z_OK;
				}
			}
		}
	};

	while ((MEASUREZ == cycleMeasure) && (0 == GET_INPUT(measurementIndex))) {
		if ((0 == GET_SENSOR0) && (0 == GET_SENSOR1)
				&& cycleMeasureZ == Z_NOT_OK) //if ((_ON == minput.in2) && (_ON == msensor.s0) && (_ON == msensor.s1)) //C=2
						{
			mmeasureValue.Z = time_Stop(TIMER_Z);
			cycleMeasureZ = Z_OK;
		}

		if ((MEASUREZ == cycleMeasure) && (0 == GET_IN2)) //else if ((MEASUREZ == cycleMeasure) && (_ON == minput.in2) && ((_OFF == msensor.s0) || (_OFF == msensor.s1))) //C=2
		{

			if (cycleMeasureZ != Z_OK) {
				minput.in2 = _OFF;
				moutput.out1 = _OFF;
				io_setOutput(moutput, ucRegCoilsBuf);
				time_Stop(TIMER_Z);
				cycleMeasureZ = Z_DONE;
				mdata.mode = ZERROR1;
				DBG("(C=2) Measure Z - [NOT] OK\r\n");
				while (0 == GET_IN2 && (0 == GET_INPUT(measurementIndex)))
					;
			} else { //Z_OK
				moutput.out1 = _ON;
				io_setOutput(moutput, ucRegCoilsBuf);
				mdata.mode = ZONLY;
				app_CalculatorValue(cycleMeasure, mdata.mode, measurementIndex);
				screen_DataMeasureType1(mdata, mcalibValue, measurementIndex,
				NOT_SHOW_HIS);

				minput.in2 = _OFF;
				cycleMeasureZ = Z_DONE;
				DBG("(C=2) Measure Z - OK\r\n");
			}
		}

		if ((Z_DONE == cycleMeasureZ) && (1 == GET_IN2))//if (((Z_OK == cycleMeasure) || (Z_NOT_OK == cycleMeasure)) && (_OFF == minput.in2))
		{
			cycleMeasure = WAITMEASUREX1Y1;
			DBG("(C=2 cycleMeasure = WAITMEASUREX1Y1\r\n");

		}
	}
	return cycleMeasure;
}

CycleMeasure meas_measurementX1Y1(CycleMeasure cycleMeasure, uint8_t measurementIndex) {
	CycleMeasureSensor cycleMeasureX = SEN_STOP;
	CycleMeasureSensor cycleMeasureY = SEN_STOP;
	while ((WAITMEASUREX1Y1 == cycleMeasure) && (0 == GET_INPUT(measurementIndex))) {
		if (0 == GET_IN2) //C=3
				{
			minput.in2 = _OFF;
			cycleMeasure = MEASUREX1Y1;
			timer_Start(TIMER_X, TIMERMAXVALUE);
			timer_Start(TIMER_Y, TIMERMAXVALUE);

			cycleMeasureX = SEN_START;
			cycleMeasureY = SEN_START;

			DBG("Start counter X1, Y1\n");
			while (0 == GET_IN2 && (0 == GET_INPUT(measurementIndex)))
				;
		}
	}

	while (cycleMeasure == MEASUREX1Y1 && (0 == GET_INPUT(measurementIndex))) {
//		msensor = io_getSensor();
		if (SEN_START == cycleMeasureX && (0 == GET_SENSOR0)) {
			mmeasureValue.X1 = time_Stop(TIMER_X); //Stop counter X
//			msensor.s0 = _OFF;
			cycleMeasureX = SEN_FINISH;
			DBG("X1 = SEN_FINISH\n");
			while (io_getSensor().s0 == _ON && (0 == GET_INPUT(measurementIndex)))
				;
		}
		if ((SEN_START == cycleMeasureY) && (0 == GET_SENSOR1)) {
			mmeasureValue.Y1 = time_Stop(TIMER_Y); //Stop counter Y
//			msensor.s1 = _OFF;
			cycleMeasureY = SEN_FINISH;
			DBG("Y1 = SEN_FINISH\n");
			while (io_getSensor().s1 == _ON && (0 == GET_INPUT(measurementIndex)))
				;
		}
		if ((SEN_FINISH == cycleMeasureX) && (SEN_FINISH == cycleMeasureY)) {
			cycleMeasure = WAITMEASUREX2Y2;
			DBG("cycleMeasure = WAITMEASUREX2Y2\n");
		}
		if (0 == GET_IN2 && cycleMeasure != WAITMEASUREX2Y2) 	//if (io_getInput().in2 == _ON && cycleMeasure != WAITMEASUREX2Y2)
		{
			cycleMeasure = _ERROR_XY;
		}
	}
	return cycleMeasure;
}

CycleMeasure meas_measurementX2Y2(CycleMeasure cycleMeasure, uint8_t measurementIndex) {
	CycleMeasureSensor cycleMeasureX = SEN_STOP;
	CycleMeasureSensor cycleMeasureY = SEN_STOP;
	while ((WAITMEASUREX2Y2 == cycleMeasure) && (0 == GET_INPUT(measurementIndex))) {
		if(0 == GET_IN2)//if (_ON == minput.in2) //C=4
				{
			minput.in2 = _OFF;
			cycleMeasure = MEASUREX2Y2;
			timer_Start(TIMER_X, TIMERMAXVALUE);
			timer_Start(TIMER_Y, TIMERMAXVALUE);

			cycleMeasureX = SEN_START;
			cycleMeasureY = SEN_START;

			DBG("Start counter X2, Y2\n");

			while (0 == GET_IN2 && (0 == GET_INPUT(measurementIndex)))
				;
		}
	}

	while (cycleMeasure == MEASUREX2Y2 && (0 == GET_INPUT(measurementIndex))) {
//		msensor = io_getSensor();
		if (SEN_START == cycleMeasureX && (0 == GET_SENSOR0)) {
			mmeasureValue.X2 = time_Stop(TIMER_X); //Stop counter X
//			msensor.s0 = _OFF;
			cycleMeasureX = SEN_FINISH;
			DBG("X2 = SEN_FINISH\n");

			while (io_getSensor().s0 == _ON && (0 == GET_INPUT(measurementIndex)))
				;
		}
		if ((SEN_START == cycleMeasureY) && (0 == GET_SENSOR1)) {
			mmeasureValue.Y2 = time_Stop(TIMER_Y); //Stop counter Y
//			msensor.s1 = _OFF;
			cycleMeasureY = SEN_FINISH;
			DBG("Y2 = SEN_FINISH\n");

			while (io_getSensor().s1 == _ON && (0 == GET_INPUT(measurementIndex)))
				;
		}
		if ((SEN_FINISH == cycleMeasureX) && (SEN_FINISH == cycleMeasureY)) {
			cycleMeasure = CALCULATORVALUE;
			DBG("cycleMeasure = CALCULATORVALUE\n");
		}
		if (0 == GET_IN2 && cycleMeasure != CALCULATORVALUE) {
			cycleMeasure = _ERROR_XY;
		}
	}
	return cycleMeasure;
}

static void updateMBRegister(void) {
	uint16_t tempInput[REG_INPUT_NREGS] = { GET_SENSOR0, GET_SENSOR1, GET_IN0,
			GET_IN1, GET_IN2, GET_IN3, GET_IN4, GET_IN5, GET_IN6, GET_IN7 };
	memcpy(usRegInputBuf, tempInput, sizeof(usRegInputBuf));

//	modbus_tcps(HTTP_SOCKET, MBTCP_PORT); //instead of using eMBPoll()
	eMBPoll();
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
