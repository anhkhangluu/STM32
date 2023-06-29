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

//DHCP
#include "socket.h"
#include "wizchip_conf.h"
#include "string.h"
#include "stdio.h"
#include "dhcp.h"
#include "stdbool.h"
#include "mb.h"

#include "structer.h"
#include "io.h"
#include "screen.h"
#include "time.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define USART_DEBUG
#ifdef USART_DEBUG
#include "stdio.h"
#include "string.h"
#define USART_DEBUG_HANDLER		&huart3
#define PRINT_NETINFO(netInfo) do { 																					\
  HAL_UART_Transmit(USART_DEBUG_HANDLER, (uint8_t*)NETWORK_MSG, strlen(NETWORK_MSG), 100);											\
  sprintf(msg, MAC_MSG, netInfo.mac[0], netInfo.mac[1], netInfo.mac[2], netInfo.mac[3], netInfo.mac[4], netInfo.mac[5]);\
  HAL_UART_Transmit(USART_DEBUG_HANDLER, (uint8_t*)msg, strlen(msg), 100);															\
  sprintf(msg, IP_MSG, netInfo.ip[0], netInfo.ip[1], netInfo.ip[2], netInfo.ip[3]);										\
  HAL_UART_Transmit(USART_DEBUG_HANDLER, (uint8_t*)msg, strlen(msg), 100);															\
  sprintf(msg, NETMASK_MSG, netInfo.sn[0], netInfo.sn[1], netInfo.sn[2], netInfo.sn[3]);								\
  HAL_UART_Transmit(USART_DEBUG_HANDLER, (uint8_t*)msg, strlen(msg), 100);															\
  sprintf(msg, GW_MSG, netInfo.gw[0], netInfo.gw[1], netInfo.gw[2], netInfo.gw[3]);										\
  HAL_UART_Transmit(USART_DEBUG_HANDLER, (uint8_t*)msg, strlen(msg), 100);															\
} while(0)
#endif
#define DHCP_SOCKET     0
#define SNTP_SOCKET      1
#define HTTP_SOCKET     0
#define MBTCP_PORT      502

#define REG_INPUT_START       0x0001
#define REG_INPUT_NREGS       8

#define REG_HOLDING_START     0x0001
#define REG_HOLDING_NREGS     8

#define REG_COILS_START       0x0001
#define REG_COILS_SIZE        16

#define REG_DISCRETE_START    0x0001
#define REG_DISCRETE_SIZE     16
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
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#ifdef USART_DEBUG
char msg[60];
#endif
/*----------------------------DHCP-----------------------------*/
uint8_t dhcp_buffer[1024];
volatile bool ip_assigned = false;

uint16_t usRegInputBuf[REG_INPUT_NREGS] = { 0x1000, 0x1001, 0x1002, 0x1003,
		0x1004, 0x1005, 0x1006, 0x1007 };
uint16_t usRegHoldingBuf[REG_HOLDING_NREGS] = { 0x2000, 0x2001, 0x2002, 0x2003,
		0x2004, 0x2005, 0x2006, 0x2007 };
uint8_t ucRegCoilsBuf[REG_COILS_SIZE / 8] = { 0xaa, 0xfe };
uint8_t ucRegDiscreteBuf[REG_DISCRETE_SIZE / 8] = { 0x98, 0x6e };
/*---------------------------------------------------*/

/*-------------SD card Fatfs-----------*/
FATFS fs;
FATFS *pfs;
FIL file;
FRESULT FOR; //file operation result
DWORD fre_clust;
uint32_t totalSpace, freeSpace;
/*-------------------------------------*/

/*----------------app.c----------------*/
static dataMeasure mdata = { { 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0 } };

static Time mtime = { 23, 05, 21, 06, 00, 00 };

static button mbutton;
static input minput;
static sensor msensor;
static output moutput;
static ledStatus mledStatus;
static MeasureValue mcalibValue;
static MeasureValue mCurrentMeasureValue;
static MeasureValue mmeasureValue;
static setCalibValue msetCalibValue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
#ifdef USART_DEBUG
void send_uart(char *string, ...) {
	uint8_t len = strlen(string);
	HAL_UART_Transmit(&USART_DEBUG_HANDLER, (uint8_t*) string, len, HAL_MAX_DELAY); // transmit in blocking mode
}
#endif

void Callback_IPAssigned(void) {
	ip_assigned = true;
}
void Callback_IPConflict(void) {
	printf("Callback: IP conflict!\r\n");
}

static void process_SD_Card(void);
void W5500_init();

/*---------------app.c------------*/
static void app_SettingRtc(void);
static void app_SettingData(void);
static void app_GetEeprom(void);
static void app_Measurement(void);
static void app_SetCalibValue(void);
static void app_GetCalibValue(void);
static void app_CalculatorValue(CycleMeasure lcycleMeasure);
static void app_HisValue(void);
static void app_ClearAllOutput(void);
static void app_GotoMainScreen(uint8_t option);
static void app_SetCurrentMeasureValue(void);
static void app_GetCurrentMeasureValue(void);
static void app_TrigerOutputON(void);
static void app_TrigerOutputOFF(void);
static optionScreen_e_t app_optionMenu(void);
//static void app_

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
	optionScreen_e_t currentOption;
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  MX_RTC_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  	  HAL_TIM_Base_Start(&htim6);//use for delay_us
	HAL_TIM_Base_Start_IT(&htim7); //timer interrupt every 100us
	HAL_TIM_Base_Start_IT(&htim1); //Start timer input capture
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);

	LCD_Init();
	process_SD_Card();
	W5500_init();

	LCD_Clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  modbus_tcps(HTTP_SOCKET, MBTCP_PORT);
		/*app.c*/
		uint32_t time = 0;
		mbutton = io_getButton();
		minput = io_getInput();

		if (_ON == minput.in0) {
			minput.in0 = _OFF;
			app_Measurement();
		}
		//------------------------------------------------------------------------------------------
		// tthem nut menu ve mang hinh chinh
		if(_ON == mbutton.menu )
		{
			while(_ON == mbutton.menu)
				mbutton = io_getButton();
			currentOption = app_optionMenu();
		}



		if (_ON == mbutton.reset) {
			mbutton.reset = _OFF;
			timer_Start(TIMER_CLEARCALIB, TIMERCLEARCALIB);
			do {
				/* code */
			} while (_ON == io_getButton().reset);
			time = time_Stop(TIMER_CLEARCALIB);

			if ((time >= 10000) && (time <= 30000)) {
				app_TrigerOutputOFF();
				moutput.out2 = _OFF;
				moutput.out3 = _OFF;
				moutput.out1 = _OFF;
				moutput.out0 = _OFF;
				moutput.rl1 = _OFF;
				moutput.rl2 = _OFF;
				io_setOutput(moutput);
				app_TrigerOutputON();
			} else if ((time >= 50000)) {
				MeasureValue vl = { 0, 0, 0, 0, 0 };
				eep_WriteDataCalib(&vl);
				eep_ReadDataCalib(&mcalibValue);
				DBG("eep_WriteDataCalib(&vl)");
				mledStatus.led3 = _OFF;
				msetCalibValue = CALIBRESET;
				io_setLedStatus(mledStatus);
				app_GotoMainScreen(CALIBRESET);
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 48000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 99;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 48;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|OUT3_Pin
                          |GPIO_PIN_8|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, W5500_CS_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUT0_Pin|OUT1_Pin|OUT2_Pin|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE4 PE5 PE6
                           OUT3_Pin PE8 PE12 PE13
                           PE14 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |OUT3_Pin|GPIO_PIN_8|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : W5500_CS_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = W5500_CS_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 BT_SET_Pin BT_RESERVED_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|BT_SET_Pin|BT_RESERVED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT0_Pin OUT1_Pin OUT2_Pin PB8
                           PB9 */
  GPIO_InitStruct.Pin = OUT0_Pin|OUT1_Pin|OUT2_Pin|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           BT_PREV_Pin BT_NEXT_Pin BT_MENU_Pin BT_RESET_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |BT_PREV_Pin|BT_NEXT_Pin|BT_MENU_Pin|BT_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
void W5500_init() {
#ifdef USART_DEBUG
	send_uart("W5500 init() called!");
#endif
	reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
	reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
	reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);
	uint8_t rx_tx_buff_sizes[] = { 2, 2, 2, 2, 2, 2, 2, 2 };
	wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);
#ifdef USART_DEBUG
	send_uart("Calling DHCP_init()...\r\n");
#endif
	wiz_NetInfo net_info = { .mac = { 0xEA, 0x11, 0x22, 0x33, 0x44, 0xEA },
			.dhcp = NETINFO_DHCP };

	setSHAR(net_info.mac);

	DHCP_init(DHCP_SOCKET, dhcp_buffer);
#ifdef USART_DEBUG
	send_uart("Registering DHCP callbacks...\r\n");
#endif
	reg_dhcp_cbfunc(Callback_IPAssigned, Callback_IPAssigned,
			Callback_IPConflict);
#ifdef USART_DEBUG
	send_uart("Calling DHCP_run()...\r\n");
#endif
	uint32_t TimeOut = 1000000;
	//get IP assigned
	while ((!ip_assigned) && (TimeOut > 0)) {
		DHCP_run();
		TimeOut--;
	}

	if (!ip_assigned) {
#ifdef USART_DEBUG
		send_uart("\r\nIP was not assigned :(\r\n");
#endif
		return;
	}

	getIPfromDHCP(net_info.ip);
	getGWfromDHCP(net_info.gw);
	getSNfromDHCP(net_info.sn);

	uint8_t dns[4];
	getDNSfromDHCP(dns);
#ifdef USART_DEBUG
	PRINT_NETINFO(net_info);
	send_uart("Calling wizchip_setnetinfo()...\r\n");
#endif
	wizchip_setnetinfo(&net_info);
	HAL_Delay(200);
	eMBErrorCode MBresult;
	MBresult = eMBTCPInit(MBTCP_PORT);
	if (MBresult == MB_ENOERR) {
#ifdef USART_DEBUG
		send_uart("\r\nModbus-TCP initial success!\r\n");
#endif
	} else {
#ifdef USART_DEBUG
		send_uart("\r\nModbus-TCP initial error\r\n");
#endif
		return;
	}

	MBresult = eMBEnable();
	HAL_Delay(200);
	if (MBresult == MB_ENOERR) {
#ifdef USART_DEBUG
		send_uart("\r\nModbus-TCP Start!\r\n");
#endif
	} else {
#ifdef USART_DEBUG
		send_uart("\r\nModbus-TCP error!\r\n");
#endif
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
}

static void process_SD_Card(void) {
	FATFS FatFs;
	FIL fil;	//file
	char buff[100];

	do {
		//mount SD card
		if (f_mount(&FatFs, "", 0) != FR_OK) //0 = mount late, 1 = mount now
				{
			break;
		}

		//Read size and free space of SD Card
		FATFS *pfs;
		DWORD fre_clust;
		uint32_t totalSpace;
		uint32_t freeSpace;

		if (f_getfree("", &fre_clust, &pfs) != FR_OK) {
			break;
		}
		totalSpace = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
		freeSpace = (uint32_t) (fre_clust * pfs->csize * 0.5);

		//Open file
		if (f_open(&fil, "test.txt", FA_READ | FA_OPEN_ALWAYS | FA_WRITE)
				!= FR_OK) //In this mode, it will create the file if file not existed
				{
			break;
		}

		//Write data to "test.txt"
		f_puts("This is a sample", &fil); //write string to file

		//Close file
		if (f_close(&fil) != FR_OK) {
			break;
		}

		//Open file
		if (f_open(&fil, "test2.txt", FA_READ) != FR_OK) {
			break; //if file d
		}
		//Read data in file
		f_gets(buff, sizeof(buff), &fil);

		//Close file
		if (f_close(&fil) != FR_OK) {
			break;
		}

#if 0
		//delete file
		if(f_unlink("test.txt") != FR_OK)
		{
			break;
		}
#endif

	} while (0);
	f_mount(NULL, "", 0); //unmount SD card

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	//this function will reach if have a rising edge appear on tim1
	if (htim1.Channel == HAL_TIM_ACTIVE_CHANNEL_1
			|| htim1.Channel == HAL_TIM_ACTIVE_CHANNEL_2) {

	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) { //should check
	if (htim == &htim7) {
		register uint8_t i;
		for (i = 0; i < MAX_TIME; i++) {
			if (TIME_RUN == mtimer[i].status) {
				mtimer[i].inc += 1;
				if (mtimer[i].inc >= mtimer[i].count) {
					mtimer[i].status = TIME_FINISH;
					mtimer[i].flat = 1;
				}
			}
		}
	}
}

// app.c
static void app_SettingRtc(void) {
	CycleTime cycle = SET_YEAR;
	uint8_t exit = 0;
	uint8_t tmp = 0;
	screen_Clear();
	do {
		/*set year*/
		if (SET_YEAR == cycle) {
			mtime = rtc_Now();
			screen_setDateTime(mtime, SET_YEAR);
			do {
				mbutton = io_getButton();
				if (_ON == mbutton.set) {
					tmp = (mtime.year >= 2000) ?
							(mtime.year - 2000) : (byte) mtime.year;
					tmp += 1;
					if (99 < tmp)
						tmp = 0;
					mtime.year = tmp + 2000;
					rtc_SetYear(mtime.year);
					screen_setDateTime(mtime, SET_YEAR);
					delay(TIME_WAIT);
				}
				if (_ON == mbutton.next) {
					cycle = SET_MONTH;
					while (_ON == io_getButton().next)
						;
				}
				if (_ON == mbutton.reset) {
					while (_ON == io_getButton().reset)
						;
					exit = 1;
					break;
				}
			} while (SET_YEAR == cycle);
		}
		/*set month*/
		if (SET_MONTH == cycle) {
			mtime = rtc_Now();
			screen_setDateTime(mtime, SET_MONTH);
			do {
				mbutton = io_getButton();
				if (_ON == mbutton.set) {
					mtime.month += 1;
					if (12 < mtime.month)
						mtime.month = 1;
					rtc_SetMonth(mtime.month);
					screen_setDateTime(mtime, SET_MONTH);
					delay(TIME_WAIT);
				}
				if (_ON == mbutton.next) {
					cycle = SET_DAY;
					while (_ON == io_getButton().next)
						;
				}
				if (_ON == mbutton.reset) {
					while (_ON == io_getButton().reset)
						;
					exit = 1;
					break;
				}
			} while (SET_MONTH == cycle);
		}
		/*set day*/
		if (SET_DAY == cycle) {
			mtime = rtc_Now();
			screen_setDateTime(mtime, SET_DAY);
			do {
				mbutton = io_getButton();
				if (1U == mbutton.set) {
					mtime.day += 1;
					if (31 < mtime.day)
						mtime.day = 1;
					rtc_SetDay(mtime.day);
					screen_setDateTime(mtime, SET_DAY);
					delay(TIME_WAIT);
				}
				if (_ON == mbutton.next) {
					cycle = SET_HOUR;
					while (_ON == io_getButton().next)
						;
				}
				if (_ON == mbutton.reset) {
					while (_ON == io_getButton().reset)
						;
					exit = 1;
					break;
				}
			} while (SET_DAY == cycle);
		}
		/*set hour*/
		if (SET_HOUR == cycle) {
			mtime = rtc_Now();
			screen_setDateTime(mtime, SET_HOUR);
			do {
				mbutton = io_getButton();
				if (_ON == mbutton.set) {
					mtime.hour += 1;
					if (59 < mtime.hour)
						mtime.hour = 1;
					rtc_SetHour(mtime.hour);
					rtc_SetSecond(0);
					screen_setDateTime(mtime, SET_HOUR);
					delay(TIME_WAIT);
				}
				if (_ON == mbutton.next) {
					cycle = SET_MINUTE;
					while (_ON == io_getButton().next)
						;
				}
				if (_ON == mbutton.reset) {
					while (0U == io_getButton().reset)
						;
					exit = 1;
					break;
				}
			} while (SET_HOUR == cycle);
		}
		/*set minute*/
		if (SET_MINUTE == cycle) {
			mtime = rtc_Now();
			screen_setDateTime(mtime, SET_MINUTE);
			do {
				mbutton = io_getButton();
				if (_ON == mbutton.set) {
					mtime.minute += 1;
					if (59 < mtime.minute)
						mtime.minute = 1;
					rtc_SetMinute(mtime.minute);
					rtc_SetSecond(0);
					screen_setDateTime(mtime, SET_MINUTE);
					delay(TIME_WAIT);
				}
				if (_ON == mbutton.next) {
					cycle = SET_YEAR;
					while (_ON == io_getButton().next)
						;
				}
				if (_ON == mbutton.reset) {
					while (_ON == io_getButton().reset)
						;
					exit = 1;
					break;
				}
			} while (SET_MINUTE == cycle);
		}
	} while (0 == exit);
	if (CALIBSET == msetCalibValue) {
		app_GotoMainScreen(CALIBSET);
	} else {
		app_GotoMainScreen(CALIBRESET);
	}
}

static void app_Measurement(void)
{
    CycleMeasure cycleMeasure = STOP;
    CycleMeasureSensor cycleMeasureX = SEN_STOP;
    CycleMeasureSensor cycleMeasureY = SEN_STOP;
    CycleMeasureSensor cycleMeasureZ = SEN_STOP;
    SensorChange XStatus = SENSORNOCHANGE;
    SensorChange YStatus = SENSORNOCHANGE;
    GetInputType getInput = GET_SENSOR;

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

    do
    {
        if(GET_BUTTON == getInput)
        {
            mbutton = io_getButton();
            minput = io_getInput();
        }
        else
        {
            minput = io_getInput();
            msensor = io_getSensor();
        }
        /********************************************## 1 ##*******************************************/
        /*Robot di chuyển tới vị trí P0 Robot xuất tín hiệu cho X10 cho phép quá trình bắt đầu*/
        if((STOP == cycleMeasure) && (_ON == minput.in0)) // X10=ON
        {
            app_ClearAllOutput();
            minput.in0 = _OFF;
            cycleMeasure = CLEARSENSOR;
            getInput = GET_SENSOR;
            timer_Start(TIMER_CLEARSENSOR, TIMERCLEARSENSOR);
            moutput.out0 = _OFF;
            moutput.out1 = _OFF;
            moutput.rl1 = _ON;
            io_setOutput(moutput);
//            DBG("cycleMeasure = CLEARSENSOR\n");
        }
        /********************************************## 2 ##*******************************************/
        /********************************************## 3 ##*******************************************/
        /*Waiting clear Sensor*/
        if((CLEARSENSOR == cycleMeasure) && (TIME_FINISH == timer_Status(TIMER_CLEARSENSOR)))
        {
            if((_OFF == msensor.s0) && (_OFF == msensor.s1))
            {
                moutput.rl1 = _OFF;
                moutput.out0 = _ON;
                moutput.out1 = _OFF;
                io_setOutput(moutput);
                msensor.s0 = _OFF;
                msensor.s1 = _OFF;
                getInput = GET_SENSOR;
                cycleMeasure = WAITMEASUREZ;
//                DBG("SENSOR OK\n");
            }
            else
            {
                moutput.rl1 = _OFF;
                moutput.out0 = _OFF;
                moutput.out1 = _OFF;
                io_setOutput(moutput);
                cycleMeasure  = ERROR;
//                DBG("SENSOR NOT OK\n");
            }
            getInput = GET_SENSOR;
        }
        /********************************************## 4 ##*******************************************/
        while ((WAITMEASUREZ == cycleMeasure) && (0 == GET_IN0))
        {
            minput = io_getInput();
            if(_ON == minput.in1) //C=1
            {
                /*Start couter*/
                timer_Start(TIMER_CLEARSENSOR, TIMEWAITX11);
                minput.in1 = _OFF;
                msensor.s0 = _OFF;
                msensor.s1 = _OFF;
                getInput = GET_SENSOR;
                cycleMeasureZ = SEN_START;
                cycleMeasure = WAITRBSTABLEZ;
//                DBG("C=1 START TIME MEASURE Z\n");
            }
        };
        while ((WAITRBSTABLEZ == cycleMeasure) && (0 == GET_IN0)) // Waiting 200ms
        {
            if(TIME_FINISH == timer_Status(TIMER_CLEARSENSOR))
            {
                /*Start couter*/
                timer_Start(TIMER_Z, TIMERMAXVALUE);
                minput.in1 = _OFF;
                msensor.s0 = _OFF;
                msensor.s1 = _OFF;
                getInput = GET_SENSOR;
                cycleMeasureZ = SEN_START;
                cycleMeasure = MEASUREZ;
//                DBG("MEASURE Z\n");
            }
        };
        /*Measure Z*/
        while ((MEASUREZ == cycleMeasure) && (0 == GET_IN0))
        {
            msensor = io_getSensor();
            minput = io_getInput();
            if((SEN_START == cycleMeasureZ) && (_ON == msensor.s0))
            {
                /*Stop couter X*/
                mmeasureValue.Z = time_Stop(TIMER_Z);
                cycleMeasureZ = SEN_FINISH;
                getInput = GET_SENSOR;
                cycleMeasure = CHECKZVALUE;
                XStatus = SENSORCHANGE;
//                DBG("SENSOR Z = ON\n");
            }
            if(_ON == minput.in1) // C=2
            {
                if(SEN_START == cycleMeasureZ)
                {
                    mmeasureValue.Z = 0;
                }
                cycleMeasureZ = SEN_FINISH;
                getInput = GET_SENSOR;
                cycleMeasure = CHECKZVALUE;
//                DBG("C=2 END MEASURE Z\n");
            }
        };
        /********************************************## 5 ##*******************************************/

        while (((CHECKZVALUE == cycleMeasure) || (Z_OK == cycleMeasure) || (Z_NOT_OK == cycleMeasure)) && (0 == GET_IN0))
        {
            msensor = io_getSensor();
            minput = io_getInput();
            if((CHECKZVALUE == cycleMeasure) && (_ON == minput.in1) && (_ON == msensor.s0) && (_ON == msensor.s1)) //C=2
            {
                minput.in1 = _OFF;
                moutput.out1 = _ON;
                io_setOutput(moutput);
                app_GetCurrentMeasureValue();
                time_Stop(TIMER_Z);
                mmeasureValue.X1 =  mCurrentMeasureValue.X1;
                mmeasureValue.Y1 =  mCurrentMeasureValue.Y1;
                mmeasureValue.X2 =  mCurrentMeasureValue.X2;
                mmeasureValue.Y2 =  mCurrentMeasureValue.Y2;
                app_SetCurrentMeasureValue();
                mdata.mode = ZONLY;
                app_CalculatorValue(cycleMeasure, mdata.mode);
                screen_DataMeasure(mdata, msetCalibValue);
                cycleMeasure = Z_OK;
                getInput = GET_BUTTON;
                XStatus = SENSORCHANGE;
                YStatus = SENSORCHANGE;
//                DBG("C=2 MEASURE Z OK\n");
            }
            else if((CHECKZVALUE == cycleMeasure) && (_ON == minput.in1) && ((_OFF == msensor.s0) || (_OFF == msensor.s1))) //C=2
            {
                minput.in1 = _OFF;
                moutput.out1 = _OFF;
                io_setOutput(moutput);
                time_Stop(TIMER_Z);
                cycleMeasure = Z_NOT_OK;
                getInput = GET_SENSOR;
                mdata.mode = ZERROR1;
                screen_DataMeasure(mdata, msetCalibValue);
                //delay(100);
//                DBG("C=2 MEASURE Z NOT OK\n");
            }
            if(((Z_OK == cycleMeasure) || (Z_NOT_OK == cycleMeasure)) && (_OFF == minput.in1))
            {
                cycleMeasure = WAITMEASUREX1Y1;
//                DBG("C=2 cycleMeasure = WAITMEASUREX1Y1\n");
            }
        }

        if((SETVALUEZ == cycleMeasure) && (_ON == mbutton.set) && (_ON == moutput.out1))
        {
            mledStatus.led3 = _ON;
            io_setLedStatus(mledStatus);
            cycleMeasure = WAITMEASUREX1Y1;
            getInput = GET_SENSOR;
//            DBG("SET Z\n");
            app_SetCalibValue();
            app_GetCalibValue();
        }
        /********************************************## 6 ##*******************************************/
        /*Robot di chuyển tới vị trí P5 Robot xuất tín hiệu cho X11*/
        while (((WAITMEASUREX1Y1 == cycleMeasure) || (SETVALUEZ == cycleMeasure)) && (0 == GET_IN0))
        {
            minput = io_getInput();
            if(_ON == minput.in1) //C=3
            {
                /*Start couter*/
                timer_Start(TIMER_CLEARSENSOR, TIMEWAITX11);
                minput.in1 = _OFF;
                moutput.out0 = _OFF;
                moutput.out1 = _OFF;
                io_setOutput(moutput);
                cycleMeasure = WAITRBSTABLEX1Y1;
                getInput = GET_SENSOR;
                cycleMeasureX = SEN_START;
                cycleMeasureY = SEN_START;
//                DBG("cycleMeasure = MEASUREX1Y1 C=3\n");
            }
        };
        while ((WAITRBSTABLEX1Y1 == cycleMeasure) && (0 == GET_IN0))
        {
            if(TIME_FINISH == timer_Status(TIMER_CLEARSENSOR))
            {
                /*Start couter*/
                timer_Start(TIMER_X, TIMERMAXVALUE);
                timer_Start(TIMER_Y, TIMERMAXVALUE);
                minput.in1 = _OFF;
                cycleMeasure = MEASUREX1Y1;
                getInput = GET_SENSOR;
                cycleMeasureX = SEN_START;
                cycleMeasureY = SEN_START;
//                DBG("Start couter X1, Y1\n");
            }
        };
        while ((MEASUREX1Y1 == cycleMeasure) && (0 == GET_IN0))
        {
            msensor = io_getSensor();
            minput = io_getInput();
            if((SEN_START == cycleMeasureX) && (_ON == msensor.s0))
            {
                /*Stop couter X*/
                mmeasureValue.X1 = time_Stop(TIMER_X);
                msensor.s0 = _OFF;
                cycleMeasureX = SEN_FINISH;
                getInput = GET_SENSOR;
                XStatus = SENSORCHANGE;
                DBG("X1 = SEN_FINISH\n");
                // DBG("mmeasureValue.X1 = %d\n",mmeasureValue.X1);
            }
            if((SEN_START == cycleMeasureY) && (_ON == msensor.s1))
            {
                /*Stop couter X*/
                mmeasureValue.Y1 = time_Stop(TIMER_Y);
                msensor.s1 = _OFF;
                cycleMeasureY = SEN_FINISH;
                getInput = GET_SENSOR;
                YStatus = SENSORCHANGE;
                DBG("Y1 = SEN_FINISH\n");
                // DBG("mmeasureValue.Y1 = %d\n",mmeasureValue.Y1);
            }
            if((SEN_FINISH == cycleMeasureX) && (SEN_FINISH == cycleMeasureY))
            {
                cycleMeasureX = SEN_STOP;
                cycleMeasureY = SEN_STOP;
                cycleMeasure = WAITMEASUREX2Y2;
                getInput = GET_SENSOR;
//                DBG("cycleMeasure = WAITMEASUREX2Y2\n");
            }
            //if(_ON == minput.in1) //C=4
            //{
            //    if(SEN_START == cycleMeasureX)
            //    {
            //        mmeasureValue.X1 = 0;
            //    }
            //    if(SEN_START == cycleMeasureY)
            //    {
            //        mmeasureValue.Y1 = 0;
            //    }
            //    cycleMeasureX = SEN_STOP;
            //    cycleMeasureY = SEN_STOP;
            //    cycleMeasure = WAITMEASUREX2Y2;
            //    getInput = GET_SENSOR;
            //    (void)time_Stop(TIMER_X);
            //    (void)time_Stop(TIMER_Y);
            //    DBG("MEASUREX1Y1 == cycleMeasure C=4\n");
            //}
        };
        /********************************************## 7 ##*******************************************/
        /*Robot di chuyển tới vị trí P10 Robot xuất tín hiệu cho X11*/
        while ((WAITMEASUREX2Y2 == cycleMeasure) && (0 == GET_IN0))
        {
            minput = io_getInput();
            if(_ON == minput.in1) // C=4
            {
                /*Start couter*/
                timer_Start(TIMER_CLEARSENSOR, TIMEWAITX11);
                minput.in1 = _OFF;
                cycleMeasure = WAITRBSTABLEX2Y2;
                getInput = GET_SENSOR;
                cycleMeasureX = SEN_START;
                cycleMeasureY = SEN_START;
//                DBG("cycleMeasure = WAITMEASUREX2Y2 C=4\n");
            }
        };
        while ((WAITRBSTABLEX2Y2 == cycleMeasure) && (0 == GET_IN0))
        {
            if(TIME_FINISH == timer_Status(TIMER_CLEARSENSOR))
            {
                /*Start couter*/
                timer_Start(TIMER_X, TIMERMAXVALUE);
                timer_Start(TIMER_Y, TIMERMAXVALUE);
                minput.in1 = _OFF;
                cycleMeasure = MEASUREX2Y2;
                getInput = GET_SENSOR;
                cycleMeasureX = SEN_START;
                cycleMeasureY = SEN_START;
//                DBG("Start couter X2, Y2\n");
            }
        };
        while ((MEASUREX2Y2 == cycleMeasure) && (0 == GET_IN0))
        {
            msensor = io_getSensor();
            if((SEN_START == cycleMeasureX) && (_ON == msensor.s0))
            {
                /*Stop couter X*/
                mmeasureValue.X2 = time_Stop(TIMER_X);
                msensor.s0 = _OFF;
                cycleMeasureX = SEN_FINISH;
                getInput = GET_SENSOR;
                XStatus = SENSORCHANGE;
//                DBG("X2 = SEN_FINISH\n");
            }
            if((SEN_START == cycleMeasureY) && (_ON == msensor.s1))
            {
                /*Stop couter X*/
                mmeasureValue.Y2 = time_Stop(TIMER_Y);
                msensor.s1 = _OFF;
                cycleMeasureY = SEN_FINISH;
                getInput = GET_SENSOR;
                YStatus = SENSORCHANGE;
//                DBG("Y2 = SEN_FINISH\n");
            }
            if((SEN_FINISH == cycleMeasureX) && (SEN_FINISH == cycleMeasureY))
            {
                /*Save D_X1, D_Y1*/
                cycleMeasureX = SEN_STOP;
                cycleMeasureY = SEN_STOP;
                cycleMeasure = WAITSETVALUE;
                getInput = GET_BUTTON;
//                DBG("WAITSETVALUE == cycleMeasure\n");
            }
            // if(_ON == minput.in1) //C=5
            // {
            //     if(SEN_START == cycleMeasureX)
            //     {
            //         mmeasureValue.X2 = 0;
            //     }
            //     if(SEN_START == cycleMeasureY)
            //     {
            //         mmeasureValue.Y2 = 0;
            //     }
            //     cycleMeasureX = SEN_STOP;
            //     cycleMeasureY = SEN_STOP;
            //     cycleMeasure = CALCULATORVALUE;
            //     getInput = GET_SENSOR;
            //     (void)time_Stop(TIMER_X);
            //     (void)time_Stop(TIMER_Y);
            // }
        };

        /*Robot di chuyển tới vị trí P18*/
        if((WAITSETVALUE == cycleMeasure) && (_ON == mbutton.set))
        {
            app_GetCalibValue();
            if ((0 == mcalibValue.X1) && (0 == mcalibValue.X2) && (0 == mcalibValue.Y1) && (0 == mcalibValue.Y2) && (0 == mcalibValue.Z) && (ZERROR1 != mdata.mode))
            {
                app_SetCalibValue();
                app_GetCalibValue();
                mledStatus.led3 = _ON;
                msetCalibValue = CALIBSET;
                io_setLedStatus(mledStatus);
            }
            getInput = GET_SENSOR;
            cycleMeasure = CALCULATORVALUE;
//            DBG("cycleMeasure = CALCULATORVALUE\n");
        }
        /********************************************## 8 ##*******************************************/
        if(((CALCULATORVALUE == cycleMeasure) || (WAITSETVALUE == cycleMeasure)) && (_ON == minput.in1)) // C=5
        {
            minput.in1 = _OFF;
            cycleMeasure = FINISH;
            getInput = GET_BUTTON;
            /*Calculator Value - Printf to screen*/
            app_SetCurrentMeasureValue();
            if(ZONLY == mdata.mode)
            {
                mdata.mode = MEASUREALL;
            }
            else if(ZERROR1 == mdata.mode)
            {
                mdata.mode = ZERROR2;
            }
            app_CalculatorValue(cycleMeasure,  mdata.mode);
            screen_DataMeasure(mdata, msetCalibValue);
//            DBG("cycleMeasure = FINISH C=5\n");
        }
    } while (0 == GET_IN0);

    app_TrigerOutputOFF();
    moutput.out0 = _OFF;
    moutput.out1 = _OFF;
    moutput.rl1  = _OFF;
    moutput.rl2  = _OFF;

    if((SENSORNOCHANGE == XStatus) || (SENSORNOCHANGE == YStatus))
    {
        moutput.out1 = _ON;
//        DBG("SENSOR IS NOT CHANGE\n");
    }
    else
    {
        moutput.out1 = _OFF;
//        DBG("SENSOR CHANGE\n");
    }
    io_setOutput(moutput);
    app_TrigerOutputON();
    if((NONE != mdata.mode) && (CALIBSET == msetCalibValue))
    {
        eep_WriteDataMeasure(&mdata);
    }
}

static void app_ClearAllOutput(void)
{
    moutput.out0 = _OFF;
    moutput.out1 = _OFF;
    moutput.rl1 = _OFF;
    io_setOutput(moutput);
}

static void app_GetCurrentMeasureValue(void)
{
    (void)eep_ReadDataCurrent(&mCurrentMeasureValue);
//    DBG("mCurrentMeasureValue.X1 = %d\n",mCurrentMeasureValue.X1);
//    DBG("mCurrentMeasureValue.Y1 = %d\n",mCurrentMeasureValue.Y1);
//    DBG("mCurrentMeasureValue.X2 = %d\n",mCurrentMeasureValue.X2);
//    DBG("mCurrentMeasureValue.Y2 = %d\n",mCurrentMeasureValue.Y2);
//    DBG("mCurrentMeasureValue.Z = %d\n",mCurrentMeasureValue.Z);
}

static void app_SetCurrentMeasureValue(void)
{
    mCurrentMeasureValue.X1 = mmeasureValue.X1;
    mCurrentMeasureValue.Y1 = mmeasureValue.Y1;
    mCurrentMeasureValue.X2 = mmeasureValue.X2;
    mCurrentMeasureValue.Y2 = mmeasureValue.Y2;
    mCurrentMeasureValue.Z  = mmeasureValue.Z;
    (void)eep_WriteDataCurrent(&mCurrentMeasureValue);
//    DBG("mCurrentMeasureValue.X1 = %d\n",mCurrentMeasureValue.X1);
//    DBG("mCurrentMeasureValue.Y1 = %d\n",mCurrentMeasureValue.Y1);
//    DBG("mCurrentMeasureValue.X2 = %d\n",mCurrentMeasureValue.X2);
//    DBG("mCurrentMeasureValue.Y2 = %d\n",mCurrentMeasureValue.Y2);
//    DBG("mCurrentMeasureValue.Z = %d\n",mCurrentMeasureValue.Z);
}

static void app_CalculatorValue(CycleMeasure lcycleMeasures, uint8_t mode)
{
    double db_DetaTX1 = 0;
    double db_DetaTY1 = 0;
    double db_DetaTX2 = 0;
    double db_DetaTY2 = 0;
    double db_DetaTZ = 0;

    double  db_anpha1 = 0;
    double  db_beta1 = 0;
    double  db_anpha2 = 0;
    double  db_beta2 = 0;

    double db_Vrb1 = 20.0; /* mm/s */
    double db_Vrb2 = 20.0; /* mm/s */
    double db_R = 20.0;    /* mm */
    double db_D = 20.0;    /* mm */
    double db_a = 1.2;   /* mm */
    double db_b = 1.2;   /* mm */

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

    mdata.time = rtc_Now();
    if(CALIBSET == msetCalibValue)
    {
//        DBG("**********************************************************\n")
        /*calculator Z*/
        if((ZONLY == mode) || (MEASUREALL == mode))
        {
            db_DetaTZ  = ((double)(mmeasureValue.Z  - mcalibValue.Z))*100/1000000.0;  /* us/1000000 => s */
//            DBG("db_DetaTZ = %lf\n",db_DetaTZ);
            db_DetaZ = db_Vrb2 * db_DetaTZ;
//            DBG("db_DetaZ = %lf\n",db_DetaZ);
            mdata.coordinates.Z = (int16_t)(db_DetaZ * 100.0);
//            DBG("Z (db_DetaZ)= %lf\n",db_DetaZ);

            if((db_DetaZ > db_b) || (db_DetaZ < (-db_b)))
            {
                moutput.out3 = _ON;
            }
            else
            {
                moutput.out3 = _OFF;
            }
        }


        /*calculator X Y*/
        if (FINISH == lcycleMeasures)
        {
            db_DetaTX1 = ((double)(mmeasureValue.X1 - mcalibValue.X1))*100/1000000.0; /* us/1000000 => s */
            db_DetaTY1 = ((double)(mmeasureValue.Y1 - mcalibValue.Y1))*100/1000000.0; /* us/1000000 => s */
            db_DetaTX2 = ((double)(mmeasureValue.X2 - mcalibValue.X2))*100/1000000.0; /* us/1000000 => s */
            db_DetaTY2 = ((double)(mmeasureValue.Y2 - mcalibValue.Y2))*100/1000000.0; /* us/1000000 => s */
//            DBG("db_DetaTX1 = %lf\n",db_DetaTX1);
//            DBG("db_DetaTY1 = %lf\n",db_DetaTY1);
//            DBG("db_DetaTX2 = %lf\n",db_DetaTX2);
//            DBG("db_DetaTY2 = %lf\n",db_DetaTY2);

            db_anpha1 = (db_Vrb1*db_DetaTX1)/db_R;
            db_beta1  = (db_Vrb1*db_DetaTY1)/db_R;
            db_anpha2 = (db_Vrb2*db_DetaTX2)/db_R;
            db_beta2  = (db_Vrb2*db_DetaTY2)/db_R;
//            DBG("db_anpha1 = %lf\n",db_anpha1);
//            DBG("db_beta1 = %lf\n",db_beta1);
//            DBG("db_anpha2 = %lf\n",db_anpha2);
//            DBG("db_beta2 = %lf\n",db_beta2);

            db_DetaXSS1 = (2*db_R) * sin(db_anpha1/2) * cos(db_anpha1/2);
            db_DetaYSS1 = (2*db_R) * sin(db_beta1/2) * cos(db_beta1/2);
            db_DetaXRB1 = db_DetaXSS1 * cos(3.142/4);
            db_DetaYRB1 = db_DetaYSS1 * cos(3.142/4);
//            DBG("db_DetaXSS1 = %lf\n",db_DetaXSS1);
//            DBG("db_DetaYSS1 = %lf\n",db_DetaYSS1);
//            DBG("db_DetaXRB1 = %lf\n",db_DetaXRB1);
//            DBG("db_DetaYRB1 = %lf\n",db_DetaYRB1);

            db_DetaXSS2 = (2*db_R) * sin(db_anpha2/2) * cos(db_anpha2/2);
            db_DetaYSS2 = (2*db_R) * sin(db_beta2/2) * cos(db_beta2/2);
            db_DetaXRB2 = db_DetaXSS2 * cos(3.142/4);
            db_DetaYRB2 = db_DetaYSS2 * cos(3.142/4);
//            DBG("db_DetaXSS2 = %lf\n",db_DetaXSS2);
//            DBG("db_DetaYSS2 = %lf\n",db_DetaYSS2);
//            DBG("db_DetaXRB2 = %lf\n",db_DetaXRB2);
//            DBG("db_DetaYRB2 = %lf\n",db_DetaYRB2);

            db_r1 = sqrt((db_DetaYSS1 * db_DetaYSS1) + (db_DetaXSS1 * db_DetaXSS1));
            db_r2 = sqrt((db_DetaYSS2 * db_DetaYSS2) + (db_DetaXSS2 * db_DetaXSS2));
//            DBG("db_r1 = %lf\n",db_r1);
//            DBG("db_r2 = %lf\n",db_r2);

            db_LXSS = (atan((db_DetaXSS2 - db_DetaXSS1)/db_D)*180)/3.142;
            db_LYSS = (atan((db_DetaYSS2 - db_DetaYSS1)/db_D)*180)/3.142;
            db_LXRB = (atan((db_DetaXRB2 - db_DetaXRB1)/db_D)*180)/3.142;
            db_LYRB = (atan((db_DetaYRB2 - db_DetaYRB1)/db_D)*180)/3.142;
//            DBG("db_LXSS = %lf\n",db_LXSS);
//            DBG("db_LYSS = %lf\n",db_LYSS);
//            DBG("db_LXRB = %lf\n",db_LXRB);
//            DBG("db_LYRB = %lf\n",db_LYRB);

            mdata.coordinates.X = (int16_t)(db_DetaXRB2 * 100.0);
            mdata.coordinates.Y = (int16_t)(db_DetaYRB2 * 100.0);

            mdata.coordinates.R = (int16_t)(db_r2 * 100.0);
            mdata.coordinates.aX = (int16_t)(db_LXRB * 10.0);
            mdata.coordinates.aY = (int16_t)(db_LYRB * 10.0);
//            DBG("X (db_DetaXRB2) = %lf\n",db_DetaXRB2);
//            DBG("Y (db_DetaYRB2)= %lf\n",db_DetaYRB2);
//            DBG("R (db_r2)= %lf\n",db_r2);
//            DBG("A (db_LXRB) = %lf\n",db_LXRB);
//            DBG("B (db_LYRB)= %lf\n",db_LYRB);
//            DBG("X1 (db_DetaXRB1)= %lf\n",db_DetaXRB1);
//            DBG("Y1 (db_DetaYRB1)= %lf\n",db_DetaYRB1);
            if(db_r2 > db_a)
            {
                moutput.out2 = _ON;
            }
            else
            {
                moutput.out2 = _OFF;
            }
        }
        io_setOutput(moutput);
    }
}

static void app_SetCalibValue(void)
{
    mcalibValue.X1 = mmeasureValue.X1;
    mcalibValue.Y1 = mmeasureValue.Y1;
    mcalibValue.X2 = mmeasureValue.X2;
    mcalibValue.Y2 = mmeasureValue.Y2;
    mcalibValue.Z  = mmeasureValue.Z;
    eep_WriteDataCalib(&mcalibValue);
//    DBG("mcalibValue.X1 = %d\n",mcalibValue.X1);
//    DBG("mcalibValue.Y1 = %d\n",mcalibValue.Y1);
//    DBG("mcalibValue.X2 = %d\n",mcalibValue.X2);
//    DBG("mcalibValue.Y2 = %d\n",mcalibValue.Y2);
//    DBG("mcalibValue.Z = %d\n",mcalibValue.Z);
}

static void app_GetCalibValue(void)
{
    eep_ReadDataCalib(&mcalibValue);
//    DBG("mcalibValue.X1 = %d\n",mcalibValue.X1);
//    DBG("mcalibValue.Y1 = %d\n",mcalibValue.Y1);
//    DBG("mcalibValue.X2 = %d\n",mcalibValue.X2);
//    DBG("mcalibValue.Y2 = %d\n",mcalibValue.Y2);
//    DBG("mcalibValue.Z = %d\n",mcalibValue.Z);
}

static void app_TrigerOutputOFF(void)
{
    moutput.rl2 = _OFF;
    io_setOutput(moutput);
    delay(100);
}

static void app_TrigerOutputON(void)
{
    delay(100);
    moutput.rl2 = _ON;
    io_setOutput(moutput);
}

static optionScreen_e_t app_optionMenu(void)
{
	optionScreen_e_t optionIndex = measurement1Setting;
	uint8_t exit = 0;

	do {
		mbutton = io_getButton();
		if (mbutton.next == _ON) {
			while (_ON == io_getButton().next)
				;
			LCD_Clear();
			screen_OptionMenu(optionIndex);
			optionIndex++;
		}

		if (mbutton.prev == _ON) {
			while (_ON == io_getButton().prev)
				;
			LCD_Clear();
			optionIndex--;
			screen_OptionMenu(optionIndex);
		}

		if (mbutton.set == _ON) {
			while (_ON == io_getButton().set)
				;
			exit = 1;
		}

		if(mbutton.menu == _ON)
		{
			while (_ON == io_getButton().menu)
				;
			exit = 1;
		}

	} while (exit == 0);
	return optionIndex;
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
