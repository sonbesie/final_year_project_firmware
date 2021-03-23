/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020-2021 Pieter Schoeman.
  * All rights reserved.</center></h2>s
  *
  ****************** ************************************************************
  */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "network.h"
#include "camera.h"
#include "jfif.h"
#include "image.h"
#include "mis.h"
#include "mem.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TIME_001S 001
#define TIME_005S 005
#define TIME_030S 030
#define TIME_060S 060
#define TIME_120S 120

#define ERR_CNT_MAX 1														// maximum camera errors during one operation
#define ERR_TIMEOUT 5														// camera timeout in seconds

#define MODE_TEST 0															// test mode
#define MODE_RACE 1															// race mode
#define MODE_CALI 2															// cali mode (calibrate)

#define MAX_DETEC 4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

uint8_t cam_flag = 0, timeout_flag = 0;											// keep track of which camera to use next
uint8_t mode = MODE_TEST;														// default mode is the test mode
uint8_t quality = 0, baudrate = 8, resolution = 0;								// default settings
float cp = 0, st = 0, sp = 0;													// keep track of how long images are taken

uint16_t cam_1_det[MAX_DETEC] = {0}, cam_2_det[MAX_DETEC] = {0}, det_index = 0;	// motion detection
uint16_t cam_1_cal = 0, cam_2_cal = 0;											// calibration

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DAC_HandleTypeDef hdac;

ETH_HandleTypeDef heth;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

COM_InitTypeDef com1;
CAM_InitTypeDef cam1;
CAM_InitTypeDef cam2;
DEC_InitTypeDef dec1;
DEC_InitTypeDef dec2;
DEC_InitTypeDef dec2;
MIS_InitTypeDef mis1;
IMA_InitTypeDef ima1;
IMA_InitTypeDef ima2;
MEM_InitTypeDef mem1;
MEM_InitTypeDef mem2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

// hall callback functions
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *);
void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *);
void HAL_GPIO_EXTI_Callback(uint16_t);

// state functions
void state_00(uint8_t*);
void state_01(uint8_t*);
void state_02(uint8_t*);
void state_03(uint8_t*);
void state_04(uint8_t*);
void state_05(uint8_t*);
void state_06(uint8_t*);
void state_07(uint8_t*);
void state_08(uint8_t*);
void state_09(uint8_t*);
void state_10(uint8_t*);

// program flow functions
void wait_for_interrupt(void);

// image analysis functions
void mode_race(void);
void mode_cali(void);

// error functions
void Error_Handler(void);

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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_DAC_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  //----------------------------------------------------------------------------
  // place system in test mode
  mode = MODE_TEST;
  //----------------------------------------------------------------------------

  //----------------------------------------------------------------------------
  // initialise the seven segment display
  mis1.Status = 15;
  DecodeSevenSegment(&mis1);
  //----------------------------------------------------------------------------

  //----------------------------------------------------------------------------
  // initialise the trigger sensitivity
  mis1.Dacx = &hdac;
  InitialiseTrigger(&mis1);
  //----------------------------------------------------------------------------

  //----------------------------------------------------------------------------
  // initialise and erase memory
  mem1.Spi = &hspi1;
  mem1.MemoryCardBank = GPIOE;
  mem1.MemoryCardPin1 = GPIO_PIN_10;
  mem1.MemoryCardPin2 = GPIO_PIN_12;
  InitMem(&mem1);

  mem2.Spi = &hspi2;
  mem2.MemoryCardBank = GPIOF;
  mem2.MemoryCardPin1 = GPIO_PIN_1 ;
  mem2.MemoryCardPin2 = GPIO_PIN_2 ;
  InitMem(&mem2);

  // test memory cards
  if ((CardTest (&mem1) != MEM_OK) || (CardTest (&mem2) != MEM_OK))
  {
	  Error_Handler();
	  // system reset
	  NVIC_SystemReset();
  }

  // erase memory cards
  if ((CardErase(&mem1) != MEM_OK) || (CardErase(&mem2) != MEM_OK))
  {
	  Error_Handler();
	  // system reset
	  NVIC_SystemReset();
  }
  //----------------------------------------------------------------------------

  //---------------------------------------------------------------------------
  // initialise the cameras
  cam1.Settings.CameraAddress = CAMERA_1;
  cam1.Settings.BaudRate = BAUD_921600;
  cam1.Uart = &huart1;
  if (InitCam(&cam1) != CAM_OK)
  {
	  Error_Handler();
	  // system reset
	  NVIC_SystemReset();
  }

  cam2.Settings.CameraAddress = CAMERA_2;
  cam2.Settings.BaudRate = BAUD_921600;
  cam2.Uart = &huart2;
  if (InitCam(&cam2) != CAM_OK)
  {
	  Error_Handler();
	  // system reset
	  NVIC_SystemReset();
  }
  //----------------------------------------------------------------------------

  //----------------------------------------------------------------------------
  // turn the cameras on
  StartCamera(&cam1);
  StartCamera(&cam2);
  // wait for data read to complete
  wait_for_interrupt();
  // turn the cameras off
  StopCamera(&cam1);
  StopCamera(&cam2);
  //----------------------------------------------------------------------------

  //----------------------------------------------------------------------------
  // initialise the communication system
  com1.Heth = &heth;
  // destination mac address
  com1.Settings.DestinMAC[0] = 0xFF;
  com1.Settings.DestinMAC[1] = 0xFF;
  com1.Settings.DestinMAC[2] = 0xFF;
  com1.Settings.DestinMAC[3] = 0xFF;
  com1.Settings.DestinMAC[4] = 0xFF;
  com1.Settings.DestinMAC[5] = 0xFF;
  // source mac address
  com1.Settings.SourceMAC[0] = heth.Init.MACAddr[0];
  com1.Settings.SourceMAC[1] = heth.Init.MACAddr[1];
  com1.Settings.SourceMAC[2] = heth.Init.MACAddr[2];
  com1.Settings.SourceMAC[3] = heth.Init.MACAddr[3];
  com1.Settings.SourceMAC[4] = heth.Init.MACAddr[4];
  com1.Settings.SourceMAC[5] = heth.Init.MACAddr[5];
  InitCom(&com1);
  //----------------------------------------------------------------------------

  //----------------------------------------------------------------------------
  InitDec(&dec1);
  InitDec(&dec2);
  //----------------------------------------------------------------------------

  //----------------------------------------------------------------------------
  InitIma(&ima1);
  InitIma(&ima2);
  //----------------------------------------------------------------------------

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
  heth.Init.MACAddr[0] =   0x00;
  heth.Init.MACAddr[1] =   0x80;
  heth.Init.MACAddr[2] =   0xE1;
  heth.Init.MACAddr[3] =   0x00;
  heth.Init.MACAddr[4] =   0x00;
  heth.Init.MACAddr[5] =   0x00;
  heth.Init.RxMode = ETH_RXINTERRUPT_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  htim1.Init.Prescaler = 21600-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
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
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart2.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart3.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart3.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, Relay_1_Pin|Relay_2_Pin|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Trigger_Set_Reset_Pin|S7_1_Pin|S7_4_Pin|S7_5_Pin
                          |S7_3_Pin|S7_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF1 PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE10 PE12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Relay_1_Pin Relay_2_Pin USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = Relay_1_Pin|Relay_2_Pin|USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Trigger_Set_Reset_Pin */
  GPIO_InitStruct.Pin = Trigger_Set_Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Trigger_Set_Reset_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Trigger_Output_Pin */
  GPIO_InitStruct.Pin = Trigger_Output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Trigger_Output_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S7_1_Pin S7_4_Pin S7_5_Pin S7_3_Pin
                           S7_2_Pin */
  GPIO_InitStruct.Pin = S7_1_Pin|S7_4_Pin|S7_5_Pin|S7_3_Pin
                          |S7_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

//----------------------------------------------------------------------------

/**
  * @brief Temp
  * @retval None
  */
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
	// this function is responsible for setting the machine state
	if (HAL_ETH_GetReceivedFrame_IT(heth) == HAL_OK)
	{
		uint8_t * addr = (uint8_t *)heth->RxDesc->Buffer1Addr;
		if ((addr[12] == 0xEE) && (addr[13] == 0xFA))
		{
			// save buffer
			uint8_t * buffer = calloc(1514, sizeof(uint8_t));
			memcpy(buffer, (uint8_t*)heth->RxDesc->Buffer1Addr, 1514);

			switch (buffer[14])
			{
				// arm system
				case 0x40:
				{
					// state 00
					mis1.Status = 0;
					DecodeSevenSegment(&mis1);
					state_00(buffer);
					break;
				}
				// disarm system
				case 0x41:
				{
					// state 01
					mis1.Status = 1;
					DecodeSevenSegment(&mis1);
					state_01(buffer);
					break;
				}
				// receive universal settings
				case 0x42:
				{
					// state 02
					mis1.Status = 2;
					DecodeSevenSegment(&mis1);
					state_02(buffer);
					break;
				}

				// capture an image
				case 0x43:
				{
					// state 03
					mis1.Status = 3;
					DecodeSevenSegment(&mis1);
					state_03(buffer);
					break;
				}

				// frame rate test
				case 0x44:
				{
					// state 04
					mis1.Status = 4;
					DecodeSevenSegment(&mis1);
					state_04(buffer);
					break;
				}

				// analyse dc values of captured photos
				case 0x45:
				{
					// state 05
					mis1.Status = 5;
					DecodeSevenSegment(&mis1);
					state_05(buffer);
					break;
				}

				// return all captured images starting and ending at a given index
				case 0x46:
				{
					// state 06
					mis1.Status = 6;
					DecodeSevenSegment(&mis1);
					state_06(buffer);
					break;
				}

				// check alive ping
				case 0x47:
				{
					// state 07
					mis1.Status = 7;
					DecodeSevenSegment(&mis1);
					state_07(buffer);
					break;
				}

				// new device added
				case 0x48:
				{
					// state 08
					mis1.Status = 8;
					DecodeSevenSegment(&mis1);
					state_08(buffer);
					break;
				}

				// toggle cameras
				case 0x49:
				{
					// state 09
					mis1.Status = 9;
					DecodeSevenSegment(&mis1);
					state_09(buffer);
					break;
				}
				case 0x50:
				{
					// state 10
					mis1.Status = 0;
					DecodeSevenSegment(&mis1);
					state_10(buffer);
					break;
				}
			}
			// free buffer
			free(buffer);
		}
		// set 'own' bit in rx descriptors: gives the buffers back to dma
		heth->RxDesc->Status |= ETH_DMARXDESC_OWN;
		// clear segment_count
		heth->RxFrameInfos.SegCount = 0;
	}
}

/**
  * @brief Temp
  * @retval None
  */
void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *heth)
{
	while (1)
	{
		break;
	}
}

/**
  * @brief Temp
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_7)
	{
		// if the system is armed, wait for the starting signal to start the race
		uint8_t Preamble[5], Data[10];
		RTC_DateTypeDef Date;
		RTC_TimeTypeDef Time;

		HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BCD);
		HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BCD);

		// preamble
		Preamble[0] = 0x5B;
		Preamble[1] = 0x40;
		Preamble[2] = 0b00000000;
		Preamble[3] = 0x00;
		Preamble[4] = 0x00;

		// time and date
		Data[0] = ((Date.Year   /16)*10)+(Date.Year   %16);
		Data[1] = ((Date.Month  /16)*10)+(Date.Month  %16);
		Data[2] = ((Date.Date   /16)*10)+(Date.Date   %16);
		Data[3] = ((Time.Hours  /16)*10)+(Time.Hours  %16);
		Data[4] = ((Time.Minutes/16)*10)+(Time.Minutes%16);
		Data[5] = ((Time.Seconds/16)*10)+(Time.Seconds%16);

		// split seconds
		Data[6] = (Time.SubSeconds & 0b11111111000000000000000000000000) >> 24;
		Data[7] = (Time.SubSeconds & 0b00000000111111110000000000000000) >> 16;
		Data[8] = (Time.SubSeconds & 0b00000000000000001111111100000000) >>  8;
		Data[9] = (Time.SubSeconds & 0b00000000000000000000000011111111) >>  0;


		// answer with an acknowledge
		TransmitData(&com1, Preamble, 5, Data, 10);

		// disarm
		mis1.Armed = 0;
		SetTriggerArmedStatus(&mis1);
	}
	else
	{
		//
	}
}

/**
  * @brief Temp
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)		// camera 1
	{
		for (int index = 0; index < 6; index++)
		{
			if (cam1.ImageData[index] != 0)		// check if data exists
			{
				// image taken (index 0)
				const uint8_t Success[3] = {0x76, 0x00, 0x32};
				if (ContainsResult(cam1.ImageData[index], cam1.ImageLength[index]+10, Success, 3) == 1)
				{
					// store image in memory
					if (StoreImage(&mem1, cam1.ImageData[index], cam1.ImageLength[index]+10, mem1.FileSystemCounter, cam1.Date[index], cam1.Time[index]) == MEM_OK)
						++mem1.FileSystemCounter;
					// free memory
					free(cam1.ImageData[index]);
					cam1.ImageData[index] = 0;
					cam1.ImageLength[index] = 0;
					// flags
					cam1.Done = 1;
				}
				break;
			}
		}
		if (cam1.InteruptResult != 0)	// check if data exists
		{
			// receive startup notification
			// look for the word end
			const uint8_t Success[] = "end";
			if (ContainsResult(cam1.InteruptResult, cam1.InteruptResultLength, Success, 3) == 1)
			{
				// free memory
				free(cam1.InteruptResult);
				cam1.InteruptResult = 0;
				cam1.InteruptResultLength = 0;
				// flags
				cam1.Done = 1;
			}
		}
	}
	else
	if (huart->Instance == USART2)		// camera 2
	{
		for (int index = 0; index < 6; index++)
		{
			if (cam2.ImageData[index] != 0)		// check if data exists
			{
				// image taken (index 0)
				const uint8_t Success[3] = {0x76, 0x01, 0x32};
				if (ContainsResult(cam2.ImageData[index], cam2.ImageLength[index]+10, Success, 3) == 1)
				{
					// store image in memory
					if (StoreImage(&mem2, cam2.ImageData[index], cam2.ImageLength[index]+10, mem2.FileSystemCounter, cam2.Date[index], cam2.Time[index]) == MEM_OK)
						++mem2.FileSystemCounter;
					// free memory
					free(cam2.ImageData[index]);
					cam2.ImageData[index] = 0;
					cam2.ImageLength[index] = 0;
					// flags
					cam2.Done = 1;
				}
				break;
			}
		}
		if (cam2.InteruptResult != 0)	// check if data exists
		{
			// receive startup notification
			// look for the word end
			const uint8_t Success[] = "end";
			if (ContainsResult(cam2.InteruptResult, cam2.InteruptResultLength, Success, 3) == 1)
			{
				// free memory
				free(cam2.InteruptResult);
				cam2.InteruptResult = 0;
				cam2.InteruptResultLength = 0;
				// flags
				cam2.Done = 1;
			}
		}
	}
	else
	if (huart->Instance == USART3)	// computer usb connection
	{
		//
	}
}

/**
  * @brief Temp
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
		// determine elapsed time
		sp = HAL_GetTick();

		if ((((sp-st)/1000.0) > cp) && (cam1.Done == 1) && (cam2.Done == 1))
		{
			// stop the timer
			HAL_TIM_Base_Stop_IT(&htim1);

			// stop capture
			StopCapture(&cam1);
			StopCapture(&cam2);

			// stop cameras
			StopCamera(&cam1);
			StopCamera(&cam2);

			// race mode
			if (mode == MODE_RACE) mode_race();

			// cali mode
			if (mode == MODE_CALI) mode_cali();

			// place into race mode and reset camera selection
			mode = MODE_TEST;
			cam_flag = 1;

			// activate the timeout flag
			timeout_flag = 1;
		}
		else
		{
			switch (mode)
			{
			case MODE_RACE:

				// wait
				while (cam1.Done == 0) {}
				while (cam2.Done == 0) {}

				// camera 1
				if (CaptureImage(&cam1, &hrtc) == CAM_OK)
				{
					ReadImageDataLength(&cam1, 0);
					DetectsMotion(&ima1, cam1.ImageLength[0]);
					if ((ima1.Motion.Motion == 1) && (det_index < MAX_DETEC))
					{
						// save location
						cam_1_det[det_index] = mem1.FileSystemCounter;
						cam_2_det[det_index] = mem2.FileSystemCounter;
						det_index++;

						ReadImageDataLength(&cam1, 0); ReadImageData(&cam1, 0);
						while (cam1.Done == 0) {}

						ReadImageDataLength(&cam2, 0); ReadImageData(&cam2, 0);
						while (cam2.Done == 0) {}

						CaptureMultipleImages(&cam1, &hrtc, 1, 5);
						CaptureMultipleImages(&cam2, &hrtc, 1, 5);

						// read camera 1
						for (int index = 1; index < 6; index++)
						{
							while (cam1.Done == 0) {}
							ReadImageDataLength(&cam1, index);
							ReadImageData(&cam1, index);
						}

						while (cam1.Done == 0) {}

						// read camera 2
						for (int index = 1; index < 6; index++)
						{
							while (cam2.Done == 0) {}
							ReadImageDataLength(&cam2, index);
							ReadImageData(&cam2, index);
						}

						while (cam2.Done == 0) {}

						// clear
						ima1.Motion.Motion = 0;
						break;
					}
				}

				// camera 2
				if (CaptureImage(&cam2, &hrtc) == CAM_OK)
				{
					ReadImageDataLength(&cam2, 0);
					DetectsMotion(&ima2, cam2.ImageLength[0]);
					if ((ima2.Motion.Motion == 1)  && (det_index < MAX_DETEC))
					{
						// save location
						cam_1_det[det_index] = mem1.FileSystemCounter;
						cam_2_det[det_index] = mem2.FileSystemCounter;
						det_index++;

						ReadImageDataLength(&cam1, 0); ReadImageData(&cam1, 0);
						while (cam1.Done == 0) {}

						ReadImageDataLength(&cam2, 0); ReadImageData(&cam2, 0);
						while (cam2.Done == 0) {}

						CaptureMultipleImages(&cam1, &hrtc, 1, 5);
						CaptureMultipleImages(&cam2, &hrtc, 1, 5);

						// read camera 1
						for (int index = 1; index < 6; index++)
						{
							while (cam1.Done == 0) {}
							ReadImageDataLength(&cam1, index);
							ReadImageData(&cam1, index);
						}

						while (cam1.Done == 0) {}

						// read camera 2
						for (int index = 1; index < 6; index++)
						{
							while (cam2.Done == 0) {}
							ReadImageDataLength(&cam2, index);
							ReadImageData(&cam2, index);
						}

						while (cam2.Done == 0) {}

						// clear
						ima2.Motion.Motion = 0;
						break;
					}
				}

				break;
			case MODE_CALI:

				// camera 1
				if ((cam_flag == 0) && (cam2.Done == 1))
				{
					if (CaptureMultipleImages(&cam1, &hrtc, 1, 5) == CAM_OK)
					{
						for (int index = 1; index < 6; index++)
						{
							while (cam1.Done == 0) {}
							ReadImageDataLength(&cam1, index);
							ReadImageData(&cam1, index);
							++cam1.ImageCounter;
						}

						// change camera
						cam_flag = 1;
					}
				}
				else
				// camera 2
				if ((cam_flag == 1) && (cam1.Done == 1))
				{
					if (CaptureMultipleImages(&cam2, &hrtc, 1, 5) == CAM_OK)
					{
						for (int index = 1; index < 6; index++)
						{
							while (cam2.Done == 0) {}
							ReadImageDataLength(&cam2, index);
							ReadImageData(&cam2, index);
							++cam2.ImageCounter;
						}

						// change camera
						cam_flag = 0;
					}
				}
				break;
			case MODE_TEST:

				// read data from camera 1
				if ((cam_flag == 0) && (cam2.Done == 1))
				{
					if (CaptureMultipleImages(&cam1, &hrtc, 1, 5) == CAM_OK)
					{
						for (int index = 1; index < 6; index++)
						{
							while (cam1.Done == 0) {}
							ReadImageDataLength(&cam1, index);
							ReadImageData(&cam1, index);
							++cam1.ImageCounter;
						}

						// change camera
						cam_flag = 1;
					}
				}
				else
				// read data from camera 2
				if ((cam_flag == 1) && (cam1.Done == 1))
				{
					if (CaptureMultipleImages(&cam2, &hrtc, 1, 5) == CAM_OK)
					{
						for (int index = 1; index < 6; index++)
						{
							while (cam2.Done == 0) {}
							ReadImageDataLength(&cam2, index);
							ReadImageData(&cam2, index);
							++cam2.ImageCounter;
						}

						// change camera
						cam_flag = 0;
					}
				}
				break;
			}
			timeout_flag = 0;
		}
	}
}

/**
  * @brief Arms the system to enable it to respond to the starting pistol. Transmits result
  * over an Ethernet connection back to the controller.
  * @retval None
  */
void state_00(uint8_t *a)
{
	uint8_t Preamble[5], Data[1];

	// arm
	mis1.Armed = 1;
	SetTriggerArmedStatus(&mis1);

	// preamble
	Preamble[0] = 0x5A;
	Preamble[1] = 0x40;
	Preamble[2] = 0b00000000;
	Preamble[3] = 0x00;
	Preamble[4] = 0x00;

	// response
	Data[0] = 0x00;

	// stop cameras
	StopCamera(&cam1);
	StopCamera(&cam2);

	// answer with an acknowledge
	TransmitData(&com1, Preamble, 5, Data, 1);
}

/**
  * @brief Disarms the system to stop it from responding to the starting pistol. Transmits result
  * over an Ethernet connection back to the controller.
  * @retval None
  */
void state_01(uint8_t *a)
{
	uint8_t Preamble[5], Data[1];

	// disarm
	mis1.Armed = 0;
	SetTriggerArmedStatus(&mis1);

	// preamble
	Preamble[0] = 0x5A;
	Preamble[1] = 0x41;
	Preamble[2] = 0b00000000;
	Preamble[3] = 0x00;
	Preamble[4] = 0x00;

	// response
	Data[0] = 0x00;

	// stop cameras
	StopCamera(&cam1);
	StopCamera(&cam2);

	// answer with an acknowledge
	TransmitData(&com1, Preamble, 5, Data, 1);
}

/**
  * @brief Applies the user selected settings received over an Ethernet connection to the cameras
  * and other board functions. Transmits result over an Ethernet connection back to the controller.
  * @retval None
  */
void state_02(uint8_t *a)
{
	uint8_t Preamble[5], Data[1];

	int CameraResult = 0;
	int ErrorCounter = 0;

	// declare date and time variables
	RTC_TimeTypeDef time = {0};
	RTC_DateTypeDef date = {0};

	// preamble
	Preamble[0] = 0x5A;
	Preamble[1] = 0x42;
	Preamble[2] = 0b00000000;
	Preamble[3] = 0x00;
	Preamble[4] = 0x00;

	// response
	Data[0] = 0x00;

	if (resolution != (int)a[15])
	{
		resolution = (int)a[15];
		// set the camera resolutions
		int res = 0;
		switch ((int)a[15])
		{
			case 1:		// 0160x0120
			{
				res = RES_0160_0120;
				break;
			}
			case 2:		// 0320x0240
			{
				res = RES_0320_0240;
				break;
			}
			case 3:		// 0640x0480
			{
				res = RES_0640_0480;
				break;
			}
			case 4:		// 1024x0768
			{
				res = RES_1024_0768;
				break;
			}
			case 5:		// 1280x0720
			{
				res = RES_1280_0720;
				break;
			}
			case 6:		// 1280x0960
			{
				res = RES_1280_0960;
				break;
			}
			case 7:		// 1920x1080
			{
				res = RES_1920_1080;
				break;
			}
			default:
			{
				res = RES_0160_0120;
				break;
			}
		}

		// turn the cameras on
		StartCamera(&cam1);
		StartCamera(&cam2);
		// wait for data read to complete
		wait_for_interrupt();

		// set the camera resolutions: camera 1
		ErrorCounter = 0;
		while (ErrorCounter < ERR_CNT_MAX)
		{
			CameraResult = SetImageResolution(&cam1, res);
			if (CameraResult != CAM_OK)
			{
				// new response
				Preamble[2] = Preamble[2] | 0b00000001;
				// increase error counter
				ErrorCounter++;
				// error handler
				Error_Handler();
			}
			else
			{
				Preamble[2] = Preamble[2] & 0b11111110;
				break;
			}
		}

		// set the camera resolutions: camera 2
		ErrorCounter = 0;
		while (ErrorCounter < ERR_CNT_MAX)
		{
			CameraResult = SetImageResolution(&cam2, res);
			if (CameraResult != CAM_OK)
			{
				// new response
				Preamble[2] = Preamble[2] | 0b00000010;
				// increase error counter
				ErrorCounter++;
				// error handler
				Error_Handler();
			}
			else
			{
				Preamble[2] = Preamble[2] & 0b11111101;
				break;
			}
		}

		// stop cameras
		StopCamera(&cam1);
		StopCamera(&cam2);
	}

	if (baudrate != (int)a[16])
	{
		baudrate = (int)a[16];
		// set the camera baud rates
		int baud = 0;
		switch ((int)a[16])
		{
			case 1:		// 009600
			{
				baud = BAUD_009600;
				break;
			}
			case 2:		// 019200
			{
				baud = BAUD_019200;
				break;
			}
			case 3:		// 038400
			{
				baud = BAUD_038400;
				break;
			}
			case 4:		// 057600
			{
				baud = BAUD_057600;
				break;
			}
			case 5:		// 115200
			{
				baud = BAUD_115200;
				break;
			}
			case 6:		// 230400
			{
				baud = BAUD_230400;
				break;
			}
			case 7:		// 460800
			{
				baud = BAUD_460800;
				break;
			}
			case 8:		// 921600
			{
				baud = BAUD_921600;
				break;
			}
			default:
			{
				baud = BAUD_921600;
				break;
			}
		}

		// turn the cameras on
		StartCamera(&cam1);
		StartCamera(&cam2);
		// wait for data read to complete
		wait_for_interrupt();

		// set the camera baud rates: camera 1
		ErrorCounter = 0;
		while (ErrorCounter < ERR_CNT_MAX)
		{
			CameraResult = SetCameraBaud(&cam1, baud);
			if (CameraResult != CAM_OK)
			{
				// new response
				Preamble[2] = Preamble[2] | 0b00000100;
				// increase error counter
				ErrorCounter++;
				// error handler
				Error_Handler();
			}
			else
			{
				Preamble[2] = Preamble[2] & 0b11111011;
				break;
			}
		}

		// set the camera baud rates: camera 2
		ErrorCounter = 0;
		while (ErrorCounter < ERR_CNT_MAX)
		{
			CameraResult = SetCameraBaud(&cam2, baud);
			if (CameraResult != CAM_OK)
			{
				// new response
				Preamble[2] = Preamble[2] | 0b00001000;
				// increase error counter
				ErrorCounter++;
				// error handler
				Error_Handler();
			}
			else
			{
				Preamble[2] = Preamble[2] & 0b11110111;
				break;
			}
		}

		// stop cameras
		StopCamera(&cam1);
		StopCamera(&cam2);
	}

	if (quality != (int)a[17])
	{
		quality = (int)a[17];
	}

	// set trigger sensitivity
	mis1.Sensitivity = a[18];
	SetTriggerSensitivity(&mis1);

	// the current time array
	time.Hours   = a[19];
	time.Minutes = a[20];
	time.Seconds = a[21];

	// the current date array
	date.Year    = a[22];
	date.Month   = a[23];
	date.Date    = a[24];

	// update the rtc time
	ErrorCounter = 0;
	while (ErrorCounter < ERR_CNT_MAX)
	{
		CameraResult = HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BCD);
		if (CameraResult != HAL_OK)
		{
			// new response
			Preamble[2] = Preamble[2] | 0b01000000;
			// increase error counter
			ErrorCounter++;
			// error handler
			Error_Handler();
		}
		else
		{
			Preamble[2] = Preamble[2] & 0b10111111;
			break;
		}
	}

	// update the rtc date
	ErrorCounter = 0;
	while (ErrorCounter < ERR_CNT_MAX)
	{
		CameraResult = HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BCD);
		if (CameraResult != HAL_OK)
		{
			// new response
			Preamble[2] = Preamble[2] | 0b10000000;
			// increase error counter
			ErrorCounter++;
			// error handler
			Error_Handler();
		}
		else
		{
			Preamble[2] = Preamble[2] & 0b01111111;
			break;
		}
	}

	// answer with an acknowledge
	TransmitData(&com1, Preamble, 5, Data, 1);
}

/**
  * @brief Test image function. Takes images for ten seconds at the maximum frame rate. Transmits
  * captured images through an Ethernet connection back to the controller.
  * @retval None
  */
void state_03(uint8_t *a)
{
	uint8_t Preamble[15], Data[1];

	// set mode
	mode = MODE_TEST;

	// start cameras
	StartCamera(&cam1);
	StartCamera(&cam2);
	// wait for data read to complete
	wait_for_interrupt();

	SetImageCompressionRatio(&cam1, quality);
	SetImageCompressionRatio(&cam2, quality);

	// clear the image counters
	cam1.ImageCounter = 0;
	cam2.ImageCounter = 0;

	// store image locations
	int loc1 = mem1.FileSystemCounter;
	int loc2 = mem2.FileSystemCounter;




	//----------------------------------------------------------------------------
	// save start time
	st = HAL_GetTick();
	cp = TIME_005S;

	// start the timer
	HAL_TIM_Base_Start_IT(&htim1);

	// blocking mode, wait for sequence to finish
	while (timeout_flag == 0) {}
	timeout_flag = 0;
	//----------------------------------------------------------------------------

	// return data from memory card 1
	for (uint32_t index = loc1; index < mem1.FileSystemCounter; index++)
	{
		if ((RecalImage(&mem1, index)) != MEM_OK)
		{
			Error_Handler();
		}

		// preamble
		Preamble[0 ] = 0xF0|((index & 0b111111110000000000000000) >> 16);
		Preamble[1 ] = 0x00|((index & 0b000000001111111111111111)      );
		Preamble[2 ] = 0b00000000;
		Preamble[3 ] = 0x00;
		Preamble[4 ] = 0x00;

		Preamble[5 ] = mem1.Date.Year 	;
		Preamble[6 ] = mem1.Date.Month	;
		Preamble[7 ] = mem1.Date.Date 	;

		Preamble[8 ] = mem1.Time.Hours	;
		Preamble[9 ] = mem1.Time.Minutes;
		Preamble[10] = mem1.Time.Seconds;

		Preamble[11] = (mem1.Time.SubSeconds & 0b11111111000000000000000000000000) >> 24;
		Preamble[12] = (mem1.Time.SubSeconds & 0b00000000111111110000000000000000) >> 16;
		Preamble[13] = (mem1.Time.SubSeconds & 0b00000000000000001111111100000000) >>  8;
		Preamble[14] = (mem1.Time.SubSeconds & 0b00000000000000000000000011111111) >>  0;

		TransmitData(&com1, Preamble, 15, &mem1.Data[0], mem1.DataLength+10);
		// free memory
		free(mem1.Data);
		mem1.Data = NULL;
		mem1.DataLength = 0;
	}

	// return data from memory card 2
	for (uint32_t index = loc2; index < mem2.FileSystemCounter; index++)
	{
		if ((RecalImage(&mem2, index)) != MEM_OK)
		{
			Error_Handler();
		}

		// preamble
		Preamble[0] = 0xE0|((index & 0b111111110000000000000000) >> 16);
		Preamble[1] = 0x00|((index & 0b000000001111111111111111)      );
		Preamble[2] = 0b00000000;
		Preamble[3] = 0x00;
		Preamble[4] = 0x00;

		Preamble[5 ] = mem2.Date.Year 	;
		Preamble[6 ] = mem2.Date.Month	;
		Preamble[7 ] = mem2.Date.Date 	;

		Preamble[8 ] = mem2.Time.Hours	;
		Preamble[9 ] = mem2.Time.Minutes;
		Preamble[10] = mem2.Time.Seconds;

		Preamble[11] = (mem2.Time.SubSeconds & 0b11111111000000000000000000000000) >> 24;
		Preamble[12] = (mem2.Time.SubSeconds & 0b00000000111111110000000000000000) >> 16;
		Preamble[13] = (mem2.Time.SubSeconds & 0b00000000000000001111111100000000) >>  8;
		Preamble[14] = (mem2.Time.SubSeconds & 0b00000000000000000000000011111111) >>  0;

		TransmitData(&com1, Preamble, 15, &mem2.Data[0], mem2.DataLength+10);
		// free memory
		free(mem2.Data);
		mem2.Data = NULL;
		mem2.DataLength = 0;
	}

	// preamble
	Preamble[0] = 0x5A;
	Preamble[1] = 0x43;
	Preamble[2] = 0b00000000;
	Preamble[3] = 0x00;
	Preamble[4] = 0x00;

	// response
	Data[0] = 0x00;

	// answer with an acknowledge
	TransmitData(&com1, Preamble, 5, Data, 1);
}

/**
  * @brief Frame rate test function. Takes images for ten seconds at the maximum frame rate. Transmits
  * captured images and the calculated frame rate through an Ethernet connection back to the controller.
  * @retval None
  */
void state_04(uint8_t *a)
{
	uint8_t Preamble[15], Data[5];

	// set mode
	mode = MODE_TEST;

	// start cameras
	StartCamera(&cam1);
	StartCamera(&cam2);
	// wait for data read to complete
	wait_for_interrupt();

	SetImageCompressionRatio(&cam1, quality);
	SetImageCompressionRatio(&cam2, quality);

	// clear the image counters
	cam1.ImageCounter = 0;
	cam2.ImageCounter = 0;

	// store image locations
	int loc1 = mem1.FileSystemCounter;
	int loc2 = mem2.FileSystemCounter;




	//----------------------------------------------------------------------------
	// save start time
	st = HAL_GetTick();
	cp = TIME_005S;

	// start the timer
	HAL_TIM_Base_Start_IT(&htim1);

	// blocking mode, wait for sequence to finish
	while (timeout_flag == 0) {}
	timeout_flag = 0;
	//----------------------------------------------------------------------------

	// return data from memory card 1
	for (uint32_t index = loc1; index < mem1.FileSystemCounter; index++)
	{
		if ((RecalImage(&mem1, index)) != MEM_OK)
		{
			Error_Handler();
		}

		// preamble
		Preamble[0] = 0xF0|((index & 0b111111110000000000000000) >> 16);
		Preamble[1] = 0x00|((index & 0b000000001111111111111111)      );
		Preamble[2] = 0b00000000;
		Preamble[3] = 0x00;
		Preamble[4] = 0x00;

		Preamble[5 ] = mem1.Date.Year 	;
		Preamble[6 ] = mem1.Date.Month	;
		Preamble[7 ] = mem1.Date.Date 	;

		Preamble[8 ] = mem1.Time.Hours	;
		Preamble[9 ] = mem1.Time.Minutes;
		Preamble[10] = mem1.Time.Seconds;

		Preamble[11] = (mem1.Time.SubSeconds & 0b11111111000000000000000000000000) >> 24;
		Preamble[12] = (mem1.Time.SubSeconds & 0b00000000111111110000000000000000) >> 16;
		Preamble[13] = (mem1.Time.SubSeconds & 0b00000000000000001111111100000000) >>  8;
		Preamble[14] = (mem1.Time.SubSeconds & 0b00000000000000000000000011111111) >>  0;

		TransmitData(&com1, Preamble, 15, &mem1.Data[0], mem1.DataLength+10);
		// free memory
		free(mem1.Data);
		mem1.Data = NULL;
		mem1.DataLength = 0;
	}

	// return data from memory card 2
	for (uint32_t index = loc2; index < mem2.FileSystemCounter; index++)
	{
		if ((RecalImage(&mem2, index)) != MEM_OK)
		{
			Error_Handler();
		}

		// preamble
		Preamble[0] = 0xE0|((index & 0b111111110000000000000000) >> 16);
		Preamble[1] = 0x00|((index & 0b000000001111111111111111)      );
		Preamble[2] = 0b00000000;
		Preamble[3] = 0x00;
		Preamble[4] = 0x00;

		Preamble[5 ] = mem2.Date.Year 	;
		Preamble[6 ] = mem2.Date.Month	;
		Preamble[7 ] = mem2.Date.Date 	;

		Preamble[8 ] = mem2.Time.Hours	;
		Preamble[9 ] = mem2.Time.Minutes;
		Preamble[10] = mem2.Time.Seconds;

		Preamble[11] = (mem2.Time.SubSeconds & 0b11111111000000000000000000000000) >> 24;
		Preamble[12] = (mem2.Time.SubSeconds & 0b00000000111111110000000000000000) >> 16;
		Preamble[13] = (mem2.Time.SubSeconds & 0b00000000000000001111111100000000) >>  8;
		Preamble[14] = (mem2.Time.SubSeconds & 0b00000000000000000000000011111111) >>  0;

		TransmitData(&com1, Preamble, 15, &mem2.Data[0], mem2.DataLength+10);
		// free memory
		free(mem2.Data);
		mem2.Data = NULL;
		mem2.DataLength = 0;
	}

	// preamble
	Preamble[0] = 0x5A;
	Preamble[1] = 0x44;
	Preamble[2] = 0b00000000;
	Preamble[3] = 0x00;
	Preamble[4] = 0x00;

	// send back the frame rate
	Data[0] = ((cam1.ImageCounter+cam2.ImageCounter)/TIME_005S);
	Data[1] = loc1;
	Data[2] = loc2;
	Data[3] = cam1.ImageCounter;
	Data[4] = cam2.ImageCounter;

	// answer with an acknowledge
	TransmitData(&com1, Preamble, 5, Data, 5);
}

/**
  * @brief Calibrate function
  * @retval None
  */
void state_05(uint8_t *a)
{
	uint8_t Preamble[15], Data[8];

	// set mode
	mode = MODE_CALI;

	// start cameras
	StartCamera(&cam1);
	StartCamera(&cam2);
	// wait for data read to complete
	wait_for_interrupt();

	SetImageCompressionRatio(&cam1, quality);
	SetImageCompressionRatio(&cam2, quality);

	// clear the image counters
	cam1.ImageCounter = 0;
	cam2.ImageCounter = 0;

	// store image locations
	int loc1 = mem1.FileSystemCounter;
	int loc2 = mem2.FileSystemCounter;

	cam_1_cal = loc1;
	cam_2_cal = loc2;

	//----------------------------------------------------------------------------
	// save start time
	st = HAL_GetTick();
	cp = TIME_005S;

	// start the timer
	HAL_TIM_Base_Start_IT(&htim1);

	// blocking mode, wait for sequence to finish
	while (timeout_flag == 0) {}
	timeout_flag = 0;
	//----------------------------------------------------------------------------

	// return data from memory card 1
	for (uint32_t index = loc1; index < mem1.FileSystemCounter; index++)
	{
		if ((RecalImage(&mem1, index)) != MEM_OK)
		{
			Error_Handler();
		}

		// preamble
		Preamble[0] = 0xF0|((index & 0b111111110000000000000000) >> 16);
		Preamble[1] = 0x00|((index & 0b000000001111111111111111)      );
		Preamble[2] = 0b00000000;
		Preamble[3] = 0x00;
		Preamble[4] = 0x00;

		Preamble[5 ] = mem1.Date.Year 	;
		Preamble[6 ] = mem1.Date.Month	;
		Preamble[7 ] = mem1.Date.Date 	;

		Preamble[8 ] = mem1.Time.Hours	;
		Preamble[9 ] = mem1.Time.Minutes;
		Preamble[10] = mem1.Time.Seconds;

		Preamble[11] = (mem1.Time.SubSeconds & 0b11111111000000000000000000000000) >> 24;
		Preamble[12] = (mem1.Time.SubSeconds & 0b00000000111111110000000000000000) >> 16;
		Preamble[13] = (mem1.Time.SubSeconds & 0b00000000000000001111111100000000) >>  8;
		Preamble[14] = (mem1.Time.SubSeconds & 0b00000000000000000000000011111111) >>  0;

		TransmitData(&com1, Preamble, 15, &mem1.Data[0], mem1.DataLength+10);
		// free memory
		free(mem1.Data);
		mem1.Data = NULL;
		mem1.DataLength = 0;
	}

	// return data from memory card 2
	for (uint32_t index = loc2; index < mem2.FileSystemCounter; index++)
	{
		if ((RecalImage(&mem2, index)) != MEM_OK)
		{
			Error_Handler();
		}

		// preamble
		Preamble[0] = 0xE0|((index & 0b111111110000000000000000) >> 16);
		Preamble[1] = 0x00|((index & 0b000000001111111111111111)      );
		Preamble[2] = 0b00000000;
		Preamble[3] = 0x00;
		Preamble[4] = 0x00;

		Preamble[5 ] = mem2.Date.Year 	;
		Preamble[6 ] = mem2.Date.Month	;
		Preamble[7 ] = mem2.Date.Date 	;

		Preamble[8 ] = mem2.Time.Hours	;
		Preamble[9 ] = mem2.Time.Minutes;
		Preamble[10] = mem2.Time.Seconds;

		Preamble[11] = (mem2.Time.SubSeconds & 0b11111111000000000000000000000000) >> 24;
		Preamble[12] = (mem2.Time.SubSeconds & 0b00000000111111110000000000000000) >> 16;
		Preamble[13] = (mem2.Time.SubSeconds & 0b00000000000000001111111100000000) >>  8;
		Preamble[14] = (mem2.Time.SubSeconds & 0b00000000000000000000000011111111) >>  0;

		TransmitData(&com1, Preamble, 15, &mem2.Data[0], mem2.DataLength+10);
		// free memory
		free(mem2.Data);
		mem2.Data = NULL;
		mem2.DataLength = 0;
	}

	// preamble
	Preamble[0] = 0x5A;
	Preamble[1] = 0x45;
	Preamble[2] = 0b00000000;
	Preamble[3] = 0x00;
	Preamble[4] = 0x00;

	// response 1
	Data[0] = ima1.Temp.OutputRow[0];
	Data[1] = ima1.Temp.OutputRow[1];
	Data[2] = ima1.Temp.OutputCol[0];
	Data[3] = ima1.Temp.OutputCol[1];

	// response 2
	Data[4] = ima2.Temp.OutputRow[0];
	Data[5] = ima2.Temp.OutputRow[1];
	Data[6] = ima2.Temp.OutputCol[0];
	Data[7] = ima2.Temp.OutputCol[1];

	// answer with an acknowledge
	TransmitData(&com1, Preamble, 5, Data, 8);

}

/**
  * @brief Full memory read function. Reads all data stored on both memory cards. Transmits data back to
  * the controller over a Ethernet connection.
  * @retval None
  */
void state_06(uint8_t *a)
{
	uint8_t Preamble[15], Data[1];

	// return data from memory card 1
	for (uint32_t index = 0; index < mem1.FileSystemCounter; index++)
	{
		if ((RecalImage(&mem1, index)) != MEM_OK)
		{
			Error_Handler();
		}

		// preamble
		Preamble[0] = 0xF0|((index & 0b111111110000000000000000) >> 16);
		Preamble[1] = 0x00|((index & 0b000000001111111111111111)      );
		Preamble[2] = 0b00000000;
		Preamble[3] = 0x00;
		Preamble[4] = 0x00;

		Preamble[5 ] = mem1.Date.Year 	;
		Preamble[6 ] = mem1.Date.Month	;
		Preamble[7 ] = mem1.Date.Date 	;

		Preamble[8 ] = mem1.Time.Hours	;
		Preamble[9 ] = mem1.Time.Minutes;
		Preamble[10] = mem1.Time.Seconds;

		Preamble[11] = (mem1.Time.SubSeconds & 0b11111111000000000000000000000000) >> 24;
		Preamble[12] = (mem1.Time.SubSeconds & 0b00000000111111110000000000000000) >> 16;
		Preamble[13] = (mem1.Time.SubSeconds & 0b00000000000000001111111100000000) >>  8;
		Preamble[14] = (mem1.Time.SubSeconds & 0b00000000000000000000000011111111) >>  0;

		TransmitData(&com1, Preamble, 15, &mem1.Data[0], mem1.DataLength+10);
		// free memory
		free(mem1.Data);
		mem1.Data = NULL;
		mem1.DataLength = 0;
	}

	// return data from memory card 2
	for (uint32_t index = 0; index < mem2.FileSystemCounter; index++)
	{
		if ((RecalImage(&mem2, index)) != MEM_OK)
		{
			Error_Handler();
		}

		// preamble
		Preamble[0] = 0xE0|((index & 0b111111110000000000000000) >> 16);
		Preamble[1] = 0x00|((index & 0b000000001111111111111111)      );
		Preamble[2] = 0b00000000;
		Preamble[3] = 0x00;
		Preamble[4] = 0x00;

		Preamble[5 ] = mem2.Date.Year 	;
		Preamble[6 ] = mem2.Date.Month	;
		Preamble[7 ] = mem2.Date.Date 	;

		Preamble[8 ] = mem2.Time.Hours	;
		Preamble[9 ] = mem2.Time.Minutes;
		Preamble[10] = mem2.Time.Seconds;

		Preamble[11] = (mem2.Time.SubSeconds & 0b11111111000000000000000000000000) >> 24;
		Preamble[12] = (mem2.Time.SubSeconds & 0b00000000111111110000000000000000) >> 16;
		Preamble[13] = (mem2.Time.SubSeconds & 0b00000000000000001111111100000000) >>  8;
		Preamble[14] = (mem2.Time.SubSeconds & 0b00000000000000000000000011111111) >>  0;

		TransmitData(&com1, Preamble, 15, &mem2.Data[0], mem2.DataLength+10);
		// free memory
		free(mem2.Data);
		mem2.Data = NULL;
		mem2.DataLength = 0;
	}

	// preamble
	Preamble[0] = 0x5A;
	Preamble[1] = 0x46;
	Preamble[2] = 0b00000000;
	Preamble[3] = 0x00;
	Preamble[4] = 0x00;

	// response
	Data[0] = 0x00;

	// answer with an acknowledge
	TransmitData(&com1, Preamble, 5, Data, 1);
}

/**
  * @brief The ping function.
  * @retval None
  */
void state_07(uint8_t *a)
{
	// ping

	uint8_t Preamble[5], Data[1];

	// preamble
	Preamble[0] = 0x5A;
	Preamble[1] = 0x47;
	Preamble[2] = 0b00000000;
	Preamble[3] = 0x00;
	Preamble[4] = 0x00;

	// response
	Data[0] = 0x00;

	// answer with an acknowledge
	TransmitData(&com1, Preamble, 5, Data, 1);
}

/**
  * @brief Controller MAC address set function. Assumes the first device to transmit command is the controller.
  * All subsequent commands are transmitted to that controller.
  * @retval None
  */
void state_08(uint8_t *a)
{
	uint8_t Preamble[5], Data[1];

	int ErrorCounter = 0;
	int CameraResult = 0;

	// preamble
	Preamble[0] = 0x5A;
	Preamble[1] = 0x48;
	Preamble[2] = 0b00000000;
	Preamble[3] = 0x00;
	Preamble[4] = 0x00;

	// response
	Data[0] = 0x00;

	// save the controller MAC address
	com1.Settings.DestinMAC[0] = a[6 ];
	com1.Settings.DestinMAC[1] = a[7 ];
	com1.Settings.DestinMAC[2] = a[8 ];
	com1.Settings.DestinMAC[3] = a[9 ];
	com1.Settings.DestinMAC[4] = a[10];
	com1.Settings.DestinMAC[5] = a[11];

	// update the source and destination MAC addresses
	ErrorCounter = 0;
	while (ErrorCounter < ERR_CNT_MAX)
	{
		CameraResult = SetDestinationMacAddress(&com1);
		if (CameraResult != COM_OK)
		{
			// new response
			Preamble[2] = Preamble[2] | 0b00000001;
			// increase error counter
			ErrorCounter++;
			// error handler
			Error_Handler();
		}
		else
		{
			Preamble[2] = Preamble[2] & 0b11111110;
			break;
		}
	}

	// answer with an acknowledge
	TransmitData(&com1, Preamble, 5, Data, 1);
}

/**
  * @brief Starts the race.
  * @retval None
  */
void state_09(uint8_t *a)
{
	uint8_t Preamble[5], Data[1];

	// preamble
	Preamble[0] = 0x5A;
	Preamble[1] = 0x49;
	Preamble[2] = 0b00000000;
	Preamble[3] = 0x00;
	Preamble[4] = 0x00;

	// response
	Data[0] = 0x00;

	// stop cameras
	StopCamera(&cam1);
	StopCamera(&cam2);

	// start cameras
	StartCamera(&cam1);
	StartCamera(&cam2);
	// wait for data read to complete
	wait_for_interrupt();

	// reset motion detection
	InitIma(&ima1);
	InitIma(&ima2);

	// reset motion location detection array
	det_index = 0;

	SetImageCompressionRatio(&cam1, quality);
	SetImageCompressionRatio(&cam2, quality);

	// answer with an acknowledge
	TransmitData(&com1, Preamble, 5, Data, 1);

	//----------------------------------------------------------------------------
	// place into race mode
	mode = MODE_RACE;

	// save start time
	st = HAL_GetTick();
	cp = TIME_060S;

	// start the timer
	HAL_TIM_Base_Start_IT(&htim1);

	// blocking mode, wait for sequence to finish
	while (timeout_flag == 0) {}
	timeout_flag = 0;
	//----------------------------------------------------------------------------
}

/**
  * @brief Get the detection area
  * @retval None
  */
void state_10(uint8_t *a)
{
	// getd

	uint8_t Preamble[5], Data[8];

	// preamble
	Preamble[0] = 0x5A;
	Preamble[1] = 0x50;
	Preamble[2] = 0b00000000;
	Preamble[3] = 0x00;
	Preamble[4] = 0x00;

	// response 1
	Data[0] = ima1.Temp.OutputRow[0];
	Data[1] = ima1.Temp.OutputRow[1];
	Data[2] = ima1.Temp.OutputCol[0];
	Data[3] = ima1.Temp.OutputCol[1];

	// response 2
	Data[4] = ima2.Temp.OutputRow[0];
	Data[5] = ima2.Temp.OutputRow[1];
	Data[6] = ima2.Temp.OutputCol[0];
	Data[7] = ima2.Temp.OutputCol[1];

	// answer with an acknowledgeb
	TransmitData(&com1, Preamble, 5, Data, 8);
}

/**
  * @brief Temp
  * @retval None
  */
void wait_for_interrupt(void)
{
	int timeout_st = 0, timeout_sp = 0;

	// wait for data read to complete
	timeout_st = HAL_GetTick();
	timeout_sp = timeout_st;
	while ((cam1.Done == 0) || (cam2.Done == 0))
	{
		// if timeout
		if (((timeout_sp-timeout_st)/1000) > ERR_TIMEOUT)
		{
			Error_Handler();
			if (cam1.Done == 0) RestartCamera(&cam1);
			if (cam2.Done == 0) RestartCamera(&cam2);
			break;
		}
		timeout_sp = HAL_GetTick();
	}
}

/**
  * @brief Decode the images one by one and analyse using the algorithm developed in Python
  * @retval None
  */
void mode_race(void)
{
	int ini = 0, fin = 0, res = 0;
	for (int det = 0; det < det_index; det++)
	{
		int counter;
		counter = 0;
		// return data from memory card 1
		for (uint32_t index = cam_1_det[det]; index < (cam_1_det[det]+6); index++)
		{
			if ((RecalImage(&mem1, index)) != MEM_OK)
			{
				Error_Handler();
			}
			else
			{
				// decode pixels: camera 1
				dec1.ImageData = (&mem1.Data[683]);
				dec1.ImageLength = mem1.DataLength-5-683;
				if (Decode(&dec1) == DEC_OK)
				{
					ini = HAL_GetTick();

					CreateDetectionMask(&ima1, dec1.PixelsCr, 80);
					GetShouldersCoordinates(&ima1, 0);

					fin = HAL_GetTick();
					res = fin - ini;
				}
				++counter;
			}
			// free memory
			free(mem1.Data);
			mem1.Data = NULL;
			mem1.DataLength = 0;
		}

		counter = 0;
		// return data from memory card 2
		for (uint32_t index = cam_2_det[det]; index < (cam_2_det[det]+6); index++)
		{
			if ((RecalImage(&mem2, index)) != MEM_OK)
			{
				Error_Handler();
			}
			else
			{
				// decode pixels: camera 2
				dec2.ImageData = &mem2.Data[683];
				dec2.ImageLength = mem2.DataLength-5-683;
				if (Decode(&dec2) == DEC_OK)
				{
					CreateDetectionMask(&ima2, dec2.PixelsCr, 80);
					GetShouldersCoordinates(&ima2, 1);
				}
				++counter;
			}
			// free memory
			free(mem2.Data);
			mem2.Data = NULL;
			mem2.DataLength = 0;
		}


		uint8_t final1 = 0, final2 = 0, minim1, minim2;
		uint8_t min1 = 0, min2 = 0, ind1 = 0, ind2 = 0;

		//////////////////////////////////////////////////////////
		min1 = 0xFF;
		min2 = 0xFF;
		ind1 = 0   ;
		ind2 = 0   ;

		for (int index = 0; index < 6; index++)
		{
			if (ima1.Mask.Diff1[index] < min1)
			{
				min1 = ima1.Mask.Diff1[index];
				ind1 = index;
			}
		}
		for (int index = 0; index < 6; index++)
		{
			if (ima1.Mask.Diff2[index] < min2)
			{
				min2 = ima1.Mask.Diff2[index];
				ind2 = index;
			}
		}
		if (min1 < min2)
		{
			minim1 = min1;
			final1 = ind1;
		}
		else
		{
			minim1 = min2;
			final1 = ind2;
		}
		//////////////////////////////////////////////////////////

		//////////////////////////////////////////////////////////
		min1 = 0xFF;
		min2 = 0xFF;
		ind1 = 0   ;
		ind2 = 0   ;
		for (int index = 0; index < 6; index++)
		{
			if (ima2.Mask.Diff1[index] < min1)
			{
				min1 = ima2.Mask.Diff1[index];
				ind1 = index;
			}
		}
		for (int index = 0; index < 6; index++)
		{
			if (ima2.Mask.Diff2[index] < min2)
			{
				min2 = ima2.Mask.Diff2[index];
				ind2 = index;
			}
		}
		if (min1 < min2)
		{
			minim2 = min1;
			final2 = ind1;
		}
		else
		{
			minim2 = min2;
			final2 = ind2;
		}
		//////////////////////////////////////////////////////////

		if ((minim1 < minim2) && (minim1 < 32)) // camera 1
		{
			if ((RecalImage(&mem1, cam_1_det[det]+(5-final1))) != MEM_OK)
			{
				Error_Handler();
			}
			else
			{
				uint8_t Preamble[15];
				//----------------------------------------------------------------------------
				Preamble[0] = 0xA0|((det & 0b111111110000000000000000) >> 16);
				Preamble[1] = 0x00|((det & 0b000000001111111111111111)      );
				Preamble[2] = 0b00000000;
				Preamble[3] = 0x00;
				Preamble[4] = 0x00;

				Preamble[5 ] = mem1.Date.Year 	;
				Preamble[6 ] = mem1.Date.Month	;
				Preamble[7 ] = mem1.Date.Date 	;

				Preamble[8 ] = mem1.Time.Hours	;
				Preamble[9 ] = mem1.Time.Minutes;
				Preamble[10] = mem1.Time.Seconds;

				Preamble[11] = (mem1.Time.SubSeconds & 0b11111111000000000000000000000000) >> 24;
				Preamble[12] = (mem1.Time.SubSeconds & 0b00000000111111110000000000000000) >> 16;
				Preamble[13] = (mem1.Time.SubSeconds & 0b00000000000000001111111100000000) >>  8;
				Preamble[14] = (mem1.Time.SubSeconds & 0b00000000000000000000000011111111) >>  0;

				TransmitData(&com1, Preamble, 15, &mem1.Data[0], mem1.DataLength+10);
				//----------------------------------------------------------------------------
				free(mem1.Data);
				mem1.Data = NULL;
				mem1.DataLength = 0;
			}
		}
		else
		if ((minim2 < minim1) && (minim2 < 32)) // camera 2
		{
			if ((RecalImage(&mem2, cam_2_det[det]+(5-final2))) != MEM_OK)
			{
				Error_Handler();
			}
			else
			{
				uint8_t Preamble[15];
				//----------------------------------------------------------------------------
				Preamble[0] = 0xB0|((det & 0b111111110000000000000000) >> 16);
				Preamble[1] = 0x00|((det & 0b000000001111111111111111)      );
				Preamble[2] = 0b00000000;
				Preamble[3] = 0x00;
				Preamble[4] = 0x00;

				Preamble[5 ] = mem2.Date.Year 	;
				Preamble[6 ] = mem2.Date.Month	;
				Preamble[7 ] = mem2.Date.Date 	;

				Preamble[8 ] = mem2.Time.Hours	;
				Preamble[9 ] = mem2.Time.Minutes;
				Preamble[10] = mem2.Time.Seconds;

				Preamble[11] = (mem2.Time.SubSeconds & 0b11111111000000000000000000000000) >> 24;
				Preamble[12] = (mem2.Time.SubSeconds & 0b00000000111111110000000000000000) >> 16;
				Preamble[13] = (mem2.Time.SubSeconds & 0b00000000000000001111111100000000) >>  8;
				Preamble[14] = (mem2.Time.SubSeconds & 0b00000000000000000000000011111111) >>  0;

				TransmitData(&com1, Preamble, 15, &mem2.Data[0], mem2.DataLength+10);
				//----------------------------------------------------------------------------
				free(mem2.Data);
				mem2.Data = NULL;
				mem2.DataLength = 0;
			}
		}
	}
}

/**
  * @brief Decode the images one by one and analyse using the algorithm developed in Python
  * @retval None
  */
void mode_cali(void)
{
	int counter;
	counter = 0;
	int ini = 0, fin = 0, res = 0;
	// read the first two images from the card
	for (uint32_t index = cam_1_cal; index < mem1.FileSystemCounter; index++)
	{
		if ((RecalImage(&mem1, index)) != MEM_OK)
		{
			Error_Handler();
		}
		else
		{
			ini = HAL_GetTick();

			if (counter == 0)
			{
				// decode start: camera 1
				dec1.ImageData = (&mem1.Data[005]);
				dec1.ImageLength = 664;
				SplitEncodedData(&dec1);
				DecodeHuffman(&dec1);
				DecodeQuantis(&dec1);

				fin = HAL_GetTick();
				res = fin - ini;
			}
			else
			if (counter == 1)
			{
				// decode pixels: camera 1
				dec1.ImageData = (&mem1.Data[683]);
				dec1.ImageLength = mem1.DataLength-5-683;
				if (Decode(&dec1) == DEC_OK)
				{
					fin = HAL_GetTick();
					res = fin - ini;

					ini = HAL_GetTick();

					// template match
					MatchTemplate(&ima1, dec1.PixelsLu, 320, IMA_SQDIFF);
					GetDetectionCoordinates(&ima1);

					fin = HAL_GetTick();
					res = fin - ini;
				}
			}
			++counter;
		}
		// free memory
		free(mem1.Data);
		mem1.Data = NULL;
		mem1.DataLength = 0;
	}

	counter = 0;
	// read the first two images from the card
	for (uint32_t index = cam_2_cal; index < mem2.FileSystemCounter; index++)
	{
		if ((RecalImage(&mem2, index)) != MEM_OK)
		{
			Error_Handler();
		}
		else
		{
			if (counter == 0)
			{
				// decode start: camera 2
				dec2.ImageData = (&mem2.Data[005]);
				dec2.ImageLength = 664;
				InitDec(&dec2);
				SplitEncodedData(&dec2);
				DecodeHuffman(&dec2);
				DecodeQuantis(&dec2);
			}
			else
			if (counter == 1)
			{
				// decode pixels: camera 2
				dec2.ImageData = (&mem2.Data[683]);
				dec2.ImageLength = mem2.DataLength-5-683;
				if (Decode(&dec2) == DEC_OK)
				{
					// template match
					MatchTemplate(&ima2, dec2.PixelsLu, 320, IMA_SQDIFF);
					GetDetectionCoordinates(&ima2);
				}
			}
			++counter;
		}
		// free memory
		free(mem2.Data);
		mem2.Data = NULL;
		mem2.DataLength = 0;
	}
}

//----------------------------------------------------------------------------

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

	while (1)
	{
		break;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
