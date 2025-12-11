/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "UARTdriver.h"//setting on uart.c is added to hal_msp.c file
#include <math.h>
#include <stdbool.h>
#include "MTi1.h"
#include "messages.h"
#include "queue_structs.h"
#include "constants.h"
#include "simpleDataLink.h"
#include "pid_conversions.h"
#include "actuator_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
PID_Inputs_struct PID_Inputs;
Actuator_struct Reaction1;
Actuator_struct Reaction2;
Actuator_struct MagneTorquer1;
Actuator_struct MagneTorquer2;
Actuator_struct MagneTorquer3;
uint8_t error_status = 0;
uint8_t Channels_mask[NUM_DRIVERS] = {1,1,1,1,1};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

// osThreadId IMUTaskHandle;
// uint32_t IMUTaskBuffer[ 4096 ];
// osStaticThreadDef_t IMUTaskControlBlock;
// osThreadId OBC_CommTaskHandle;
// uint32_t OBC_CommTaskBuffer[ 16384 ];
// osStaticThreadDef_t OBC_CommTaskControlBlock;
// osThreadId ControlAlgorithmTaskHandle;
// uint32_t ControlAlgorithmTaskBuffer[ 4096 ];
// osStaticThreadDef_t ControlAlgorithmTaskControlBlock;
// osThreadId FirstCheckTaskHandle;
// uint32_t FirstCheckTaskBuffer[ 4096 ];
// osStaticThreadDef_t FirstCheckTaskControlBlock;
// osMessageQId IMUQueue1Handle;
// uint8_t IMUQueue1Buffer[ 512 * sizeof( uint32_t ) ];
// osStaticMessageQDef_t IMUQueue1ControlBlock;
// osMessageQId IMUQueue2Handle;
// uint8_t IMUQueue2Buffer[ 512 * sizeof( uint32_t ) ];
// osStaticMessageQDef_t IMUQueue2ControlBlock;
// osMessageQId ADCSHouseKeepingQueueHandle;
// uint8_t ADCSHouseKeepingQueueBuffer[ 512 * sizeof( uint32_t ) ];
// osStaticMessageQDef_t ADCSHouseKeepingQueueControlBlock;
/* USER CODE BEGIN PV */
osThreadId IMUTaskHandle;
uint32_t IMUTaskBuffer[ stack_size]; //4096
osStaticThreadDef_t IMUTaskControlBlock;

osThreadId OBC_CommTaskHandle;
uint32_t OBC_CommTaskBuffer[ stack_size1 ]; //16384
osStaticThreadDef_t OBC_CommTaskControlBlock;

osThreadId ControlAlgorithmTaskHandle;
uint32_t ControlAlgorithmTaskBuffer[ stack_size ]; //4096
osStaticThreadDef_t ControlAlgorithmTaskControlBlock;

osThreadId FirstCheckTaskHandle;
uint32_t FirstCheckTaskBuffer[ stack_size ];//4096
osStaticThreadDef_t FirstCheckTaskControlBlock;

osMessageQId ADCSHouseKeepingQueueHandle;
uint8_t ADCSHouseKeepingQueueBuffer[ 256 * sizeof( float ) ];
osStaticMessageQDef_t ADCSHouseKeepingQueueControlBlock;

osMessageQId IMUQueue2Handle;
uint8_t IMUQueue2Buffer[ 256 * sizeof( imu_queue_struct ) ];
osStaticMessageQDef_t IMUQueue2ControlBlock;

osMessageQId IMUQueue1Handle;
uint8_t IMUQueue1Buffer[ 256 * sizeof( imu_queue_struct ) ];
osStaticMessageQDef_t IMUQueue1ControlBlock;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
void IMU_Task(void const * argument);
void OBC_Comm_Task(void const * argument);
void Control_Algorithm_Task(void const * argument);
void Check_current_temp(void const * argument);

/* USER CODE BEGIN PFP */

//defining putch to enable printf
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

// PUTCHAR_PROTOTYPE{
//   HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
//   return ch;
// }
PUTCHAR_PROTOTYPE{
	uint8_t c=(uint8_t)ch;
	sendDriver_UART(&huart2,&c,1);
	return c;
}
void IMU_Task(void const * argument);
void OBC_Comm_Task(void const * argument);

//defining serial line I/O functions
//using UART driver
uint8_t txFunc1(uint8_t byte){
	return (sendDriver_UART(&huart1, &byte, 1)!=0);
}
uint8_t rxFunc1(uint8_t* byte){
	return (receiveDriver_UART(&huart1, byte, 1)!=0);
}

//defining tick function for timeouts
uint32_t sdlTimeTick(){
	return HAL_GetTick();
}


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
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  
  initDriver_UART();
  // uint8_t status = addDriver_UART(&huart2, UART4_IRQn, keep_new);
  // if (status != 0) {
  //   char err[64];
  //   int len = snprintf(err, sizeof(err), "addDriver_UART failed: %d\r\n", status);
  //   HAL_UART_Transmit(&huart2, (uint8_t*)err, len, 100);
  // }

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  // IMURead_ControlMutex = xSemaphoreCreateMutexStatic(&xIMURead_ControlMutexBuffer);
	// configASSERT(IMURead_ControlMutex);
	// xSemaphoreGive(IMURead_ControlMutex);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  // /* definition and creation of IMUQueue1 */
  // osMessageQStaticDef(IMUQueue1, 512, uint32_t, IMUQueue1Buffer, &IMUQueue1ControlBlock);
  // IMUQueue1Handle = osMessageCreate(osMessageQ(IMUQueue1), NULL);

  // /* definition and creation of IMUQueue2 */
  // osMessageQStaticDef(IMUQueue2, 512, uint32_t, IMUQueue2Buffer, &IMUQueue2ControlBlock);
  // IMUQueue2Handle = osMessageCreate(osMessageQ(IMUQueue2), NULL);

  // /* definition and creation of ADCSHouseKeepingQueue */
  // osMessageQStaticDef(ADCSHouseKeepingQueue, 512, uint32_t, ADCSHouseKeepingQueueBuffer, &ADCSHouseKeepingQueueControlBlock);
  // ADCSHouseKeepingQueueHandle = osMessageCreate(osMessageQ(ADCSHouseKeepingQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* definition and creation of IMUQueue1 */
	osMessageQStaticDef(IMUQueue1, 512, uint32_t,IMUQueue1Buffer, &IMUQueue1ControlBlock);
	IMUQueue1Handle = osMessageCreate(osMessageQ(IMUQueue1), NULL);
  /* definition and creation of IMUQueue2 */
	osMessageQStaticDef(IMUQueue2, 512, uint32_t, IMUQueue2Buffer, &IMUQueue2ControlBlock);
	IMUQueue2Handle = osMessageCreate(osMessageQ(IMUQueue2), NULL);
  /* definition and creation of ADCSHouseKeepingQueue */
	osMessageQStaticDef(ADCSHouseKeepingQueue, 512, uint32_t, ADCSHouseKeepingQueueBuffer, &ADCSHouseKeepingQueueControlBlock);
	ADCSHouseKeepingQueueHandle = osMessageCreate(osMessageQ(ADCSHouseKeepingQueue), NULL);
  /* definition and creation of FirstCheckTask */
  osThreadStaticDef(FirstCheckTask, Check_current_temp, osPriorityAboveNormal, 0, stack_size, FirstCheckTaskBuffer, &FirstCheckTaskControlBlock);
  FirstCheckTaskHandle = osThreadCreate(osThread(FirstCheckTask), NULL);
  /* definition and creation of IMUTask */
  osThreadStaticDef(IMUTask, IMU_Task, osPriorityNormal, 0,stack_size, IMUTaskBuffer, &IMUTaskControlBlock);
  IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);
  /* definition and creation of OBC_CommTask */
  osThreadStaticDef(OBC_CommTask, OBC_Comm_Task, osPriorityAboveNormal, 0,stack_size1, OBC_CommTaskBuffer, &OBC_CommTaskControlBlock);
  OBC_CommTaskHandle = osThreadCreate(osThread(OBC_CommTask), NULL);
  /* definition and creation of ControlAlgorithmTask */
  osThreadStaticDef(ControlAlgorithmTask, Control_Algorithm_Task, osPriorityNormal, 0,stack_size, ControlAlgorithmTaskBuffer, &ControlAlgorithmTaskControlBlock);
	ControlAlgorithmTaskHandle = osThreadCreate(osThread(ControlAlgorithmTask), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of IMUTask */
  // osThreadStaticDef(IMUTask, IMU_Task, osPriorityNormal, 0, 4096, IMUTaskBuffer, &IMUTaskControlBlock);
  // IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

  /* definition and creation of OBC_CommTask */
  // osThreadStaticDef(OBC_CommTask, OBC_Comm_Task, osPriorityAboveNormal, 0, 16384, OBC_CommTaskBuffer, &OBC_CommTaskControlBlock);
  // OBC_CommTaskHandle = osThreadCreate(osThread(OBC_CommTask), NULL);

  /* definition and creation of ControlAlgorithmTask */
  // osThreadStaticDef(ControlAlgorithmTask, Control_Algorithm_Task, osPriorityNormal, 0, 4096, ControlAlgorithmTaskBuffer, &ControlAlgorithmTaskControlBlock);
  // ControlAlgorithmTaskHandle = osThreadCreate(osThread(ControlAlgorithmTask), NULL);

  /* definition and creation of FirstCheckTask */
  // osThreadStaticDef(FirstCheckTask, Check_current_temp, osPriorityAboveNormal, 0, 4096, FirstCheckTaskBuffer, &FirstCheckTaskControlBlock);
  // FirstCheckTaskHandle = osThreadCreate(osThread(FirstCheckTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Period = 199;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 199;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Period = 199;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD4_Pin */
  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_IMU_Task */
/**
  * @brief  Function implementing the IMUTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_IMU_Task */
void IMU_Task(void const * argument)
{
  /* USER CODE BEGIN 5 */
  // huart4.gState = HAL_UART_STATE_READY;
  // huart4.RxState = HAL_UART_STATE_READY;
  // making sure that UART driver is initialized and UARTs are added after freertos started
  //initDriver_UART();
	//UART2 = for printf
  uint8_t status = addDriver_UART(&huart2, USART2_IRQn, keep_new);
  // if (status == 0) {
  //   char msg[] = "USART2 Driver initialized OK\r\n";
  //   HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
  // } else {
  //   char msg[] = "USART2 Driver FAILED\r\n";
  //   HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
  // }

	// // UART4 = for IMU
  uint8_t status2 = addDriver_UART(&huart4, UART4_IRQn, keep_new);
  // if (status2 == 0) {
  //   char msg[] = "UART4 Driver initialized OK\r\n";
  //   HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
  // } else {
  //   char msg[] = "UART4 Driver FAILED\r\n";
  //   HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
  // }

  // osDelay(1000); //when in doubt add a delay

  #if enable_printf
	printf("Initializing IMU \n");
  #endif
    //uint8_t ret = 1;
    uint8_t ret = initIMUConfig(&huart4);
  #if enable_printf
    if(ret) printf("IMU correctly configured \n");
    else printf("Error configuring IMU \n");
  #endif

	float gyro[3]={1,2,3};
	float mag[3]={4,5,6};
	float acc[3] = {7,8,9};

	imu_queue_struct *local_imu_struct =(imu_queue_struct*) malloc(sizeof(imu_queue_struct));

	/* Infinite loop */
	for(;;)
	{

		//Non c'è bisogno di settare o resettare il CTS e l'RTS della UART4 per IMU perchè le funzioni
		//UART_Transmit e UART_Receive gestiscono la cosa automaticamente se dall'altro lato il dispositivo ha abilitato pure
		//queste due linee per l'UART
		//Se voglio far comunicare IMU e Nucleo con solo le 2 linee UART tx ed Rx basta che disabilito l'hardware flow control
		//da CubeMx.


		ret=readIMUPacket(&huart4, gyro, mag, acc, 500); //mag measured in Gauss(G) unit -> 1G = 10^-4 Tesla
		mag[0]/=10000; //1G = 10^-4 Tesla
		mag[1]/=10000; //1G = 10^-4 Tesla
		mag[2]/=10000; //1G = 10^-4 Tesla
		/*if (xSemaphoreTake(IMURead_ControlMutex, (TickType_t)10) == pdTRUE)//If reading IMU DO NOT CONTROL
		{
			printf("IMU Task : Taken IMURead_Control control");
			ret=readIMUPacket(&huart4, gyro, mag, 50);
			xSemaphoreGive(IMURead_ControlMutex);
			printf("IMU Task : Released IMURead_Control control");
		}*/
    // printf("IMU status %d \r\n",ret);
		if(ret)
		{
			/*for(uint32_t field=0; field<3;field++){
					printf("%f \t",gyro[field]);
			}
			printf("\nMagnetometer: ");
			for(uint32_t field=0; field<3;field++){
				printf("%f \t",mag[field]);
			}
			printf("\n");*/
			if (local_imu_struct == NULL) {
				printf("IMU TASK: allocazione struttura fallita !\n");
			}
			else
			{
				//Riempio struct con valori letti da IMU,per poi inviareli a Task Controllo
				for (int i = 0; i < 3; i++)
				{
					local_imu_struct->gyro_msr[i] = gyro[i];
					local_imu_struct->mag_msr[i] = mag[i];
					local_imu_struct->acc_msr[i] = acc[i];
					printf("Accelerometer axis %d, value %f \r\n", i, acc[i]);
					printf("Gyroscope axis %d, value %f \r\n", i, gyro[i]);
					printf("Magnetometer axis %d, value %f \r\n", i, mag[i]);
				}
				//Invio queue a Control Task
			 	if (osMessagePut(IMUQueue1Handle,(uint32_t)local_imu_struct,300) != osOK) {
			    	//printf("Invio a Control Task fallito \n");
			       	free(local_imu_struct); // Ensure the receiving task has time to process
				} else {
			        //printf("Dati Inviati a Control Task \n");

			 	}
			 	//Invio queue a OBC Task
			 	if (osMessagePut(IMUQueue2Handle,(uint32_t)local_imu_struct,300) != osOK) {
			    	//printf("Invio a OBC Task fallito \n");
			       	free(local_imu_struct); // Ensure the receiving task has time to process
			 	} else {
			    	//printf("Dati a Control Inviati \n");
				}
			}
		}
		else{
			//printf("IMU: Error configuring IMU \n");
			osDelay(2000);
		}
    // printf("Hello from STM32L4\r\n");
    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_OBC_Comm_Task */
/**
* @brief Function implementing the OBC_CommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OBC_Comm_Task */
void OBC_Comm_Task(void const * argument)
{
  /* USER CODE BEGIN OBC_Comm_Task */
    //initDriver_UART();
  //UART1 = for OBC communication
  // /*uint8_t status = */addDriver_UART(&huart4, UART4_IRQn, keep_new);
  //addDriver_UART(&huart1,USART1_IRQn,keep_old);
  /*uint8_t status2 = */addDriver_UART(&huart1, USART1_IRQn, keep_old);
  // if (status2 == 0) {
  //   char msg[] = "UART1 Driver initialized OK\r\n";
  //   HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
  // } else {
  //   char msg[] = "UART1 Driver FAILED\r\n";
  //   HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
  // }
  static serial_line_handle line1;
	

  // uint8_t fuck=0x67;
  // uint8_t status4 = txFunc1(fuck);
  
  // char msg[32];
  // int len = snprintf(msg, sizeof(msg), "txFunc1 returned: %u\r\n", status4);
  // HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
  
  // if (status4 == 1)  {  // expect 1 byte sent
  //   char ok[] = "tx OK (1 byte)\r\n";
  //   HAL_UART_Transmit(&huart2, (uint8_t*)ok, strlen(ok), 100);
  // } else {
  //   char fail[] = "tx FAILED (0 bytes)\r\n";
  //   HAL_UART_Transmit(&huart2, (uint8_t*)fail, strlen(fail), 100);
  // }
  // vTaskDelay(pdMS_TO_TICKS(1000));

  //Inizialize Serial Line for UART1
  sdlInitLine(&line1,&txFunc1,&rxFunc1,50,2);
	uint8_t opmode=0;
	uint32_t rxLen;

	setAttitudeADCS *RxAttitude = (setAttitudeADCS*) malloc(sizeof(setAttitudeADCS));
	housekeepingADCS TxHousekeeping;
	attitudeADCS TxAttitude;
	setOpmodeADCS RxOpMode;
	opmodeADCS TxOpMode;
	osEvent retvalue1,retvalue;
	uint8_t cnt1 = 0,cnt2 = 0;
	char rxBuff[SDL_MAX_PAY_LEN];
    /* Infinite loop */
  for(;;)
  {
    
	 /*-------------------SEND TO OBC-------------------------*/
	//sampling
	  /* in theory here we should sample values and fill telemetry structures
	  telemetryStruct.temp1=...;
	  telemetryStruct.speed=...;
	  .....*/
	
	 //Receive HouseKeeping sensor values via Queue
	retvalue = osMessageGet(ADCSHouseKeepingQueueHandle,300);

	// //printf("OBC Task: Tick_Time: %lu \n",HAL_GetTick());

	if (retvalue.status == osEventMessage)
	// {
		processCombinedData((void*)&retvalue,(void *)&TxHousekeeping,receive_Current_Tempqueue_OBC);
		//attitude sampling
	// 	//in this case we just send the local copy of the structure
	// 	//ALWAYS remember to set message code (use the generated defines

	// 	//printf("OBC: Trying to send attitude \n");
	// 	//finally we send the message

			printf("OBC TASK: after 7 counts: %lu \n",HAL_GetTick());
			TxHousekeeping.code=HOUSEKEEPINGADCS_CODE;
			TxHousekeeping.ticktime=HAL_GetTick();
			//printf("OBC: Trying to send housekeeping \n");
			//finally we send the message

			if(sdlSend(&line1,(uint8_t *)&TxHousekeeping,sizeof(housekeepingADCS),0)){}

	// }

	//Receive Telemetry IMU via Queue
	retvalue1 = osMessageGet(IMUQueue2Handle, 300);

	if (retvalue1.status == osEventMessage)
	{
		processCombinedData((void*)&retvalue1,(void *)&TxAttitude,receive_IMUqueue_OBC);
		//in this case we just fill the structure with random values
		//ALWAYS remember to set message code (use the generated defines
			TxAttitude.code=ATTITUDEADCS_CODE;
			TxAttitude.ticktime=HAL_GetTick();
    // printf("OBC TASK:i am alive %lu \r\n",HAL_GetTick());
    // uint8_t sendStatus = sdlSend(&line1,(uint8_t *)&TxAttitude,sizeof(attitudeADCS),0);
    // printf("OBC TASK: sdlSend status: 0x%02X at %lu \r\n", sendStatus, HAL_GetTick());
		if(sdlSend(&line1,(uint8_t *)&TxAttitude,sizeof(attitudeADCS),0)){
      printf("OBC TASK:i am connected %lu \r\n",HAL_GetTick());
    }


	}

	opmodeADCS opmodeMsg;
	opmodeMsg.opmode=opmode;
	//ALWAYS remember to set message code (use the generated defines
	opmodeMsg.code=OPMODEADCS_CODE;
	//finally we send the message (WITH ACK REQUESTED)
	// printf("OBC: Trying to send opmodeADCS \r\n");
	if(sdlSend(&line1,(uint8_t *)&opmodeMsg,sizeof(opmodeADCS),1)){
    printf("OBC: success to send opmodeADCS \r\n");
  }


  	osDelay(2000);
  }
  /* USER CODE END OBC_Comm_Task */
}

/* USER CODE BEGIN Header_Control_Algorithm_Task */
/**
* @brief Function implementing the ControlAlgorith thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Control_Algorithm_Task */
void Control_Algorithm_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Algorithm_Task */
  uint8_t flag = 0;
	osEvent retvalue,retvalue1;
	//Inizialize actuators struct
	init_actuator_handler(&Reaction1,&htim1,TIM_CHANNEL_1,TIM_CHANNEL_2,100000,50); //100 khz
	init_actuator_handler(&Reaction2,&htim2,TIM_CHANNEL_3,TIM_CHANNEL_4,20000,50);
	init_actuator_handler(&MagneTorquer1,&htim3,TIM_CHANNEL_1,TIM_CHANNEL_2,20000,20); //89 khz //this measured 10khz, idkwhy
	init_actuator_handler(&MagneTorquer2,&htim3,TIM_CHANNEL_3,TIM_CHANNEL_4,10000,50); //also this
	init_actuator_handler(&MagneTorquer3,&htim2,TIM_CHANNEL_1,TIM_CHANNEL_2,94000,50); //94 khz // this measured 100khz, idkwhy
//12332
	//Inizialize PID struct
	PID_INIT(&PID_Inputs);

  /* Infinite loop */
  for(;;)
  {
  //printf("We are in Control Algorithm TASK \n");
#if enable_printf
		//printf("We are in Control Algorithm TASK \n");
#endif
    printf("I am alive from Control_Algorithm_Task at %lu ms\r\n", HAL_GetTick());
		//Receive Telemetry IMU via Queue

		// retvalue1 = osMessageGet(setAttitudeADCSQueueHandle,200);
		// processCombinedData((void*)&retvalue1,(void *)&PID_Inputs,receive_Attitudequeue_control);
    // the reason why i commented the above is that there is no task sending to that queue therefore its technically useless

		retvalue = osMessageGet(IMUQueue1Handle, 300);
		processCombinedData((void*)&retvalue,(void *)&PID_Inputs,receive_IMUqueue_control);


		//ALGORITHM
		//PID_main(&PID_Inputs);

		//Update PWM values
		//X Magnetorquer

		if(!flag)
		{
      //WARNING: WHO EVER WORK ON THIS PART REMEMBER AFTER EVERY TEST TO COMMENT THE PART BELOW 
      //BECAUSE THE DRIVER ITSELF GET HOT EXTREAMLY FAST, AND IF YOU LEAVE IT OVER NIGHT IT WILL BURN 
      //COMMENT AND PUSH AND PULL FROM THE LENOVO AND RUN AGAIN IN CUBE MX 
			// actuator_START(&Reaction1);
			// actuator_START(&Reaction2);
			// actuator_START(&MagneTorquer1);
			// actuator_START(&MagneTorquer2);
			// actuator_START(&MagneTorquer3);
      // printf("I am spinning at %lu ms\r\n", HAL_GetTick());
			flag = 1;// ALSO SET THIS FLAG BACK TO 1 AFTER TEST JUST TO MAKE SURE 
		}


		//No change dir:
		//update_duty_dir(&Reaction1,50,0);
		//Change dir :
		//update_duty_dir(&MagneTorquer1,70,1);



		//X Magnetorquer
		//Change dir :
		//update_duty_dir(&MagneTorquer1,PID_Inputs.th_Dutycycle[0],1);
		//No change dir:
		//update_duty_dir(&MagneTorquer1,PID_Inputs.th_Dutycycle[0],0);
		//Y Magnetorquer
		//Change dir :
		//update_duty_dir(&MagneTorquer1,PID_Inputs.th_Dutycycle[1],1);
		//No change dir:
		//update_duty_dir(&MagneTorquer1,PID_Inputs.th_Dutycycle[1],0);
		//Z Magnetorquer
		//Change dir :
		//update_duty_dir(&MagneTorquer1,PID_Inputs.th_Dutycycle[2],1);
		//No change dir:
		//update_duty_dir(&MagneTorquer1,PID_Inputs.th_Dutycycle[2],0);



		/*if (xSemaphoreTake(IMURead_ControlMutex, (TickType_t)10) == pdTRUE) //If control don't read IMU
		{
			printf("Control Task : Taken IMURead_ControlMutex control");
			//Spegnere i magnetorquer
			xSemaphoreGive(IMURead_ControlMutex);
			printf("Control Task : Released IMURead_ControlMutex control");
		}
		*/
		osDelay(2000);
  }
  /* USER CODE END Control_Algorithm_Task */
}

/* USER CODE BEGIN Header_Check_current_temp */
/**
* @brief Function implementing the FirstCheckTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Check_current_temp */
void Check_current_temp(void const * argument)
{
  /* USER CODE BEGIN Check_current_temp */
//declaring serial line
	//static serial_line_handle line;
	//Inizialize Serial Line for UART3
	//sdlInitLine(&line,&txFunc3,&rxFunc3,50,2);
	// init_tempsens_handler(&ntc_values);
	volatile float currentbuf[NUM_ACTUATORS],voltagebuf[NUM_ACTUATORS];
	Current_Temp_Struct *local_current_temp_struct = (Current_Temp_Struct*) malloc(sizeof(Current_Temp_Struct));
	static uint8_t count = 0;
	
	/*Start calibration */
	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) !=  HAL_OK)
	{
#if enable_printf
	  	printf("Error with ADC: not calibrated correctly \n");
#endif
	}

	/* Infinite loop */
	for(;;)
	{
		//volatile float prev = HAL_GetTick();
		//printf("We are in CHECK TASK \n");
#if enable_printf
		//printf("We are in CHECK TASK \n");
#endif
		//----------------------------------------------------------------------

		//GET TEMPERATURES------------------------------------------------------
		//float prev1 = HAL_GetTick();
		// get_temperatures(&hspi2,&ntc_values,count);
		//float next1 = HAL_GetTick();
		//printf("Execussion of get_temperatures: %.1f ms\n",next1-prev1);
		count ++;
		//----------------------------------------------------------------------

		//GET ACTUATORS CURRENT
		get_actuator_current(&hadc1,voltagebuf,currentbuf,Channels_mask);
		for(int i=0;i<NUM_DRIVERS;i++)
		{
			printf("Actuator %d current value: %f",i,currentbuf[i]);
		}
		//----------------------------------------------------------------------

		//CHECK IF THEY ARE OK
		for(int i=0;i<NUM_ACTUATORS;i++)
		{
			if(i>=0 && i<2)
			{
				//Check Reaction Wheels currents
				if(current_buf[i] > 1.0f) //> 1A
				{
					printf("MAgnetorquer %d current value: %f is above threshold!!!! ",i,currentbuf[i]);
					error_status = 5;
				}
			}
			else{
				//Check MagneTorquers currents
				if(current_buf[i] > 0.05f) // >50mA
				{
					printf("Magnetorquer %d current value: %f is above threshold!!!! ",i,currentbuf[i]);
					error_status = 4;
				}
			}
		}

		// for(int i=0;i<NUM_TEMP_SENS;i++)
		// {
		// 	if(ntc_values.temp[i]>50) //>50 gradi
		// 	{
		// 		printf("Temp %d value: %f is above threshold!!!! ",i,ntc_values.temp[i]);
		// 		error_status = 3;
		// 	}

		// }
		 
		switch(error_status)
		{
			case 0:
				//ALL IS OK
				//Send Housekeeping to OBC task
				
				if (local_current_temp_struct == NULL) {
#if enable_printf
					   printf("IMU TASK: allocazione struttura fallita !\n");
#endif
				}
				else
				{
					if(count == 8)
					{
						for(int i=0;i<NUM_ACTUATORS;i++)
						{
							local_current_temp_struct->current[i] = currentbuf[i];
#if enable_printf
							printf("Task check: Current n%d,value: %f,current vect:%f \n",i+1,local_current_temp_struct->current[i],currentbuf[i]);
#endif
			    	// 	}
						// for(int i=NUM_ACTUATORS;i<NUM_TEMP_SENS+NUM_ACTUATORS;i++)
						// {
						// 	local_current_temp_struct->temperature[i - NUM_ACTUATORS] = ntc_values.temp[i - NUM_ACTUATORS];
#if enable_printf
							// printf("Task check: Temperature n%d,ntc value: %f,value: %f \n",i-4,ntc_values.temp[i-NUM_ACTUATORS],local_current_temp_struct->temperature[i-NUM_ACTUATORS]);
#endif
						}

						//Invio queue a OBC Task
						if (osMessagePut(ADCSHouseKeepingQueueHandle,(uint32_t)local_current_temp_struct,300) != osOK) {
#if enable_printf
			    		   	printf("Invio a OBC Task fallito \n");
#endif
			       			free(local_current_temp_struct); // Ensure the receiving task has time to process
						} else {
#if enable_printf
			    		    printf("Dati Inviati a OBC Task\n");
#endif
						}
						count = 0;
					}
				}
				break;
			case 1:
				break;
			case 2:
				break;
			case 3:
				//PROBLEM WITH TEMPERATURE SENSORS
				//Fare partire un interrupt
				break;
			case 4:
				//PROBLEM WITH MAGNETORQUERS
				//Fare partire un interrupt
				break;
			case 5:
				//PROBLEM WITH REACTION WHEELS
				//Fare partire un interrupt
				break;

		}
		//volatile next = HAL_GetTick();
		//printf("Execussion of check task: %.1f ms\n",next-prev);
	    osDelay(2000);
	  }
  /* USER CODE END Check_current_temp */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM16)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
