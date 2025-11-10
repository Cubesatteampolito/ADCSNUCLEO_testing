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

/* For mutex */
#include "FreeRTOS.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---- simpleDataLink (SDL) minimal TX on USART2 ----
 * Frame: 0x7E | [code(1), ackWanted(1), hash_be(2)] | payload | crc_be(2) | 0x7E
 * Byte-stuff: 0x7E -> 0x7D 0x5E ; 0x7D -> 0x7D 0x5D
 * CRC16-CCITT, poly 0x1021, init 0xFFFF, computed on header+payload.
 * We'll send DATA (0x01) and ackWanted=0 for telemetry.
 */
#define SDL_FLAG       0x7E
#define SDL_ESC        0x7D
#define SDL_ESC_7E     0x5E
#define SDL_ESC_7D     0x5D
#define SDL_CODE_DATA  0x01u
#define SDL_MAX_PAYLOAD 240
#define UART_TX_TIMEOUT_MS 300
/* USER CODE END PD */

/* Private macro ------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BE16(x) (uint16_t)((((x)&0xFF)<<8) | (((x)>>8)&0xFF))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ stack_size]; //4096
osStaticThreadDef_t defaultTaskHandlecontrolBlock;

osMessageQId IMUQueue1Handle;
uint8_t IMUQueue1Buffer[ 256 * sizeof( imu_queue_struct ) ];
osStaticMessageQDef_t IMUQueue1ControlBlock;
osMessageQId IMUQueue2Handle;
uint8_t IMUQueue2Buffer[ 256 * sizeof( imu_queue_struct ) ];
osStaticMessageQDef_t IMUQueue2ControlBlock;
/* USER CODE BEGIN PV */
/* Shared UART2 mutex so printf and frame TX never interleave */
static SemaphoreHandle_t UART2_Mutex = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* SDL helpers */
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len);
static void stuff_and_append(uint8_t byte, uint8_t *out, uint16_t *olen);
static void sdl_send_payload_USART2(const uint8_t *payload, uint16_t len);
/* AOCS telemetry frame */
static void send_attitudeADCS_over_sdl(const float gyro[3], const float mag_T[3], uint32_t tick_ms);
/* USER CODE END PFP */

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
  /* serialize with SDL frame TX */
  if (UART2_Mutex) xSemaphoreTake(UART2_Mutex, pdMS_TO_TICKS(UART_TX_TIMEOUT_MS));
	sendDriver_UART(&huart2,&c,1);
  if (UART2_Mutex) xSemaphoreGive(UART2_Mutex);
	return c;
}


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
  /* USER CODE BEGIN 2 */

  
  // initDriver_UART();
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

  /* Create UART2 mutex (dynamic OK for CMSIS v1 with FreeRTOS backend) */
  UART2_Mutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  // /* definition and creation of IMUQueue1 */
	// osMessageQStaticDef(IMUQueue1, 512, uint32_t,IMUQueue1Buffer, &IMUQueue1ControlBlock);
	// IMUQueue1Handle = osMessageCreate(osMessageQ(IMUQueue1), NULL);
  // /* definition and creation of IMUQueue2 */
	// osMessageQStaticDef(IMUQueue2, 512, uint32_t, IMUQueue2Buffer, &IMUQueue2ControlBlock);
	// IMUQueue2Handle = osMessageCreate(osMessageQ(IMUQueue2), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0,stack_size, defaultTaskBuffer, &defaultTaskHandlecontrolBlock);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

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
  RCC_OscInitStruct.HSICalibrationValue =  RCC_HSICALIBRATION_DEFAULT;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_I2C4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* --------- SDL helpers (CRC + stuffing + frame TX) ---------- */
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint32_t i=0; i<len; ++i){
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b=0; b<8; ++b){
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else              crc = (crc << 1);
    }
  }
  return crc;
}

static void stuff_and_append(uint8_t byte, uint8_t *out, uint16_t *olen){
  if (byte == SDL_FLAG){
    out[(*olen)++] = SDL_ESC; out[(*olen)++] = SDL_ESC_7E;
  } else if (byte == SDL_ESC){
    out[(*olen)++] = SDL_ESC; out[(*olen)++] = SDL_ESC_7D;
  } else {
    out[(*olen)++] = byte;
  }
}

/* Build and send: 0x7E | header | payload | crc | 0x7E */
static void sdl_send_payload_USART2(const uint8_t *payload, uint16_t len)
{
  /* Header: DATA, ackWanted=0, rolling hash (big-endian) */
  uint8_t header[4];
  static uint16_t hash = 0;
  header[0] = SDL_CODE_DATA;
  header[1] = 0x00;
  uint16_t h = BE16(++hash);
  header[2] = (uint8_t)(h >> 8);
  header[3] = (uint8_t)(h & 0xFF);

  /* CRC over header+payload (big-endian on the wire) */
  uint8_t tmp[4 + SDL_MAX_PAYLOAD];
  memcpy(tmp, header, 4);
  memcpy(tmp+4, payload, len);
  uint16_t crc = crc16_ccitt(tmp, (uint32_t)len + 4);
  uint16_t crc_be = BE16(crc);

  /* Frame with stuffing */
  uint8_t frame[2 + 4 + SDL_MAX_PAYLOAD + 2 + 64];
  uint16_t flen = 0;
  frame[flen++] = SDL_FLAG;
  for (int i=0;i<4;i++) stuff_and_append(header[i], frame, &flen);
  for (uint16_t i=0;i<len;i++) stuff_and_append(payload[i], frame, &flen);
  stuff_and_append((uint8_t)(crc_be>>8), frame, &flen);
  stuff_and_append((uint8_t)(crc_be&0xFF), frame, &flen);
  frame[flen++] = SDL_FLAG;

  /* Serialize prints and frame through the same port */
  if (UART2_Mutex) xSemaphoreTake(UART2_Mutex, pdMS_TO_TICKS(UART_TX_TIMEOUT_MS));
  sendDriver_UART(&huart2, frame, flen);
  if (UART2_Mutex) xSemaphoreGive(UART2_Mutex);
}

static void send_attitudeADCS_over_sdl(const float gyro[3], const float mag_T[3], uint32_t tick_ms)
{
  attitudeADCS pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.code = ATTITUDEADCS_CODE; /* 21 */

  pkt.omega_x = gyro[0];
  pkt.omega_y = gyro[1];
  pkt.omega_z = gyro[2];

  pkt.b_x = mag_T[0];
  pkt.b_y = mag_T[1];
  pkt.b_z = mag_T[2];

  /* If your messages.h includes theta_* and suntheta_* they remain 0 here */
  pkt.ticktime = tick_ms;

  sdl_send_payload_USART2((uint8_t*)&pkt, (uint16_t)sizeof(pkt));
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  // huart4.gState = HAL_UART_STATE_READY;
  // huart4.RxState = HAL_UART_STATE_READY;
  // making sure that UART driver is initialized and UARTs are added after freertos started
  initDriver_UART();
	//UART2 = for printf and OBC link
  uint8_t status = addDriver_UART(&huart2, USART2_IRQn, keep_new);
	// UART4 = for IMU
  uint8_t status2 = addDriver_UART(&huart4, UART4_IRQn, keep_new);

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

  /* Send a few 0x7E flags to help SDL RX sync before prints start */
  if (UART2_Mutex) xSemaphoreTake(UART2_Mutex, pdMS_TO_TICKS(UART_TX_TIMEOUT_MS));
  for (int i=0;i<8;i++){ uint8_t f=SDL_FLAG; sendDriver_UART(&huart2,&f,1); }
  if (UART2_Mutex) xSemaphoreGive(UART2_Mutex);

	/* Infinite loop */
	for(;;)
	{
		ret=readIMUPacket(&huart4, gyro, mag, acc, 500); //mag measured in Gauss(G) unit -> 1G = 10^-4 Tesla
		mag[0]/=10000; //1G = 10^-4 Tesla
		mag[1]/=10000; //1G = 10^-4 Tesla
		mag[2]/=10000; //1G = 10^-4 Tesla

		if(ret)
		{
			if (local_imu_struct == NULL) {
				printf("IMU TASK: allocazione struttura fallita !\n");
			}
			else
			{
				for (int i = 0; i < 3; i++)
				{
					local_imu_struct->gyro_msr[i] = gyro[i];
					local_imu_struct->mag_msr[i] = mag[i];
					local_imu_struct->acc_msr[i] = acc[i];
					printf("Accelerometer axis %d, value %f \r\n", i, acc[i]);
					printf("Gyroscope axis %d, value %f \r\n", i, gyro[i]);
					printf("Magnetometer axis %d, value %f \r\n", i, mag[i]);
				}
        /* Send SDL-framed attitude packet to OBC */
        send_attitudeADCS_over_sdl(gyro, mag, HAL_GetTick());
			}
		}
		else{
			osDelay(2000);
		}
    osDelay(100);
  /* USER CODE END 5 */
  }
}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
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
