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
  /* NOTE (AOCS FIX):
   * This file was patched to actually forward IMU data to the OBC and to
   * create the FreeRTOS queues you intended to use.
   *
   * Summary of changes you should keep:
   *  1) Queues: IMUQueue1/IMUQueue2 are now created with osMessageQDef/osMessageCreate.
   *     (Your previous static creation was commented out and never executed.)
   *  2) Memory: a fresh imu_queue_struct is allocated for each IMU sample using pvPortMalloc().
   *     The RECEIVER task must vPortFree() the pointer after processing.
   *  3) Telemetry: a simple ASCII frame "IMU,<tick>,ax,ay,az,gx,gy,gz,mx,my,mz\\r\\n"
   *     is sent out over USART2 using sendDriver_UART() so the OBC sees data immediately.
   *  4) UART: printf() is still routed to USART2 via sendDriver_UART().
   *
   * CubeMX guidance (safe to keep):
   *  - No peripheral settings were changed here. If you re-generate code from CubeMX,
   *    re-apply only the blocks within USER CODE sections below or merge this diff.
   *  - UART4 still uses HW flow control (RTS/CTS). If you disable HW flow on the IMU,
   *    set huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE and regenerate MSP init pins.
   *  - FreeRTOS/CMSIS-RTOS v1 APIs are used (osMessageQDef/osMessageCreate/osMessagePut).
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

/* For pvPortMalloc()/vPortFree() */
#include "FreeRTOS.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Stream IMU ASCII frames to OBC on USART2 */
#define STREAM_IMU_TO_OBC   1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ stack_size]; //4096
osStaticThreadDef_t defaultTaskHandlecontrolBlock;

/* USER CODE BEGIN PV */
/* Queues: we'll use CMSIS-RTOS v1 message queues carrying pointers (uint32_t). */
osMessageQId IMUQueue1Handle;
osMessageQId IMUQueue2Handle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
static void send_imu_ascii_frame_to_obc(uint32_t tick_ms,
                                        const float acc[3],
                                        const float gyro[3],
                                        const float mag_T[3]);
/* USER CODE END PFP */

//defining putch to enable printf
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE{
  uint8_t c=(uint8_t)ch;
  /* Route printf to USART2 via the async driver */
  sendDriver_UART(&huart2,&c,1);
  return c;
}


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
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* IMPORTANT:
   * Your original static queue creation was commented out, so IMUQueue{1,2} were NULL.
   * That made osMessagePut() fail and nothing reached your OBC/control tasks.
   * We create both queues here dynamically (pointers carried in uint32_t). */
  {
    osMessageQDef(IMUQueue1, 32, uint32_t);  /* 32 pointer slots */
    IMUQueue1Handle = osMessageCreate(osMessageQ(IMUQueue1), NULL);

    osMessageQDef(IMUQueue2, 32, uint32_t);  /* 32 pointer slots */
    IMUQueue2Handle = osMessageCreate(osMessageQ(IMUQueue2), NULL);
  }
  /* If you prefer static allocation, re-enable your osMessageQStaticDef() lines
     but MAKE SURE the buffers are sized for the selected 'type' (uint32_t) and queue length. */
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
  huart4.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS; /* If you remove HW flow on IMU, set to NONE and regenerate MSP pins */
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
static void send_imu_ascii_frame_to_obc(uint32_t tick_ms,
                                        const float acc[3],
                                        const float gyro[3],
                                        const float mag_T[3])
{
#if STREAM_IMU_TO_OBC
  /* Simple, CSV-like frame for OBC parsing.
     Keep it human-readable unless/ until you move to your messages.h binary format. */
  char line[192];
  int n = snprintf(line, sizeof(line),
                   "IMU,%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
                   (unsigned long)tick_ms,
                   acc[0], acc[1], acc[2],
                   gyro[0], gyro[1], gyro[2],
                   mag_T[0], mag_T[1], mag_T[2]);
  if (n > 0) {
    sendDriver_UART(&huart2, (uint8_t*)line, (uint16_t)n);
  }
#else
  (void)tick_ms; (void)acc; (void)gyro; (void)mag_T;
#endif
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
  /* Initialize UART driver after scheduler starts */
  initDriver_UART();
  /* UART2 = printf / OBC link */
  (void)addDriver_UART(&huart2, USART2_IRQn, keep_new);
  /* UART4 = IMU link */
  (void)addDriver_UART(&huart4, UART4_IRQn, keep_new);

#if enable_printf
  printf("Initializing IMU \r\n");
#endif
  uint8_t ret = initIMUConfig(&huart4);
#if enable_printf
  if(ret) printf("IMU correctly configured \r\n");
  else    printf("Error configuring IMU \r\n");
#endif

  float gyro[3]={0,0,0};
  float mag[3]={0,0,0};   /* Gauss from sensor; converted to Tesla below */
  float acc[3] = {0,0,0};

  /* Infinite loop */
  for(;;)
  {
    /* Read a full packet from the IMU (blocking up to 500 ms) */
    ret = readIMUPacket(&huart4, gyro, mag, acc, 500);
    /* Convert magnetometer to Tesla */
    mag[0] /= 10000.0f;
    mag[1] /= 10000.0f;
    mag[2] /= 10000.0f;

    if(ret)
    {
      /* Allocate one struct per sample; RECEIVER must vPortFree() it */
      imu_queue_struct *p = (imu_queue_struct*) pvPortMalloc(sizeof(imu_queue_struct));
      if (p == NULL) {
        printf("IMU TASK: pvPortMalloc failed\r\n");
      } else {
        for (int i = 0; i < 3; i++) {
          p->gyro_msr[i] = gyro[i];
          p->mag_msr[i]  = mag[i];
          p->acc_msr[i]  = acc[i];
        }

        /* Send pointer to Control Task if you enable it
           (uncomment once the consumer is ready to osMessageGet and vPortFree) */
        // if (IMUQueue1Handle) {
        //   if (osMessagePut(IMUQueue1Handle, (uint32_t)p, 0) != osOK) {
        //     printf("Send to Control Task failed\r\n");
        //   }
        // }

        /* Send pointer to OBC Task */
        if (IMUQueue2Handle) {
          if (osMessagePut(IMUQueue2Handle, (uint32_t)p, 0) != osOK) {
            printf("Send to OBC Task failed\r\n");
            vPortFree(p); /* no consumer -> free here */
          } else {
            /* Optional debug */
            // printf("IMU sample enqueued to OBC task\r\n");
          }
        } else {
          /* Queue not created: free and warn */
          vPortFree(p);
          printf("IMUQueue2Handle is NULL\r\n");
        }
      }

      /* Also stream an ASCII frame over USART2 so the OBC gets data immediately */
      send_imu_ascii_frame_to_obc(HAL_GetTick(), acc, gyro, mag);

#if enable_printf
      /* Throttled debug print; comment out if too chatty */
      for (uint32_t i = 0; i < 3; i++) {
        printf("ACC[%lu]=%f  GYR[%lu]=%f  MAG[%lu]=%f\r\n",
               (unsigned long)i, acc[i],
               (unsigned long)i, gyro[i],
               (unsigned long)i, mag[i]);
      }
#endif

    } else {
      /* IMU read timeout or parse error; back off a bit */
      osDelay(200);
    }

    osDelay(100);
  }
  /* USER CODE END 5 */
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
