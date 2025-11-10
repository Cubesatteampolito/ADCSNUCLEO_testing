/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (AOCS → OBC binary telemetry, no theta/sun fields)
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
#include "UARTdriver.h"
#include <math.h>
#include <stdbool.h>
#include "MTi1.h"
#include "messages.h"      // MUST be the same header the OBC uses
#include "queue_structs.h"
#include "constants.h"

#include "FreeRTOS.h"
#include "task.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_TX_TIMEOUT_MS      300
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ stack_size]; //4096
osStaticThreadDef_t defaultTaskHandlecontrolBlock;

/* USER CODE BEGIN PV */
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
static void send_attitudeADCS_binary_over_usart2(const float gyro[3],
                                                 const float mag_T[3],
                                                 uint32_t tick_ms);
/* USER CODE END PFP */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE{
  uint8_t c=(uint8_t)ch;
  sendDriver_UART(&huart2,&c,1);
  return c;
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();

  /* USER CODE BEGIN RTOS_QUEUES */
  {
    osMessageQDef(IMUQueue1, 32, uint32_t);
    IMUQueue1Handle = osMessageCreate(osMessageQ(IMUQueue1), NULL);

    osMessageQDef(IMUQueue2, 32, uint32_t);
    IMUQueue2Handle = osMessageCreate(osMessageQ(IMUQueue2), NULL);
  }
  /* USER CODE END RTOS_QUEUES */

  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0,stack_size, defaultTaskBuffer, &defaultTaskHandlecontrolBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
  osKernelStart();
  while (1) {}
}

static void MX_UART4_Init(void)
{
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS; /* set NONE if not used */
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK) { Error_Handler(); }
}

static void MX_USART2_UART_Init(void)
{
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
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  initDriver_UART();
  (void)addDriver_UART(&huart2, USART2_IRQn, keep_new);
  (void)addDriver_UART(&huart4, UART4_IRQn, keep_new);

  printf("AOCS: Initializing IMU\r\n");
  uint8_t ret = initIMUConfig(&huart4);
  printf("AOCS: IMU %s\r\n", ret ? "OK" : "FAIL");

  float gyro[3]={0,0,0};
  float magG[3]={0,0,0};
  float magT[3]={0,0,0};
  float acc[3]={0,0,0};

  for(;;)
  {
    ret = readIMUPacket(&huart4, gyro, magG, acc, 500);
    if (ret)
    {
      magT[0] = magG[0] / 10000.0f;
      magT[1] = magG[1] / 10000.0f;
      magT[2] = magG[2] / 10000.0f;

      /* Queue to internal OBC task (300 ms timeout) */
      imu_queue_struct *p = (imu_queue_struct*) pvPortMalloc(sizeof(imu_queue_struct));
      if (p) {
        for (int i=0;i<3;i++){ p->gyro_msr[i]=gyro[i]; p->mag_msr[i]=magT[i]; p->acc_msr[i]=acc[i]; }
        if (IMUQueue2Handle) {
          if (osMessagePut(IMUQueue2Handle, (uint32_t)p, 300) != osOK) {
            printf("IMU→OBC queue: send timeout/fail\r\n");
            vPortFree(p);
          }
        } else {
          printf("IMU→OBC queue: handle NULL\r\n");
          vPortFree(p);
        }
      } else {
        printf("IMU: pvPortMalloc failed\r\n");
      }

      /* Send the binary frame to the external OBC link */
      send_attitudeADCS_binary_over_usart2(gyro, magT, HAL_GetTick());
    } else {
      osDelay(200);
    }

    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN 4 */
/* Pack ONLY the fields that exist in your messages.h attitudeADCS */
static void send_attitudeADCS_binary_over_usart2(const float gyro[3],
                                                 const float mag_T[3],
                                                 uint32_t tick_ms)
{
  attitudeADCS frame;
  memset(&frame, 0, sizeof(frame)); /* in case messages.h adds fields later */
  frame.code     = ATTITUDEADCS_CODE; /* 21 or as defined in your header */

  /* Required members present in your header:
     - omega_x, omega_y, omega_z
     - b_x, b_y, b_z
     - ticktime
     (No theta_* or suntheta_* in this version.) */
  frame.omega_x  = gyro[0];
  frame.omega_y  = gyro[1];
  frame.omega_z  = gyro[2];

  frame.b_x      = mag_T[0];
  frame.b_y      = mag_T[1];
  frame.b_z      = mag_T[2];

  frame.ticktime = tick_ms;

  (void)HAL_UART_Transmit(&huart2, (uint8_t*)&frame, (uint16_t)sizeof(frame), UART_TX_TIMEOUT_MS);
}
/* USER CODE END 4 */

/* ===== CubeMX glue kept as-is ===== */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) { Error_Handler(); }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_I2C4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) { HAL_IncTick(); }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
