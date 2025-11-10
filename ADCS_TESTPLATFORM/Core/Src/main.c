/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : AOCS → OBC using simpleDataLink (no interleaved prints)
  ******************************************************************************
  */
 /* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdbool.h>
#include <math.h>

#include "UARTdriver.h"
#include "MTi1.h"
#include "messages.h"
#include "queue_structs.h"
#include "constants.h"
#include "FreeRTOS.h"
#include "task.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
/* ---- simpleDataLink minimal TX ---- */
#define SDL_FLAG       0x7E
#define SDL_ESC        0x7D
#define SDL_ESC_7E     0x5E
#define SDL_ESC_7D     0x5D
#define SDL_CODE_DATA  0x01u
#define UART_TX_TIMEOUT_MS 300
#define SDL_MAX_PAYLOAD 240

/* Route printf somewhere else (0: drop; 1: USART2; 2: SWO ITM if enabled) */
#define OBC_UART_PRINTF 0
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4; /* IMU */
UART_HandleTypeDef huart2; /* OBC link */

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ stack_size];
osStaticThreadDef_t defaultTaskHandlecontrolBlock;

/* USER CODE BEGIN PV */
osMessageQId IMUQueue2Handle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len);
static uint16_t be16(uint16_t v);
static void sdl_send_payload_USART2(const uint8_t *payload, uint16_t len);
static void send_attitudeADCS_over_sdl(const float gyro[3], const float mag_T[3], uint32_t tick_ms);
/* USER CODE END PFP */

/* redirect printf (avoid USART2 to prevent corrupting frames) */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE{
#if OBC_UART_PRINTF == 1
  uint8_t c=(uint8_t)ch;
  sendDriver_UART(&huart2,&c,1);
  return c;
#elif OBC_UART_PRINTF == 2
  ITM_SendChar(ch);
  return ch;
#else
  (void)ch;
  return 0;
#endif
}

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();

  /* USER CODE BEGIN RTOS_QUEUES */
  osMessageQDef(IMUQueue2, 32, uint32_t);
  IMUQueue2Handle = osMessageCreate(osMessageQ(IMUQueue2), NULL);
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
  (void)addDriver_UART(&huart2, USART2_IRQn, keep_new); /* OBC link */
  (void)addDriver_UART(&huart4, UART4_IRQn, keep_new);  /* IMU */

  /* Optional: initial idle flags to help SDL resync */
  for (int i=0;i<16;i++){ uint8_t f=0x7E; sendDriver_UART(&huart2,&f,1); }
  vTaskDelay(pdMS_TO_TICKS(50));

  uint8_t ret = initIMUConfig(&huart4);

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

      /* Optional internal queue (300 ms timeout) */
      imu_queue_struct *p = (imu_queue_struct*) pvPortMalloc(sizeof(imu_queue_struct));
      if (p) {
        for (int i=0;i<3;i++){ p->gyro_msr[i]=gyro[i]; p->mag_msr[i]=magT[i]; p->acc_msr[i]=acc[i]; }
        if (IMUQueue2Handle) {
          if (osMessagePut(IMUQueue2Handle, (uint32_t)p, 300) != osOK) { vPortFree(p); }
        } else { vPortFree(p); }
      }

      /* Send SDL frame */
      send_attitudeADCS_over_sdl(gyro, magT, HAL_GetTick());
    } else {
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN 4 */
static uint16_t be16(uint16_t v){ return (uint16_t)((v>>8) | (v<<8)); }

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

/* Build and send frame via sendDriver_UART (no HAL on same UART) */
static void sdl_send_payload_USART2(const uint8_t *payload, uint16_t len)
{
  /* Header: code, ackWanted, hash (be16) */
  uint8_t header[4];
  static uint16_t hash = 0;
  header[0] = SDL_CODE_DATA;
  header[1] = 0x00; /* ackWanted = 0 for telemetry */
  uint16_t h = be16(++hash);
  header[2] = (uint8_t)(h >> 8);
  header[3] = (uint8_t)(h & 0xFF);

  /* Compute CRC on header+payload */
  uint8_t tmp[4 + SDL_MAX_PAYLOAD];
  memcpy(tmp, header, 4);
  memcpy(tmp+4, payload, len);
  uint16_t crc = crc16_ccitt(tmp, (uint32_t)len + 4);
  uint16_t crc_be = be16(crc);

  /* Build stuffed frame */
  uint8_t frame[2 /*flags*/ + 4 + SDL_MAX_PAYLOAD + 2 /*crc*/ + 64 /*stuff*/];
  uint16_t flen = 0;
  frame[flen++] = SDL_FLAG;
  for (int i=0;i<4;i++) stuff_and_append(header[i], frame, &flen);
  for (uint16_t i=0;i<len;i++) stuff_and_append(payload[i], frame, &flen);
  stuff_and_append((uint8_t)(crc_be>>8), frame, &flen);
  stuff_and_append((uint8_t)(crc_be&0xFF), frame, &flen);
  frame[flen++] = SDL_FLAG;

  /* Send atomically through the same IRQ-driven driver used by printf to avoid HAL/IRQ contention */
  sendDriver_UART(&huart2, frame, flen);
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

  /* theta_* and suntheta_* left = 0 if not defined in header */
  pkt.ticktime = tick_ms;

  sdl_send_payload_USART2((uint8_t*)&pkt, (uint16_t)sizeof(pkt));
}
/* USER CODE END 4 */

/* ===== CubeMX glue (unchanged) ===== */
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
