/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : AOCS → OBC with simpleDataLink (prints off UART2)
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
#define SDL_MAX_PAYLOAD 128 /* OBC reports 128 in your log */
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4; /* IMU */
UART_HandleTypeDef huart2; /* OBC link */

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ stack_size];
osStaticThreadDef_t defaultTaskHandlecontrolBlock;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len);
static void sdl_send_payload_USART2(const uint8_t *payload, uint16_t len);
static void send_attitudeADCS_over_sdl(const float gyro[3], const float mag_T[3], uint32_t tick_ms);
/* USER CODE END PFP */

/* redirect printf AWAY from USART2 (no gibberish): use SWO ITM */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE{
  /* If SWO not enabled, this will be ignored by host; UART2 remains clean */
  ITM_SendChar(ch);
  return ch;
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
  huart4.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS; /* set NONE if IMU doesn't use HW flow control */
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
  (void)addDriver_UART(&huart2, USART2_IRQn, keep_new); /* OBC link via IRQ driver */
  (void)addDriver_UART(&huart4, UART4_IRQn, keep_new);  /* IMU */

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

      send_attitudeADCS_over_sdl(gyro, magT, HAL_GetTick());
    } else {
      vTaskDelay(pdMS_TO_TICKS(200));
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN 4 */
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

/* LITTLE-ENDIAN header and CRC on the wire (hash_lo, hash_hi, crc_lo, crc_hi) */
static void sdl_send_payload_USART2(const uint8_t *payload, uint16_t len)
{
  /* Header: DATA, ackWanted=0, rolling hash (little-endian) */
  uint8_t header[4];
  static uint16_t hash = 0;
  header[0] = SDL_CODE_DATA;
  header[1] = 0x00; /* no ack */
  uint16_t h = ++hash;
  header[2] = (uint8_t)(h & 0xFF);      /* hash_lo */
  header[3] = (uint8_t)((h >> 8) & 0xFF); /* hash_hi */

  /* CRC over header+payload */
  uint8_t tmp[4 + SDL_MAX_PAYLOAD];
  memcpy(tmp, header, 4);
  memcpy(tmp+4, payload, len);
  uint16_t crc = crc16_ccitt(tmp, (uint32_t)len + 4);

  /* Build stuffed frame */
  uint8_t frame[2 + 4 + SDL_MAX_PAYLOAD + 2 + 64];
  uint16_t flen = 0;
  frame[flen++] = SDL_FLAG;
  for (int i=0;i<4;i++) stuff_and_append(header[i], frame, &flen);
  for (uint16_t i=0;i<len;i++) stuff_and_append(payload[i], frame, &flen);
  stuff_and_append((uint8_t)(crc & 0xFF), frame, &flen);       /* crc_lo */
  stuff_and_append((uint8_t)((crc >> 8) & 0xFF), frame, &flen);/* crc_hi */
  frame[flen++] = SDL_FLAG;

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
