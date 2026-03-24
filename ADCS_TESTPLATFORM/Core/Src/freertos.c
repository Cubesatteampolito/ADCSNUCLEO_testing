/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "freertos.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Defining Task related variables */ 
osThreadId IMUTaskHandle; 
uint32_t IMUTaskBuffer[stack_size]; 
// 4096 osStaticThreadDef_t IMUTaskControlBlock; 
osThreadId OBC_CommTaskHandle; 
uint32_t OBC_CommTaskBuffer[stack_size1]; 
// 16384 osStaticThreadDef_t OBC_CommTaskControlBlock; 
osThreadId ControlAlgorithmTaskHandle; 
uint32_t ControlAlgorithmTaskBuffer[stack_size]; 
// 4096 osStaticThreadDef_t ControlAlgorithmTaskControlBlock; 
osThreadId FirstCheckTaskHandle; 
uint32_t FirstCheckTaskBuffer[stack_size]; 
// 4096 osStaticThreadDef_t FirstCheckTaskControlBlock; 
xSemaphoreHandle IMURead_ControlMutex; 
StaticSemaphore_t xIMURead_ControlMutexBuffer; 
osMessageQId ADCSHouseKeepingQueueHandle; 
uint8_t ADCSHouseKeepingQueueBuffer[256 * sizeof(float)]; 
osStaticMessageQDef_t ADCSHouseKeepingQueueControlBlock; 
osMessageQId IMUQueue2Handle; 
uint8_t IMUQueue2Buffer[256 * sizeof(imu_queue_struct)];
osStaticMessageQDef_t IMUQueue2ControlBlock; 
osMessageQId IMUQueue1Handle; 
uint8_t IMUQueue1Buffer[256 * sizeof(imu_queue_struct)]; 
osStaticMessageQDef_t IMUQueue1ControlBlock; 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void MX_FREERTOS_Init(void);
/* USER CODE END FunctionPrototypes */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* Function to initialize FreeRTOS */
void MX_FREERTOS_Init(void)
{
  /* Create Mutex and Semaphore */
  IMURead_ControlMutex = xSemaphoreCreateMutexStatic(&xIMURead_ControlMutexBuffer);
  configASSERT(IMURead_ControlMutex);
  xSemaphoreGive(IMURead_ControlMutex);

  /* Create RTOS QUEUES: IMU1, IMU2 and housekeeping */
  osMessageQStaticDef (IMUQueue1, 512, uint32_t, IMUQueue1Buffer, &IMUQueue1ControlBlock);
  osMessageQStaticDef (IMUQueue2, 512, uint32_t, IMUQueue2Buffer, &IMUQueue2ControlBlock);
  osMessageQStaticDef (ADCSHouseKeepingQueue, 512, uint32_t, ADCSHouseKeepingQueueBuffer, &ADCSHouseKeepingQueueControlBlock);
  IMUQueue1Handle             = osMessageCreate(osMessageQ(IMUQueue1), NULL);
  IMUQueue2Handle             = osMessageCreate(osMessageQ(IMUQueue2), NULL);
  ADCSHouseKeepingQueueHandle = osMessageCreate(osMessageQ(ADCSHouseKeepingQueue), NULL);
  
  /* Create RTOS threads: FirstCheckTask, IMUTask, OBC_CommTask, ControlAlgorithmTask */
  osThreadStaticDef (FirstCheckTask, Check_current_temp, osPriorityAboveNormal, 0, stack_size, FirstCheckTaskBuffer, &FirstCheckTaskControlBlock);
  osThreadStaticDef (IMUTask, IMU_Task, osPriorityNormal, 0, stack_size, IMUTaskBuffer, &IMUTaskControlBlock);
  osThreadStaticDef (OBC_CommTask, OBC_Comm_Task, osPriorityAboveNormal, 0, stack_size1, OBC_CommTaskBuffer, &OBC_CommTaskControlBlock);
  osThreadStaticDef (ControlAlgorithmTask, Control_Algorithm_Task, osPriorityNormal, 0, stack_size, ControlAlgorithmTaskBuffer, &ControlAlgorithmTaskControlBlock);
  FirstCheckTaskHandle        = osThreadCreate(osThread(FirstCheckTask), NULL);
  IMUTaskHandle               = osThreadCreate(osThread(IMUTask), NULL);
  OBC_CommTaskHandle          = osThreadCreate(osThread(OBC_CommTask), NULL);
  ControlAlgorithmTaskHandle  = osThreadCreate(osThread(ControlAlgorithmTask), NULL);
}

/* USER CODE END Application */
