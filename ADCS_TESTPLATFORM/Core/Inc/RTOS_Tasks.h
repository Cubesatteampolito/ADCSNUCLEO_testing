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
uint8_t IMUQueue1Buffer[ 256 * sizeof(imu_queue_struct)]; 
osStaticMessageQDef_t IMUQueue1ControlBlock; 

/* Readings from IMU */ 
void IMU_Task(void const * argument); 
/* Communication with OBC - by now simulated with py program */ 
void OBC_Comm_Task(void const * argument); 
/* Attitude control logic */ 
void Control_Algorithm_Task(void const * argument); 
/* Current & Temperature monitors */ 
void Check_current_temp(void const * argument);