#include "RTOS_Tasks.h"

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

  #if ( DEBUG_MSGS == 1 )
	  printf("Initializing IMU \r\n");
  #endif
    //uint8_t ret = 1;
    uint8_t ret = initIMUConfig(&huart4);
  #if ( DEBUG_MSGS == 1 )
    if(ret) printf("IMU correctly configured \r\n");
    else printf("Error configuring IMU \r\n");
  #endif

	float gyro[3]={1,2,3};
	float mag[3]={4,5,6};
	float acc[3] = {7,8,9};
  float m_con[3] = {0,0,0},  k = 50.0f;

	imu_queue_struct *local_imu_struct =(imu_queue_struct*) malloc(sizeof(imu_queue_struct));

	/* Infinite loop that runs forever under RTOS scheduling */
	for(;;)
	{

    /* There is not need to set nor reset the CTS and RTS of UART4 for IMU because the functions 
    * UART_Transmit and UART_Receive automatically manage this if on the other side the device has enabled these two lines for UART
    * If I want to make IMU and Nucleo communicate with only the 2 UART tx and Rx lines I just have to disable the hardware flow control
    * from CubeMx.
    */

		ret = readIMUPacket(&huart4, gyro, mag, acc, 500); // mag measured in Gauss(G) unit -> 1G = 10^-4 Tesla
		mag[0] /= 10000; //1G = 10^-4 Tesla
		mag[1] /= 10000; //1G = 10^-4 Tesla
		mag[2] /= 10000; //1G = 10^-4 Tesla

		/* Mutex protection: while reading the IMU, the control task must not interfere */
		if (xSemaphoreTake(IMURead_ControlMutex, (TickType_t) 10) == pdTRUE) //If reading IMU DO NOT CONTROL
		{
			// printf("IMU Task : Taken IMURead_Control control\r\n");
			ret = readIMUPacket(&huart4, gyro, mag, acc, 500);
			xSemaphoreGive(IMURead_ControlMutex);
			// printf("IMU Task : Released IMURead_Control control\r\n");
		}
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
				#if (DEBUG_MSGS == 1) 
          printf("IMU Task: Memory allocation for IMU struct failed \r\n");
        #endif
			}
			else
			{
				//Fill the struct with values read from IMU, then send them to Control Task
				for (int i = 0; i < 3; i++)
				{
					local_imu_struct->gyro_msr[i] = gyro[i];
					local_imu_struct->mag_msr[i] = mag[i];
					local_imu_struct->acc_msr[i] = acc[i];
					// printf("Accelerometer axis %d, value %f \r\n", i, acc[i]);
					// printf("Gyroscope axis %d, value %f \r\n", i, gyro[i]);
					// printf("Magnetometer axis %d, value %f \r\n", i, mag[i]);
				}

        /* BDOT IMPLEMENTATION HERE */
        // compute_mcon(mag, gyro, k, m_con);
        // printf("Dipole Moment: %f %f %f \r\n", m_con[0], m_con[1], m_con[2]);
        // T = m x B
        /* BDOT FINISHED */
				//Invio queue a Control Task
			 	if (osMessagePut(IMUQueue1Handle,(uint32_t)local_imu_struct,300) != osOK) {
			    	//printf("Invio a Control Task fallito \r\n");
			       	free(local_imu_struct); // Ensure the receiving task has time to process
				} else {
			        // printf("Dati Inviati a Control Task \r\n");

			 	}
			 	//Invio queue a OBC Task
			 	if (osMessagePut(IMUQueue2Handle,(uint32_t)local_imu_struct,300) != osOK) {
			    	//printf("Invio a OBC Task fallito \r\n");
			       	free(local_imu_struct); // Ensure the receiving task has time to process
			 	} else {
			    	//printf("Dati a Control Inviati \r\n");
				}
			}
		}
		else{
			//printf("IMU: Error configuring IMU \r\n");
			osDelay(2000);
		}
    // printf("Hello from STM32L4\r\n");
    osDelay(100);
  }
  /* USER CODE END 5 */
}

