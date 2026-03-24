#include "RTOS_Tasks.h"

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
	uint32_t start_time;
	uint8_t count = 0;


	float gyro[3]={1,2,3};
		float mag[3]={4,5,6};
		float acc[3] = {7,8,9};
	float m_con[3] = {0,0,0};


	float k = 50.0f; // BDOT gain
	float duty_cycle[3] = {0,0,0};
	uint8_t direction[3] = {0,0,0};

	// Original ones:
	//const float coil_turn[3] = {300.0f, 300.0f, 210.0f}; //number of turns of the

	// Test tweaked values:
	const float coil_turn[3] = {600.0f, 600.0f, 600.0f}; //number of turns of the

	const float coil_area[3] = {0.007225f, 0.007225f, 0.007225f}; //coil area in m^2
	const float re_coil[3] = {30.7f, 30.7f, 23.0f}; //coil resistance in ohm
	const float VDD_coil[3] = {12.0f, 12.0f, 12.0f}; //coil supply voltage
	Actuator_struct* coils[3] = {&Reaction1,&Reaction2,&MagneTorquer1}; //coil
	static uint8_t active[3] = {0,0,0};

		imu_queue_struct *local_imu_struct1 =(imu_queue_struct*) malloc(sizeof(imu_queue_struct));
		//Inizialize actuators struct
		init_actuator_handler(&Reaction1,&htim1,TIM_CHANNEL_1,TIM_CHANNEL_2,100000,50); //100 khz
		init_actuator_handler(&Reaction2,&htim2,TIM_CHANNEL_3,TIM_CHANNEL_4,20000,50);
		init_actuator_handler(&MagneTorquer1,&htim3,TIM_CHANNEL_1,TIM_CHANNEL_2,89000,50); //89 khz //this measured 10khz, idkwhy
		// init_actuator_handler(&MagneTorquer2,&htim3,TIM_CHANNEL_3,TIM_CHANNEL_4,10000,50); //also this
		// init_actuator_handler(&MagneTorquer3,&htim2,TIM_CHANNEL_1,TIM_CHANNEL_2,94000,50); //94 khz // this measured 100khz, idkwhy
	//12332
		//Inizialize PID struct
		PID_INIT(&PID_Inputs);

	/* Infinite loop */
	for(;;)
	{
		//printf("We are in Control Algorithm TASK \r\n");
		#if ( DEBUG_MSGS_CONTROL == 1 )
			//printf("We are in Control Algorithm TASK \r\n");
		#endif
		// printf("I am alive from Control_Algorithm_Task at %lu ms\r\n", HAL_GetTick());
			//Receive Telemetry IMU via Queue

			// retvalue1 = osMessageGet(setAttitudeADCSQueueHandle,200);
			// processCombinedData((void*)&retvalue1,(void *)&PID_Inputs,receive_Attitudequeue_control);
		// the reason why i commented the above is that there is no task sending to that queue therefore its technically useless

			retvalue = osMessageGet(IMUQueue1Handle, 300);
			processCombinedData((void*)&retvalue,(void *)&local_imu_struct1,receive_IMUqueue_control);
		//algorithm
		#if( DEBUG_MSGS_CONTROL )
		printf("\r\n\r\n");
		#endif
		
		for (int i = 0; i < 3; i++)
		{
		gyro[i] = local_imu_struct1->gyro_msr[i];
		mag[i] = local_imu_struct1->mag_msr[i];
		acc[i] = local_imu_struct1->acc_msr[i];
		#if ( DEBUG_MSGS_CONTROL == 1 )
			printf("\r\n ====  IMU VALUES ==== \r\n");
			printf("Accelerometer axis %d, value %f \r\n", i, acc[i]);
			printf("Gyroscope axis %d, value %f \r\n", i, gyro[i]);
			printf("Magnetometer axis %d, value %f \r\n", i, mag[i]);
		#endif
		}
			// ALGORITHM
		compute_mcon(mag, gyro, k, m_con);  // Compute angular momentum
		compute_duty_cycle(m_con, coil_turn, coil_area, re_coil, VDD_coil, duty_cycle, direction); 

		for (int i = 0; i < 3; i++)
		{
		#if ( DEBUG_MSGS_CONTROL == 1 )
			printf("\r\n====  COMPUTED VALUES ====\r\n");
			printf("duty cycle %f, direction %d \r\n", duty_cycle[i], direction[i]);
			printf("Magnetic Dipole Moment axis %d, value %f \r\n", i, m_con[i]);
			// printf("Duty Cycle axis %d, value %f \r\n", i, duty_cycle[i]);
			// printf("Direction axis %d, value %d \r\n", i, direction[i]);
		#endif
		}

		count++;

		for (int i = 0; i < 3; i++) {
		// clamp and optional lower threshold during bring-up
			if (duty_cycle[i] < 0.0f) duty_cycle[i] = 0.0f;
			if (duty_cycle[i] > 100.0f) duty_cycle[i] = 100.0f;

			update_duty_dir(coils[i], duty_cycle[i], direction[i]);  // first update the duty cycle
			if (duty_cycle[i] > 20.0f) {
				if (!active[i]) {
					#if ( DEBUG_MSGS_CONTROL )
						printf("Coil %d started with direction %d\r\n", i, direction[i]);
					#endif
					//actuator_START(coils[i]);              // start once per axis
					active[i] = 1;
				}
			} else {
				if (active[i]) {
					#if ( DEBUG_MSGS_CONTROL )
						printf("Coil %d stopped\r\n", i);
					#endif
					//actuator_STOP(coils[i]);               // stop only if previously active
					active[i] = 0;
				}
			}
		}


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


		if (count >= 5){
		if (xSemaphoreTake(IMURead_ControlMutex, (TickType_t)10) == pdTRUE) //If control don't read IMU
		{
			// printf("Control Task : Taken IMURead_ControlMutex control \r\n");
			//Spegnere i magnetorquer
			xSemaphoreGive(IMURead_ControlMutex);
		//   printf("Control Task : Released IMURead_ControlMutex control \r\n");
		}
		}
		free(local_imu_struct1);   // TODO must be checked
		osDelay(2000);
	}
	/* USER CODE END Control_Algorithm_Task */
}

