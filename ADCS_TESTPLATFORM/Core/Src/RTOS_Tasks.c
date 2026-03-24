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

	for(;;)
	{

    /* There is not need to set nor reset the CTS and RTS of UART4 for IMU because the functions 
    * UART_Transmit and UART_Receive automatically manage this if on the other side the device has enabled these two lines for UART
    * If I want to make IMU and Nucleo communicate with only the 2 UART tx and Rx lines I just have to disable the hardware flow control
    * from CubeMx.
    */

		ret=readIMUPacket(&huart4, gyro, mag, acc, 500); //mag measured in Gauss(G) unit -> 1G = 10^-4 Tesla
		mag[0]/=10000; //1G = 10^-4 Tesla
		mag[1]/=10000; //1G = 10^-4 Tesla
		mag[2]/=10000; //1G = 10^-4 Tesla
		if (xSemaphoreTake(IMURead_ControlMutex, (TickType_t)10) == pdTRUE)//If reading IMU DO NOT CONTROL
		{
			// printf("IMU Task : Taken IMURead_Control control\r\n");
			ret=readIMUPacket(&huart4, gyro, mag, acc, 500);
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

	// //printf("OBC Task: Tick_Time: %lu \r\n",HAL_GetTick());

	if (retvalue.status == osEventMessage)
	// {
		processCombinedData((void*)&retvalue,(void *)&TxHousekeeping,receive_Current_Tempqueue_OBC);
		//attitude sampling
	// 	//in this case we just send the local copy of the structure
	// 	//ALWAYS remember to set message code (use the generated defines

	// 	//printf("OBC: Trying to send attitude \r\n");
	// 	//finally we send the message
			TxHousekeeping.code=HOUSEKEEPINGADCS_CODE;
			TxHousekeeping.ticktime=HAL_GetTick();
			//printf("OBC: Trying to send housekeeping \r\n");
			//finally we send the message
      /*
			if(sdlSend(&line1,(uint8_t *)&TxHousekeeping,sizeof(housekeepingADCS),0)){
        // printf("sucess send temp current %lu \r\n",HAL_GetTick());
      }*/

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
		/*if(sdlSend(&line1,(uint8_t *)&TxAttitude,sizeof(attitudeADCS),0)){
      // printf("success send ADCS packet %lu \r\n",HAL_GetTick());
    }*/


	}

	opmodeADCS opmodeMsg;
	opmodeMsg.opmode=opmode;
	//ALWAYS remember to set message code (use the generated defines
	opmodeMsg.code=OPMODEADCS_CODE;
	//finally we send the message (WITH ACK REQUESTED)
	// printf("OBC: Trying to send opmodeADCS \r\n");
	/*if(sdlSend(&line1,(uint8_t *)&opmodeMsg,sizeof(opmodeADCS),1)){
    // printf("OBC: success to send opmodeADCS \r\n");
  }*/


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
	#if ( DEBUG_MSGS == 1 )
		printf("Error with ADC: not calibrated correctly \r\n");
	#endif
	}

	/* Infinite loop */
	for(;;)
	{
		//volatile float prev = HAL_GetTick();
		//printf("We are in CHECK TASK \r\n");
		#if ( DEBUG_MSGS == 1 )
			//printf("We are in CHECK TASK \r\n");
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
			//printf("Actuator %d current value: %f \r\n",i,currentbuf[i]);
		}
		//----------------------------------------------------------------------

		//CHECK IF THEY ARE OK
		for(int i=0;i<NUM_ACTUATORS;i++)
		{
			if(i>=0 && i<2)
			{
				//Check Reaction Wheels currents
				if(currentbuf[i] > 1.0f) //> 1A
				{
					// printf("MAgnetorquer %d current value: %f is above threshold!!!! \r\n",i,currentbuf[i]);
					error_status = 5;
				}
			}
			else{
				//Check MagneTorquers currents
				if(currentbuf[i] > 0.05f) // >50mA
				{
					// printf("Magnetorquer %d current value: %f is above threshold!!!! \r\n",i,currentbuf[i]);
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
					#if ( DEBUG_MSGS == 1 )
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
							#if ( DEBUG_MSGS == 1 )
								printf("Task check: Current n%d,value: %f,current vect:%f \r\n",i+1,local_current_temp_struct->current[i],currentbuf[i]);
							#endif
			    			// 	}
							// for(int i=NUM_ACTUATORS;i<NUM_TEMP_SENS+NUM_ACTUATORS;i++)
							// {
							// 	local_current_temp_struct->temperature[i - NUM_ACTUATORS] = ntc_values.temp[i - NUM_ACTUATORS];
							#if ( DEBUG_MSGS == 1 )
								// printf("Task check: Temperature n%d,ntc value: %f,value: %f \r\n",i-4,ntc_values.temp[i-NUM_ACTUATORS],local_current_temp_struct->temperature[i-NUM_ACTUATORS]);
							#endif
						}

						//Invio queue a OBC Task
						if (osMessagePut(ADCSHouseKeepingQueueHandle,(uint32_t)local_current_temp_struct,300) != osOK) {
							#if ( DEBUG_MSGS == 1 )
								printf("Invio a OBC Task fallito \r\n");
							#endif
			       			free(local_current_temp_struct); // Ensure the receiving task has time to process
						} else {
							#if ( DEBUG_MSGS == 1 )
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
				// Trigger an interrupt
				break;
			case 4:
				//PROBLEM WITH MAGNETORQUERS
				// Trigger an interrupt
				break;
			case 5:
				//PROBLEM WITH REACTION WHEELS
				// Trigger an interrupt
				break;

		}
		//volatile next = HAL_GetTick();
		//printf("Execussion of check task: %.1f ms\n",next-prev);
	    osDelay(2000);
	  }
  /* USER CODE END Check_current_temp */
}