#include "RTOS_Tasks.h"

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