#include "RTOS_Tasks.h"

/* RTOS thread (task) responsible for handling communication between the Attitude Determination and Control System (ADCS) and the On-Board Computer (OBC).
 * It collects data from queues, packages telemetry and sends it over UART using a serial protocol (SDL). 
 */

void OBC_Comm_Task(void const * argument)
{
    /* THREAD INITIALIZATION */
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

