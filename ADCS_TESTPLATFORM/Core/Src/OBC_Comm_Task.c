#include "RTOS_Tasks.h"

/* RTOS thread (task) responsible for handling communication between the Attitude Determination and Control System (ADCS) and the On-Board Computer (OBC).
 * (1) Initializes communication (UART + serial SDL protocol)
 * (2) Waits for data from other tasks (queues)
 * (3) Converts that data into packets
 * (4) Sends packets to OBC
 * (5) Repeats periodically
 */

 // TODO: make it so that the OBC can set the ADCS's opcode, right now it's harcored to 0

void OBC_Comm_Task(void const * argument)
{
	/* INITIALIZATION */

	/* Initialize low-level UART driver */
	addDriver_UART(&huart1, UASRT1_IRQn, keep_old);

	/* Initialize protocol later on top of UART */
	static serial_line_handle line1;
	sdlinitLine(&line1, &txFunc1, &rxFunc1, 50, 2);

	housekeepingADCS txHousekeeping;
	attitudeADCS txAttitude;
	opmodeADCS opmodeMsg;

	uint8_t opmode = 0;

	/* MAIN LOOP */
	for(;;)
	{
		/* === HOUSEKEEPING === */
		osEvent hkEvt = osMessageGet(ADCHouseKeepingQueueHandle, 300);
		if (evt.status == osEventMessage)
		{
			/* Extract queue data and fill a packet */
			processCombinedData((void*)&hkEvt, (void*)&txHousekeeping, receive_Current_Tempqueue_OBC);

			/* Add metadata (code + timestamp) to the queue data */
			txHousekeeping.code = HOUSEKEEPINGADCS_CODE;
			txHousekeeping.ticktime = HAL_GetTick();

			/* Send the data */
			sdlSend(&line1, (uint8_t*)&txHousekeeping, sizeof(housekeepingADCS), 0);
		}

		/* === IMU === */
		attitudeADCS txAttitude;
		osEvent imuEvt = osMessageGet(IMUQueue2Handle, 300);
		if (imuEvt.status == osEventMessage)
		{
			processCombinedData((void*)&imuEvt, (void*)&txAttitude, receive_IMUqueue_OBC);

			txAttitude.code = ATTITUDEADCS_CODE;
			txAttitude.ticktime = HAL_GetTick();

			sdlSend(&line1, (uint8_t*)&txAttitude, sizeof(attitudeADCS), 0);
		}
		
		/* === OPMODE (tells the OBC what operation mode the system is in [safe, Sun pointing, detumbling, Science mode...]) === */
		opmodeMsg.opmode = opmode;
		opmodeMsg.code = OPMODEADCS_CODE;

		sdlSend(&line1, (uint8_t*)&opmodeMsg, sizeof(opmodeADCS), 1);

		osDelay(2000); 
	}
}
