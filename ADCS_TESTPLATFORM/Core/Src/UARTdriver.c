/* UART lines driver */
#include "UARTdriver.h"

typedef struct DriverHandle_UART
{
    volatile uint8_t _usageFlag;                  		// To flag that the structure is valid (0 if not used)
    uint8_t _rxByte;                        			// Where the HAL ISR will store the received byte
    uint8_t _txByte;                        			// Where the HAL ISR will store the sent byte
    UART_HandleTypeDef* _huartHandle;       			// UART handle
    IRQn_Type _irq;										// IRQ number of the uart
    QueueHandle_t	_rxQueueHandle;						// RX queue handle
    uint8_t _rxQueueStorageBuffer[SERIAL_RX_BUFF_LEN];	// RX queue data buffer
    StaticQueue_t _rxQueueBuffer;						// RX queue buffer
    QueueHandle_t	_txQueueHandle;						// TX queue handle
    uint8_t _txQueueStorageBuffer[SERIAL_TX_BUFF_LEN];	// TX queue data buffer
    StaticQueue_t _txQueueBuffer;						// TX queue buffer
    fifo_policy _policyRX;								// RC buffer policy
} DriverHandle_UART;

volatile DriverHandle_UART _driverHandle_UART[MAX_UART_HANDLE]; 	// Handle structures array

/* Initialize the UART data structure */
void initDriver_UART()
{
    for(uint32_t handleIndex = 0; handleIndex < MAX_UART_HANDLE; handleIndex++)
    {
        _driverHandle_UART[handleIndex]._usageFlag = 0;
    }
}

/* Register the UART driver */
uint8_t addDriver_UART(
    UART_HandleTypeDef* huartHandle,    //
    IRQn_Type irq,                      //
    fifo_policy policyRX                //
){
    /* Scan the data structure to find a free position (or if the handle is already inserted) */
    for(uint32_t handleIndex = 0; handleIndex < MAX_UART_HANDLE; handleIndex++)
    {
        // If it finds an occupied position
        if(_driverHandle_UART[handleIndex]._usageFlag == 1)
        {
            // If the handle is already inside the structure
            if(_driverHandle_UART[handleIndex]._huartHandle == huartHandle)
            {
                // error
                return 1; 
            }
        }
        else
        {
        	// Disable the IRQ
        	uint32_t irqState=NVIC_GetEnableIRQ(irq);
        	NVIC_DisableIRQ(irq);

            // Initialize the strcture for this handle
            _driverHandle_UART[handleIndex]._huartHandle = huartHandle;
            _driverHandle_UART[handleIndex]._irq = irq; // store IRQ if it doesnt works then remove this on next commit
            _driverHandle_UART[handleIndex]._rxQueueHandle = xQueueCreateStatic(SERIAL_RX_BUFF_LEN,1,(void*)&_driverHandle_UART[handleIndex]._rxQueueStorageBuffer,&_driverHandle_UART[handleIndex]._rxQueueBuffer);
            _driverHandle_UART[handleIndex]._txQueueHandle = xQueueCreateStatic(SERIAL_TX_BUFF_LEN,1,(void*)&_driverHandle_UART[handleIndex]._txQueueStorageBuffer,&_driverHandle_UART[handleIndex]._txQueueBuffer);
            _driverHandle_UART[handleIndex]._usageFlag = 1;
            _driverHandle_UART[handleIndex]._policyRX = policyRX;

            HAL_UART_Receive_IT(huartHandle,&_driverHandle_UART[handleIndex]._rxByte,1);

            if(irqState) NVIC_EnableIRQ(irq);

            return 0;
        }
    }
    return 1;
}

/* Receive data from UART line */
uint32_t receiveDriver_UART(UART_HandleTypeDef* huartHandle, uint8_t* buff, uint32_t size){

    if(size == 0) return 0;

    // Scanning the array with the structures to find the handle
    for(uint32_t handleIndex = 0; handleIndex < MAX_UART_HANDLE; handleIndex++)
    {
        // If it finds the handle
        if((_driverHandle_UART[handleIndex]._usageFlag == 1) && (huartHandle == _driverHandle_UART[handleIndex]._huartHandle))
        {
        	uint8_t rxNum=0;
        	while(rxNum < size && xQueueReceive(_driverHandle_UART[handleIndex]._rxQueueHandle, &buff[rxNum], 0) == pdTRUE){
        		rxNum++;
        	}

            // 0 bytes read
            return rxNum;

            /*
            * In case reception crashed and error handle didn't relaunch it, relaunch it (this should not happen)
            * disable the IRQ
            * Removed the part that disables interrupt to avoid losing packets if this function is
            * preempted, execution SHOULD be IRQ safe anyway
        	* NVIC_DisableIRQ(_driverHandle_UART[handleIndex]._irq);
            * HAL_UART_Receive_IT(huartHandle,&_driverHandle_UART[handleIndex]._rxByte,1);
            * NVIC_EnableIRQ(_driverHandle_UART[handleIndex]._irq);
            */

        }
    }
    return 0;
}

/* Transmit data on UART line */
uint32_t sendDriver_UART(UART_HandleTypeDef* huartHandle,uint8_t* buff,uint32_t size){
	if(size == 0) return 0;

	// Scanning the array with the structures to find the handle
	for(uint32_t handleIndex = 0; handleIndex < MAX_UART_HANDLE; handleIndex++)
	{
		// If it finds the handle
		if((_driverHandle_UART[handleIndex]._usageFlag == 1) && (huartHandle == _driverHandle_UART[handleIndex]._huartHandle))
		{
			// Inserting bytes inside queue
			uint8_t txNum=0;
			while((txNum+1)<size && xQueueSendToBack(_driverHandle_UART[handleIndex]._txQueueHandle,&buff[txNum],0)==pdTRUE){
				txNum++;
			}
			// If no transmission ongoing and pipe is not empty, start transmission now
            // disable the IRQ
        	NVIC_DisableIRQ(_driverHandle_UART[handleIndex]._irq);

			if(huartHandle->gState == HAL_UART_STATE_READY){
				_driverHandle_UART[handleIndex]._txByte=buff[txNum];
				HAL_UART_Transmit_IT(_driverHandle_UART[handleIndex]._huartHandle, &_driverHandle_UART[handleIndex]._txByte, 1); //try restarting transmit if not ongoing
				txNum++;
			}else{
				if(xQueueSendToBack(_driverHandle_UART[handleIndex]._txQueueHandle,&buff[txNum],0)==pdTRUE){
					txNum++;
                }
			}

            NVIC_EnableIRQ(_driverHandle_UART[handleIndex]._irq);

			return txNum;
		}
	}

	return 0;
}


/* Flush the RX buffer of a given UART line */
void flushRXDriver_UART(UART_HandleTypeDef* huartHandle){
	// Scanning the structure array
	for(uint32_t handleIndex = 0; handleIndex < MAX_UART_HANDLE; handleIndex++)
	{
		// If it finds the handle in the structure
		if(_driverHandle_UART[handleIndex]._usageFlag == 1 && huartHandle == _driverHandle_UART[handleIndex]._huartHandle)
		{
			//flushing queue
			xQueueReset(_driverHandle_UART[handleIndex]._rxQueueHandle);
		}
	}
}

/* Flush the TX buffer of a given UART line */
void flushTXDriver_UART(UART_HandleTypeDef* huartHandle){
	// Scanning the structure array
	for(uint32_t handleIndex = 0; handleIndex < MAX_UART_HANDLE; handleIndex++)
	{
		// If it finds the handle in the structure
		if(_driverHandle_UART[handleIndex]._usageFlag == 1 && huartHandle == _driverHandle_UART[handleIndex]._huartHandle)
		{
			// Flushing queue
			xQueueReset(_driverHandle_UART[handleIndex]._txQueueHandle);
		}
	}
}

/* Interrupt handler for errors on the UART line */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huartHandle){

    //scanning the array with the structures to find the handle
    for(uint32_t handleIndex = 0; handleIndex < MAX_UART_HANDLE; handleIndex++)
    {
        //if it finds the handle
        if((_driverHandle_UART[handleIndex]._usageFlag == 1) && (huartHandle == _driverHandle_UART[handleIndex]._huartHandle))
        {
            HAL_UART_Receive_IT(huartHandle,&_driverHandle_UART[handleIndex]._rxByte,1);
            return;
        }
    }
    return;
    
}

/* Rx interrupt handler for UART line */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//scanning the structure array
    for(uint32_t handleIndex = 0; handleIndex < MAX_UART_HANDLE; handleIndex++)
    {
        //if it finds the handle in the structure
        if(_driverHandle_UART[handleIndex]._usageFlag == 1 && huart == _driverHandle_UART[handleIndex]._huartHandle)
        {
        	if(_driverHandle_UART[handleIndex]._policyRX==keep_new && xQueueIsQueueFullFromISR(_driverHandle_UART[handleIndex]._rxQueueHandle)){
        		uint8_t c;
        		xQueueReceiveFromISR(_driverHandle_UART[handleIndex]._rxQueueHandle, &c, NULL);
        	}

            xQueueSendToBackFromISR(_driverHandle_UART[handleIndex]._rxQueueHandle,(void*)&_driverHandle_UART[handleIndex]._rxByte,NULL);

            //relaunching ISR
            HAL_UART_Receive_IT(huart,&_driverHandle_UART[handleIndex]._rxByte,1);

            return;
        }
    }
	return;
}

/* Tx interrupt handler for UART line */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	//scanning the structure array
    for(uint32_t handleIndex = 0; handleIndex < MAX_UART_HANDLE; handleIndex++)
    {
        //if it finds the handle in the structure
        if(_driverHandle_UART[handleIndex]._usageFlag == 1 && huart == _driverHandle_UART[handleIndex]._huartHandle)
        {
            
			if(xQueueReceiveFromISR(_driverHandle_UART[handleIndex]._txQueueHandle,&_driverHandle_UART[handleIndex]._txByte,NULL)==pdTRUE){
				HAL_UART_Transmit_IT(_driverHandle_UART[handleIndex]._huartHandle, &_driverHandle_UART[handleIndex]._txByte, 1);
			}

            return;
        }
    }
	return;

}
