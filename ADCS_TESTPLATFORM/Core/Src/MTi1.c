#include "MTi1.h"

#ifdef DEBUG_MODE

#define DEBUG_PRINTF //comment this to disable debug printf even if DEBUG is defined

#endif

const uint8_t outputConfigData[]=		{IMU_OUTPUT_CONFIG};
const uint8_t outputConfigAckData[]=	{IMU_OUTPUT_CONFIG_ACK};

circular_buffer_handle rxcBuff; //rx and search buffer
uint8_t rxBuffer[IMU_BUFFER_LEN]; //memory buffer for rxBuff
uint8_t tmpBuff[IMU_BUFFER_LEN]; //temporary buffer where to store received packets

/* Function to compute message checksum: sum of all bytes including checksum must be 0 */
static uint8_t computeChecksum(imu_packet_struct * pckt){
	if (pckt == NULL) return 0;

	uint8_t crc = IMU_BID + pckt->mid + pckt->len;
	for (uint32_t d = 0; d < pckt->len; d++)
	{
		crc += pckt->data[d];
	}

	return -crc;
}

/* Function to send message from MCU to IMU */
static void sendMsg(UART_HandleTypeDef* IMUhandle, imu_packet_struct * pckt){
	if (pckt == NULL) return;

	uint8_t tmp = IMU_PREAMBLE;
	sendDriver_UART(IMUhandle, &tmp, 1);
	tmp=IMU_BID;
	sendDriver_UART(IMUhandle, &tmp, 1);
	sendDriver_UART(IMUhandle, &pckt->mid, 1);
	sendDriver_UART(IMUhandle, &pckt->len, 1);
	sendDriver_UART(IMUhandle, pckt->data, pckt->len);
	tmp=computeChecksum(pckt);
	sendDriver_UART(IMUhandle, &tmp, 1);
}

/* Function that reads bytes from UART -> finds a valide IMU packet -> optionally checks CRC -> returns it. Always flush buffers before calling this function. */
static uint8_t receiveMsg(UART_HandleTypeDef* IMUhandle, imu_packet_struct * pckt, imu_packet_struct* format, uint8_t checkCRC, uint32_t timeout)
{
	/* Initialize counter when the function is called */
	uint32_t startTick = HAL_GetTick();

	uint8_t len = 0; //temporary variable to store target number of bytes to search
	uint8_t mid = 0; //temporary variable to store message id

	/* State machine phases: _header to find start of packet and _packet to read full packet */
	typedef enum{
		_header,
		_packet
	} search_phase;

	search_phase phase = _header;

	uint8_t headTail[4] = {IMU_PREAMBLE, IMU_BID, 0, 0};

	search_frame_rule rule;

	rule.head = (uint8_t *) headTail;
	rule.tail = NULL;
	rule.tailLen = 0;
	rule.maxLen = 0;
	rule.policy = soft;

	circular_buffer_handle foundPckt;
	imu_packet_struct tmpPckt;
	tmpPckt.data = tmpBuff;

	/* Look for valid packet until timeout or packet found. This is done at least once because of the do{} while(). */
	do
	{
		/* Fill circular buffer until it is full */
		while(!cBuffFull(&rxcBuff))
		{ 
			/* Read bytes received over UART into the circular buffer */
			uint8_t c;
			if (receiveDriver_UART(IMUhandle, &c, 1))
			{
				cBuffPush(&rxcBuff, &c, 1,1);
			}
			else
			{
				break;
			}
		}

		/* STATE MACHINE: HEADER PHASE */
		if (phase == _header)
		{ 
			/* If the format is specified skip header detection and jump to PACKET PHASE */
			if (format != NULL)
			{	
				mid = format -> mid;
				len = format -> len;
				phase = _packet;
				continue;	// jump to PACKET PHASE
			}

			/* If no format is specified, look for a header [0XFA 0XFF] */
			rule.headLen = 2;
			rule.minLen = 2;
			if (searchFrameAdvance(&rxcBuff, &foundPckt, &rule, SHIFTOUT_FULL | SHIFTOUT_CURR | SHIFTOUT_FAST))
			{	
				mid = foundPckt.buff[2];	// extract message ID (MID)
				len = foundPckt.buff[3];	// extract payload size (LEN)
				phase = _packet;
			}
		}
		else if (phase == _packet)
		{
			headTail[2] = mid;
			headTail[3] = len;
			rule.headLen = 4;
			rule.minLen = len + 1;	// define expected size (len + 1) to house CRC

			/* Search full packet in the buffer */
			if (searchFrameAdvance(&rxcBuff, &foundPckt, &rule, SHIFTOUT_FULL | SHIFTOUT_NEXT | SHIFTOUT_FAST))
			{
				#if (DEBUG_MSGS == 1)
				printf("RAW IMU FRAME:\n");
				#endif
				//cBuffPrint(&foundPckt,PRINTBUFF_HEX | PRINTBUFF_NOEMPTY);

				/* Extract the data, skipping the first 4 bytes */
				tmpPckt.mid = mid;
				tmpPckt.len = len;
				if (len != 0) cBuffRead(&foundPckt, tmpPckt.data, foundPckt.elemNum, 0, 4);

				/* Return packet to caller if you wish */
				if (pckt != NULL)
				{ 
					pckt -> mid = mid;
					pckt -> len = len;
					if (len != 0) pckt->data = tmpPckt.data;
				}

				/* Compare the checksums of the incoming packet and compare it to the checksum from the IMU */
				if(checkCRC)
				{
					if (cBuffReadByte(&foundPckt, 1, 0) == computeChecksum(&tmpPckt))	
					{
						return 1;			// checksum valid, return 1
					}
					else
					{	
						phase = _header;	// if checksums don't match, keep searching
					}		
				}
				else	
				{
					return 1;				// always return 1 if checksum is not active
				}

				#if (DEBUG_MSGS == 1)
					printf("Checksum verification failed!\n");
				#endif

			}
		}
		else
		{
			phase = _header;	// in case of any state errors, return to default state
		}

	} while ((HAL_GetTick() - startTick) < timeout); // keep looping until timeout expires
3
	return 0;
}

/* Function to send command MCU -> IMU and wait for IMU acknowledgement after MCU to IMU communication */
static uint8_t imuAckTransaction(UART_HandleTypeDef* IMUhandle, imu_packet_struct * cmd, imu_packet_struct * ack, uint32_t timeout)
{
	if (cmd == NULL || ack == NULL) return 0;

	/* Flush the circular buffer */
	cBuffFlush(&rxcBuff);

	sendMsg(IMUhandle, cmd);

	/* The ACK must be the next transaction to be received */
    if (receiveMsg(IMUhandle, NULL, ack, 1, timeout))
	{
    	return 1;	// ACK received
    }
	else
	{
    	return 0;	// NACK received
    }
}

/* Function to configure the Xsens Mti IMU before you start reading data. IMUhandle is the UART handle associated with IMU communication */
uint8_t initIMUConfig(UART_HandleTypeDef* IMUhandle)
{
	
	/* Initialize buffer */
	cBuffInit(&rxcBuff, rxBuffer, sizeof(rxBuffer), 0);
    HAL_Delay(500);
    flushRXDriver_UART(IMUhandle);

    imu_packet_struct cmd;
    imu_packet_struct ack;

    cmd.len = 0;
    ack.len = 0;

    /* Go to CONFIG mode */
    cmd.mid = IMU_GOTO_CONFIG_MID;
    ack.mid = IMU_GOTO_CONFIG_ACK_MID;

	/* Loop until ACK received or retries exhausted */
    for(uint32_t retry=0;retry<IMU_CONFIG_RETRY;retry++)
	{
    	if (imuAckTransaction(IMUhandle, &cmd, &ack, IMU_ACK_DELAY)) break;
    	else if (retry == (IMU_CONFIG_RETRY - 1)) return 0;
    }

    /* Set output configuration */
    cmd.mid = IMU_SET_OCONFIG_MID;
    cmd.len = IMU_SET_OCONFIG_LEN;
    cmd.data = (uint8_t *) outputConfigData;
	ack.mid = IMU_SET_OCONFIG_ACK_MID;
	ack.len = IMU_SET_OCONFIG_ACK_LEN;
	for (uint32_t retry = 0; retry < IMU_CONFIG_RETRY; retry++)
	{
		if (imuAckTransaction(IMUhandle, &cmd, &ack, IMU_ACK_DELAY)) break;
		else if (retry == (IMU_CONFIG_RETRY - 1)) return 0;
	}

    /* Go to MEASUREMENT mode */
    cmd.mid = IMU_GOTO_MEAS_MID;
    cmd.len = 0;
	ack.mid = IMU_GOTO_MEAS_ACK_MID;
	ack.len = 0;
	for (uint32_t retry = 0; retry < IMU_CONFIG_RETRY; retry++)
	{
		if (imuAckTransaction(IMUhandle, &cmd, &ack, IMU_ACK_DELAY)) break;
		else if (retry == (IMU_CONFIG_RETRY - 1)) return 0;
	}

#ifdef IMU_GROUND_CALIBRATION
	estimate_stationary_gyro_bias();
#endif

	return 1;
}

/* Function to combine 4 bytes into a big-endian 32-bit integer [byte0][byte1][byte2][byte3] -> uint32_t */
uint32_t buff2Int32(uint8_t buff[4])
{
	uint32_t retval = 0;
	for (int b = 0; b < 4; b++)
	{
		retval = retval << 8;
		retval |= buff[b];
	}
	return retval;
}

/* Convert 4 uint8_t frames into uint32_t and store in data array for all dataSize data fields */
void writeIMUDataArray(uint8_t* frame, uint32_t* data, uint32_t dataSize)
{
	for (uint32_t d = 0; d < dataSize; d++)
	{
		uint32_t raw = 0;
		for (uint32_t byte = 0; byte < 4; byte++)
		{
			raw |= ((uint32_t) frame[d * 4 + byte]) << (8 * (3 - byte));
		}
		data[d] = raw;
	}
	return;
}

uint8_t readIMUPacket(UART_HandleTypeDef* IMUhandle, float gyroscope[3], float magnetometer[3], float accelerometer[3], uint32_t timeout)
{
	/* Flush the local buffer */
	cBuffFlush(&rxcBuff); 

	/* Define expected packet: only accept measurement packets. All other fields in the struct are initialized to zero -> safe approach */
	imu_packet_struct format = 
	{
		.mid = IMU_DATA_PACKET_MID,
		.len = IMU_DATA_PACKET_LEN,
	};

	imu_packet_struct meas;

	/* Receive packet with valid checksum */
	if (receiveMsg(IMUhandle, &meas, &format, 1, timeout))
	{
		/* Extract accelerometer, gyroscope and magnetometer data from IMU */
		writeIMUDataArray(&meas.data[IMU_DATA_ACC_INDEX], (uint32_t*) accelerometer, 3);
		writeIMUDataArray(&meas.data[IMU_DATA_GYRO_INDEX], (uint32_t*) gyroscope, 3);
		writeIMUDataArray(&meas.data[IMU_DATA_MAG_INDEX], (uint32_t*) magnetometer, 3);

		return 1;
	}

	return 0;
}

#ifdef IMU_GROUND_CALIBRATION
/* Function used during initialization to estimate and remove gyroscope bias on the ground. ONLY USE WHEN SYSTEM STATIONARY ON THE GROUND. */
static void static void estimate_stationary_gyro_bias()
{
	/* Let measurements stabilize */
	HAL_Delay(500);

	/* Average bias over N samples */
	const uint32_t N = 1000;

	/* Make sure bias is initialized to zero */
	gyro_bias[0] = 0.0f;
	gyro_bias[1] = 0.0f;
	gyro_bias[2] = 0.0f;

	/* Array to store IMU data (3 bytes per sensor) */
	float gyro_data[3];
	float mag_data[3];
	float acc_data[3];

	/* Read N data samples and accumulate gyroscope bias */
	for (int i = 0; i < N; i++)
	{
		if (readIMUPacket(IMUhandle, gyro_data, mag_data, acc_data, 100))
		{
			gyro_bias[0] += gyro_data[0];
			gyro_bias[1] += gyro_data[1];
			gyro_bias[3] += gyro_data[3];
		}
		else
		{
			i--;	// retry if packet failed
		}
	}

	/* Compute the average of the biases over N samples */
	gyro_bias[0] =/ N;
	gyro_bias[1] =/ N;
	gyro_bias[2] =/ N;

	/* Print the bias so you can store it */
	printf("Gyro bias: %f \t %f \t %f\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
}
#endif