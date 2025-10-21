#include "MTi3.h"

#ifdef DEBUG_MODE

#define DEBUG_PRINTF //comment this to disable debug printf even if DEBUG is defined

#endif

#define IMU_ACK_DELAY 100	//maximum time to wait for ack
#define IMU_CONFIG_RETRY 2  //number of times configuration commands will be sent if ack is not received

typedef struct{
	uint8_t mid;
	uint8_t len;
	uint8_t* data;
} imu_packet_struct;


//defines
#define IMU_PREAMBLE 	0xFA //preamble hex value
#define IMU_BID			0xFF//BID hex value
//MID definitions
#define IMU_GOTO_CONFIG_MID			0x30 //gotoconfig MID value
#define IMU_GOTO_CONFIG_ACK_MID		0x31 //gotoconfig ack MID value
#define IMU_SET_OCONFIG_MID 		0xC0 //set output config MID value
#define IMU_SET_OCONFIG_ACK_MID		0xC1 //set output config ack MID value
#define IMU_GOTO_MEAS_MID 			0x10 //gotomeas MID value
#define IMU_GOTO_MEAS_ACK_MID		0x11 //gotomeas ack MID value
#define IMU_DATA_PACKET_MID 		0x36 //data packet MID value
//LEN definitions (for messages with LEN!=0) 
#define IMU_SET_OCONFIG_LEN 		sizeof(outputConfigData)
#define IMU_SET_OCONFIG_ACK_LEN 	sizeof(outputConfigAckData)
#define IMU_DATA_PACKET_LEN			45
//data definitions
#define IMU_OUTPUT_CONFIG 		0x80, 0x20, 0x04, 0x80, /* Rate of turn */ \
								0xC0, 0x20, 0x04, 0x80, /* Magnetic Field */\
								0x40, 0x20, 0x04, 0x80 /*Accelerometer data*/

#define IMU_OUTPUT_CONFIG_ACK 	0x80, 0x20, 0x04, 0x80, /* Rate of turn */ \
								0xC0, 0x20, 0x04, 0x80, /* Magnetic Field */\
								0x40, 0x20, 0x04, 0x80 /*Accelerometer data*/
//others
#define IMU_DATA_GYRO_INDEX	3	//starting index (inside data) for gyroscope data
#define IMU_DATA_MAG_INDEX	18	//starting index (inside data) of magnetometer data
#define IMU_DATA_ACC_INDEX 33 //starting index (inside data) of accelerometer data

const uint8_t outputConfigData[]=		{IMU_OUTPUT_CONFIG};
const uint8_t outputConfigAckData[]=	{IMU_OUTPUT_CONFIG_ACK};