/* Functions used to manage a MTI-2 / MTI-3 IMU
*  Protocol description: https://www.xsens.com/hubfs/Downloads/Manuals/MT_Low-Level_Documentation.pdf
*  Only the limited subset of needed functionalieties has been implemented
*  IMU <-> MCU communication happens over UART. 
*  Data is exchanged using a packed-based protocol (Xbus). Packet structure: [Preamble][BID][MID][LEN][DATA][CHECKSUM]
*  The code uses blocking polling for UART reception, no interrupt-based communication nor Direct Memory Access.
*/

#ifndef MTI1_H
#define MTI1_H

#include <stdint.h>
#include <stddef.h>

#include "frameUtils.h"
#include "bufferUtils.h"
#include "UARTdriver.h"

#define IMU_BUFFER_LEN 100	//local buffer length

#define IMU_PREAMBLE 	0xfa    // the preamble is always equal to 0xFA (start of message)
#define IMU_BID			0xff    // bus identifier

#define IMU_ACK_DELAY 100	//maximum time to wait for ack
#define IMU_CONFIG_RETRY 2 //number of times configuration commands will be sent if ack is not received

//MID definitions
#define IMU_GOTO_CONFIG_MID			0x30
#define IMU_GOTO_CONFIG_ACK_MID		0x31
#define IMU_SET_OCONFIG_MID 		0xC0
#define IMU_SET_OCONFIG_ACK_MID		0xC1
#define IMU_GOTO_MEAS_MID 			0x10
#define IMU_GOTO_MEAS_ACK_MID		0x11
#define IMU_DATA_PACKET_MID 		0x36
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
#define IMU_DATA_MAG_INDEX  18	//starting index (inside data) of magnetometer data
#define IMU_DATA_ACC_INDEX  33 //starting index (inside data) of accelerometer data

/* Struct used to transmit IMU data (excluding [Preamble][BID] and [CHECKSUM]) */
typedef struct
{
	uint8_t  mid;
	uint8_t  len;
	uint8_t *data;
} imu_packet_struct;

/* Function to compute checksum of IMU packet */
static uint8_t computeChecksum(imu_packet_struct * pckt);

/* Function to transmit data from MCU to IMU byte-by-byte over UART */
static void sendMsg(UART_HandleTypeDef* IMUhandle, imu_packet_struct * pckt);

/* Function to continuously read UART transmission from IMU to MCU */
static uint8_t receiveMsg(UART_HandleTypeDef* IMUhandle, imu_packet_struct * pckt, imu_packet_struct* format, uint8_t checkCRC, uint32_t timeout);

/* Function to send command MCU -> IMU and wait for IMU acknowledgement after MCU to IMU communication */
static uint8_t imuAckTransaction(UART_HandleTypeDef* IMUhandle, imu_packet_struct * cmd, imu_packet_struct * ack, uint32_t timeout);

/* Function to configure the Xsens Mti IMU before you start reading data.
*  It must be called ONLY after HAL_Init() and SystemClock_Config(), or delays and interrupts will break if the system timer interrupts are not running. */
uint8_t initIMUConfig(UART_HandleTypeDef* IMUhandle)

/* Function to combine 4 bytes into a big-endian 32-bit integer [byte0][byte1][byte2][byte3] -> uint32_t */
uint32_t buff2Int32(uint8_t buff[4])

/* Function to convert 4 big-endian [MSB][...][LSB] uint8_t frames into uint32_t little-endian [LSB][...][MSB] and store in data array for all dataSize data fields */
void writeIMUDataArray(uint8_t* frame, uint32_t* data, uint32_t dataSize)

/* Function to extract sensor data from IMU communications */
uint8_t readIMUPacket(UART_HandleTypeDef* IMUhandle, float gyroscope[3], float magnetometer[3], float accelerometer[3] ,uint32_t timeout);

