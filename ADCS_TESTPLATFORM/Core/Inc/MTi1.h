/* Functions used to manage a MTI-2 / MTI-3 IMU
*  Protocol description: https://www.xsens.com/hubfs/Downloads/Manuals/MT_Low-Level_Documentation.pdf
*  Only the limited subset of needed functionalieties has been implemented (not all message IDs are defined)
*  IMU <-> MCU communication happens over UART. 
*
*  Data is exchanged using a packed-based protocol (Xbus). Packet structure: [Preamble][BID][MID][LEN][DATA][CHECKSUM]
*  [Preamble]	1 byte	0xFA (start of packet)
*  [BID]		1 byte	0xFF (bus identifier/address)
*  [LEN]		1 byte	# bytes in DATA field (for standard length message maximum value is 0xFE = 254) 
*  [DATA]		0 - 254 bytes	
*  [CHECKSUM]	1 byte	(checksum of message)
*
*  The code uses blocking polling for UART reception, no interrupt-based communication nor Direct Memory Access.
*/

#ifndef MTI1_H
#define MTI1_H

#include <stdint.h>
#include <stddef.h>
#include "frameUtils.h"
#include "bufferUtils.h"
#include "UARTdriver.h"

#define IMU_BUFFER_LEN 	100		// local buffer length

#define IMU_PREAMBLE 	0xFA    // the preamble is always equal to 0xFA (start of message)
#define IMU_BID			0xFF    // bus identifier

#define IMU_ACK_DELAY 	100	//maximum time to wait for ack
#define IMU_CONFIG_RETRY 2 //number of times configuration commands will be sent if ack is not received

/* Definition of Message IDs (MID) - find them in chapter 5.3 of the Communication Protocol PDF */
#define IMU_GOTO_CONFIG_MID			0x30	// GoToConfig: 			switch device state from measurement to configuration (MCU -> IMU)
#define IMU_GOTO_CONFIG_ACK_MID		0x31	// GoToConfigAck:	(IMU -> MCU)
#define IMU_GOTO_MEAS_MID 			0x10	// GoToMeasurement: 	switch device state from configuration to measurement (MCU -> IMU)
#define IMU_GOTO_MEAS_ACK_MID		0x11	// GoToMeasurementAck:	(IMU -> MCU)
#define IMU_SET_OCONFIG_MID 		0xC0	// SetOutputConfiguration: request the output configuration of the device
#define IMU_SET_OCONFIG_ACK_MID		0xC1	// SetOutputConfigurationAck
#define IMU_DATA_PACKET_MID 		0x36	// MTData2: MID that specifies that packet contains measurement data (IMU -> MCU)

/* Definitions of DATA lengths (when DATA field is not null) */
#define IMU_SET_OCONFIG_LEN 		sizeof(outputConfigData)		// specifies how many byte are in the data payload
#define IMU_SET_OCONFIG_ACK_LEN 	sizeof(outputConfigAckData)
#define IMU_DATA_PACKET_LEN			45								// total packet length (12 bytes/sensor * 3 sensors) = 36 measurement bytes) + overhead

/* Data definitions MCU -> IMU (don't be tempted to collect the Quaternion directly, as you need to combine IMU data with other sensors to get the right attitude quaternion) */
 
#define IMU_OUTPUT_CONFIG 		0x80, 0x20, 0x04, 0x80, 	\ 	// Gyroscope: 0x8020,
								0xC0, 0x20, 0x04, 0x80, 	\ 	// Magnetometer: 0xC020,
								0x40, 0x20, 0x04, 0x80 			// Accelerometer: 0x4020,

/* Data definitions IMU -> MCU */
#define IMU_OUTPUT_CONFIG_ACK 	0x80, 0x20, 0x04, 0x80, /* Rate of turn */ \
								0xC0, 0x20, 0x04, 0x80, /* Magnetic Field */\
								0x40, 0x20, 0x04, 0x80 /*Accelerometer data*/

/* Offsets for sensor data within the DATA packet */
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

