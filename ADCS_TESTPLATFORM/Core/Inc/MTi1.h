/* Function to manage a MTI-2 / MTI-3 imu
* protocol description: https://www.xsens.com/hubfs/Downloads/Manuals/MT_Low-Level_Documentation.pdf
*
* only the limited subset of needed functionalieties has been implemented
* */

#ifndef MTI1_H
#define MTI1_H

#include "frameUtils.h"
#include "bufferUtils.h"
#include <stdint.h>
#include "UARTdriver.h"
#include <stddef.h>

#define IMU_BUFFER_LEN 100	//local buffer length


/**
 * @brief Initializes the MTI-1 IMU by configuring it through a series of commands and acknowledgments. It delays so it must be called when HAL_GetTick interrupts are enabled
 * @param IMUhandle Pointer to the UART handle associated with the IMU communication
 * @return uint8_t Returns 1 if the initialization is successful 
 */
uint8_t initIMUConfig(UART_HandleTypeDef* IMUhandle);


/**
 * @brief Waits for and reads an IMU data packet from the specified UART handle, extracting gyroscope, magnetometer, and accelerometer data into provided buffers.
 * @param IMUhandle Pointer to the UART handle associated with the IMU communication
 * @param gyroscope Output buffer to store the extracted gyroscope data (3 elements)
 * @param magnetometer Output buffer to store the extracted magnetometer data (3 elements)
 * @param accelerometer Output buffer to store the extracted accelerometer data (3 elements)
 * @param timeout Maximum time to wait for the IMU packet (in milliseconds)
 * @return uint8_t Returns 1 if a valid IMU packet is received and data is successfully extracted, 0 otherwise
 */
uint8_t readIMUPacket(UART_HandleTypeDef* IMUhandle, float gyroscope[3], float magnetometer[3], float accelerometer[3], uint32_t timeout);

#endif
