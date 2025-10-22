#ifndef MTI3_H
#define MTI3_H

#include "main.h"
#include <stdint.h>

typedef struct {
  float ax_ms2, ay_ms2, az_ms2;
  uint8_t valid;
} MTI3_Accel_t;

typedef struct {
  uint32_t device_id;  // big-endian parsed to native
  uint8_t valid;
} MTI3_DeviceID_t;

void MTI3_Init(UART_HandleTypeDef *huart);
void MTI3_Poll(void);                      // call in loop or a task
void MTI3_SendGoToConfig(void);
void MTI3_SendGoToMeasurement(void);
void MTI3_ReqDeviceID(void);

const MTI3_Accel_t*   MTI3_GetAccel(void);
const MTI3_DeviceID_t* MTI3_GetDeviceID(void);

#endif