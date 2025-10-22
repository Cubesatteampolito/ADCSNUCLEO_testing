#ifndef MTI3_H
#define MTI3_H

#include "main.h"
#include <stdint.h>

typedef struct {
  float ax_ms2;
  float ay_ms2;
  float az_ms2;
  uint8_t valid;   // 1 when a fresh sample was parsed
} MTI3_Accel_t;

void MTI3_Init(UART_HandleTypeDef *huart);
void MTI3_Poll(void);                 // call in main loop
const MTI3_Accel_t* MTI3_GetAccel(void);

#endif
