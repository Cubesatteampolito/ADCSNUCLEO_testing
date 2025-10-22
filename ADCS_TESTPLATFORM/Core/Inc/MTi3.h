#ifndef MTI3_H
#define MTI3_H

#include "main.h"
#include <stdint.h>

#define MTI3_RX_BUF_SZ 512

// Latest acceleration [ax, ay, az] in m/s^2. Updated when a valid packet is parsed.
extern volatile float mti3_accel[3];

// Call once after HAL init
void MTI3_Init(UART_HandleTypeDef *huart);

// Call often (e.g., in the main loop) to parse new bytes from DMA buffer
void MTI3_Poll(void);

#endif