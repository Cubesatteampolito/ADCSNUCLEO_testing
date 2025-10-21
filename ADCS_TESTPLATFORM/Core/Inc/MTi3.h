#ifndef MTI13_H
#define MTI13_H

/* Function to manage a MTI-2 / MTI-3 imu
 * protocol description: https://www.xsens.com/hubfs/Downloads/Manuals/MT_Low-Level_Documentation.pdf
 *
 * only the limited subset of needed functionalieties has been implemented
 * */

#include <stdint.h>

/* ------------------- WARNING -------------------- */


/* Function to parse a msg(PREAMBLE, PACKET) thru UART and compute checksum*/
static int read_xbus_full_message(uint8_t *out_buf, uint8_t out_buf_size, uint8_t expected_mid, uint32_t timeout_per_byte);

/* Send reduced xbus message: raw data without preamble/BID, but including checksum.
   msg points to [MID, LEN, DATA...], msg_len is count of those bytes (MID..lastData)
*/
static HAL_StatusTypeDef xbus_send_reduced(const uint8_t *msg, uint8_t msg_len, uint32_t timeout);

static void imu_wakeUp(void);


#endif
