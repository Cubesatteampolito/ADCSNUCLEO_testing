#include "stm32f4xx_hal.h" // prlly not needed if in the main alr
#include "MTI3.h"
extern UART_HandleTypeDef huart1; // created by CubeMX
#define IMU_PREAMBLE 	0xFA
#define IMU_BID			0xFF

#define IMU_WAKEUP      0x3E
#define IMU_WAKEUP_ACK  0x3F

/* ------------------- WARNINGS -------------------- 
    1) add task into rtos
*/

// Compute checksum for reduced message: CS = -(BID + MID + LEN + DATA) % 256 = 0
static uint8_t xbus_compute_checksum_reduced(const uint8_t *msg, uint8_t len /*len includes MID, LEN, DATA...*/) {
    uint32_t s = 0;
    s += IMU_BID; 
    for (uint8_t i = 0; i < len; ++i) s += msg[i];
    return (uint8_t)(-(s & 0xFF)); // s & 0xFF <=> s % 256
}

/* Send reduced xbus message: raw data without preamble/BID, but including checksum.
   msg points to [MID, LEN, DATA...], msg_len is count of those bytes (MID..lastData)
*/
static HAL_StatusTypeDef xbus_send_reduced(const uint8_t *msg, uint8_t msg_len, uint32_t timeout) {
    uint8_t buf[64];
    if (msg_len + 1 > sizeof(buf)) return HAL_ERROR;
    memcpy(buf, msg, msg_len);
    buf[msg_len] = xbus_compute_checksum_reduced(msg, msg_len); // checksum
    return HAL_UART_Transmit(&huart1, buf, msg_len + 1, timeout); /* (handletypedef, data buf, length, timeout) returns HAL_status*/
}

/* Read one byte with timeout */
static HAL_StatusTypeDef uart_read_byte(uint8_t *b, uint32_t timeout) {
    return HAL_UART_Receive(&huart1, b, 1, timeout); /* (handletypedef, data buf, length, timeout) returns HAL_status*/
}

/* Try to read a full Xbus full message (preamble included). This is a simple parser:
   - waits for 0xFA then reads BID, MID, LEN, LEN bytes, then CS
   - returns true if checksum ok and MID matches expectedMID (or -1 to accept any)
*/
static int read_xbus_full_message(uint8_t *out_buf, uint8_t out_buf_size, uint8_t expected_mid, uint32_t timeout_per_byte) {
    uint8_t b;
    uint32_t elapsed = 0;

    /* ------------ HEADER --------------*/

    // wait for preamble 0xFA
    for (;;) {
        if (uart_read_byte(&b, timeout_per_byte) != HAL_OK) return -1; // returns -1 if times out
        if (b == IMU_PREAMBLE) break; // exit loop if finds matching header
    }
    // read BID, MID, LEN
    uint8_t BID, MID, LEN;
    if (uart_read_byte(&BID, timeout_per_byte) != HAL_OK) return -1;
    if (uart_read_byte(&MID, timeout_per_byte) != HAL_OK) return -1; /* returns -1 if times out */
    if (uart_read_byte(&LEN, timeout_per_byte) != HAL_OK) return -1;
    if (expected_mid != 0xFF && expected_mid != MID) {
        // still read packet+CS and discard
    }
    if (LEN + 1 > out_buf_size) return -1; // +1 for checksum
    
    /* ------------ PACKET --------------*/

    // read packet LEN bytes
    for (uint8_t i = 0; i < LEN; ++i) {
        if (uart_read_byte(&out_buf[i], timeout_per_byte) != HAL_OK) return -1;
    }
    // read checksum
    uint8_t cs;
    if (uart_read_byte(&cs, timeout_per_byte) != HAL_OK) return -1;
    // verify checksum: sum BID+MID+LEN+DATA+cs == 0 (mod 256)
    uint32_t s = BID + MID + LEN;
    for (uint8_t i = 0; i < LEN; ++i) s += out_buf[i]; // + DATA
    s += cs;
    if ((s & 0xFF) != 0) 
        return -2; // checksum fail
    // otherwise success -> return MID (so caller can check)
    return (int)MID;
}

/* requestDeviceID function */
void imu_test_req_did(void) {
    // ReqDID reduced message: MID=0x00, LEN=0x00
    uint8_t req[] = { 0x00, 0x00 }; // MID, LEN
    if (xbus_send_reduced(req, sizeof(req), 100) != HAL_OK) {
        // transmit failed
        return;
    }
    // The module normally replies with a full message: preamble(0xFA) BID(0xFF) MID(0x01) LEN(0x04) DeviceID(4) CS
    uint8_t packet[32];
    int mid = read_xbus_full_message(packet, sizeof(packet), 0x01, 200); // expect MID=0x01 <=> REQUEST DEVICE ID
    if (mid == 0x01) {
        // packet[0..3] contain DeviceID (HH HL LH LL)
        uint32_t deviceId = (packet[0] << 24) | (packet[1] << 16) | (packet[2] << 8) | packet[3];
        printf("DeviceID: 0x%08X\n", deviceId);
    } else if (mid == -2) {
        printf("Checksum Failed");
    } else {
        printf("Timeout or other error");
    }
}

/* wakeUp + wakeUpAck function */
void imu_wakeUp(void) {
    /* ---------- READING ------------ */
    uint8_t packet[32];
    int mid = read_xbus_full_message(packet, sizeof(packet), IMU_WAKEUP, 100); // expect MID=0x3E <=> WAKEUP
    if (mid == IMU_WAKEUP) {
        /* ---------- WRITING ------------ */
        uint8_t req[] = { IMU_WAKEUP_ACK, 0x00 }; // MID, LEN, sending wakeUpAck MID = 0X3F
        if (xbus_send_reduced(req, sizeof(req), 100) != HAL_OK) return; // transmit failed
    } else if (mid == -2) {
        print("Checksum Failed");
    } else {
        print("Timeout or other error");
    }
}