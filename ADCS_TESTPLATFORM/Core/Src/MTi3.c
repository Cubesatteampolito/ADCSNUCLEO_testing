#include "mti3.h"
#include <string.h>

#define XBUS_PREAMBLE 0xFA
#define XBUS_BID      0xFF
#define MID_MTDATA2   0x36
#define DID_ACCEL     0x4030u  // Acceleration (m/s^2), 3x float32 big-endian
#define PAY_MAX       256

static UART_HandleTypeDef *s_huart;
static uint8_t rx;
static volatile uint8_t rx_ready = 0;

static uint8_t pkt[4 + PAY_MAX + 1]; // preamble + BID + MID + LEN + payload + checksum
static uint16_t pkt_len = 0;
static uint16_t pkt_idx = 0;
static enum { S_WAIT_PREAMBLE, S_BID, S_MID, S_LEN, S_PAYLOAD, S_CHECK } st = S_WAIT_PREAMBLE;

static MTI3_Accel_t out = {0};

static uint8_t xbus_checksum(uint8_t *buf, uint16_t n) {
  // Sum of BID..payload, checksum makes sum % 256 == 0
  uint32_t sum = 0;
  for (uint16_t i = 1; i < n-1; ++i) sum += buf[i];
  return (uint8_t)(-((int32_t)sum));
}

static float be_f32(const uint8_t *p) {
  uint32_t u = ((uint32_t)p[0]<<24) | ((uint32_t)p[1]<<16) | ((uint32_t)p[2]<<8) | p[3];
  float f;
  memcpy(&f, &u, sizeof(f));
  return f;
}

static void parse_mtdata2_fields(const uint8_t *payload, uint16_t len) {
  uint16_t i = 0;
  while (i + 4 <= len) {
    uint16_t did = ((uint16_t)payload[i]<<8) | payload[i+1];  // big-endian
    uint16_t flen = ((uint16_t)payload[i+2]<<8) | payload[i+3];
    i += 4;
    if (i + flen > len) break;

    if (did == DID_ACCEL && flen == 12) {
      out.ax_ms2 = be_f32(&payload[i+0]);
      out.ay_ms2 = be_f32(&payload[i+4]);
      out.az_ms2 = be_f32(&payload[i+8]);
      out.valid = 1;
    }
    i += flen;
  }
}

static void feed(uint8_t b) {
  switch (st) {
    case S_WAIT_PREAMBLE:
      if (b == XBUS_PREAMBLE) {
        pkt_idx = 0;
        pkt[pkt_idx++] = b;
        st = S_BID;
      }
      break;
    case S_BID:
      pkt[pkt_idx++] = b;
      st = (b == XBUS_BID) ? S_MID : S_WAIT_PREAMBLE;
      break;
    case S_MID:
      pkt[pkt_idx++] = b;
      st = S_LEN;
      break;
    case S_LEN:
      pkt[pkt_idx++] = b;
      pkt_len = b;                // 1-byte LEN (sufficient for our minimal case)
      if (pkt_len > PAY_MAX) { st = S_WAIT_PREAMBLE; break; }
      st = (pkt_len == 0) ? S_CHECK : S_PAYLOAD;
      break;
    case S_PAYLOAD:
      pkt[pkt_idx++] = b;
      if (pkt_idx == 4 + pkt_len) st = S_CHECK;
      break;
    case S_CHECK:
      pkt[pkt_idx++] = b;         // checksum
      if (xbus_checksum(pkt, pkt_idx)) { /* bad */ }
      else if (pkt[2] == MID_MTDATA2) {
        parse_mtdata2_fields(&pkt[4], pkt_len);
      }
      st = S_WAIT_PREAMBLE;
      break;
  }
}

void MTI3_Init(UART_HandleTypeDef *huart) {
  s_huart = huart;
  st = S_WAIT_PREAMBLE;
  pkt_idx = 0;
  out.valid = 0;
  HAL_UART_Receive_IT(s_huart, &rx, 1);
}

void MTI3_Poll(void) {
  // no-op; parser runs in IRQ callback
  if (rx_ready) { rx_ready = 0; HAL_UART_Receive_IT(s_huart, &rx, 1); }
}

const MTI3_Accel_t* MTI3_GetAccel(void) { return &out; }

// HAL weak override: call from IRQ when a byte arrives
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == s_huart) {
    feed(rx);
    rx_ready = 1;
  }
}