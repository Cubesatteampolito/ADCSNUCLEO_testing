#include "mti3.h"
#include <string.h>

#define XBUS_PRE   0xFA
#define XBUS_BID   0xFF
#define MID_GOTOCONFIG     0x30
#define MID_GOTOMEAS       0x10
#define MID_REQ_DID        0x00
#define MID_DEVICEID       0x01
#define MID_MTDATA2        0x36
#define DID_ACCEL          0x4030u

#define PAY_MAX 256

static UART_HandleTypeDef *s_huart;
static uint8_t rx;
static volatile uint8_t rx_arm = 0;

static uint8_t pkt[4 + PAY_MAX + 1];
static uint16_t pkt_idx = 0;
static uint16_t pay_len = 0;

static enum { P_PRE, P_BID, P_MID, P_LEN, P_PAY, P_CHK } st;

static MTI3_Accel_t acc = {0};
static MTI3_DeviceID_t did = {0};

static uint8_t checksum(uint8_t *buf, uint16_t n) {
  uint32_t sum = 0;
  for (uint16_t i = 1; i < n-1; ++i) sum += buf[i];
  return (uint8_t)(-((int32_t)sum));
}
static float be_f32(const uint8_t *p){
  uint32_t u = (uint32_t)p[0]<<24 | (uint32_t)p[1]<<16 | (uint32_t)p[2]<<8 | p[3];
  float f; memcpy(&f,&u,4); return f;
}
static uint32_t be_u32(const uint8_t *p){
  return (uint32_t)p[0]<<24 | (uint32_t)p[1]<<16 | (uint32_t)p[2]<<8 | p[3];
}

static void parse_mtdata2(const uint8_t *pay, uint16_t len){
  uint16_t i=0;
  while (i+4 <= len){
    uint16_t did_be = (uint16_t)pay[i]<<8 | pay[i+1];
    uint16_t flen   = (uint16_t)pay[i+2]<<8 | pay[i+3];
    i += 4;
    if (i+flen > len) break;
    if (did_be == DID_ACCEL && flen == 12){
      acc.ax_ms2 = be_f32(&pay[i+0]);
      acc.ay_ms2 = be_f32(&pay[i+4]);
      acc.az_ms2 = be_f32(&pay[i+8]);
      acc.valid  = 1;
    }
    i += flen;
  }
}

static void feed(uint8_t b){
  switch(st){
    case P_PRE:
      if (b == XBUS_PRE){ pkt_idx = 0; pkt[pkt_idx++] = b; st = P_BID; }
      break;
    case P_BID:
      pkt[pkt_idx++] = b;
      st = (b == XBUS_BID) ? P_MID : P_PRE;
      break;
    case P_MID:
      pkt[pkt_idx++] = b;
      st = P_LEN;
      break;
    case P_LEN:
      pkt[pkt_idx++] = b;
      pay_len = b;
      if (pay_len > PAY_MAX){ st = P_PRE; break; }
      st = pay_len ? P_PAY : P_CHK;
      break;
    case P_PAY:
      pkt[pkt_idx++] = b;
      if (pkt_idx == 4 + pay_len) st = P_CHK;
      break;
    case P_CHK:
      pkt[pkt_idx++] = b;
      if (checksum(pkt, pkt_idx) == 0){
        uint8_t mid = pkt[2];
        const uint8_t *pay = &pkt[4];
        if (mid == MID_DEVICEID && pay_len == 4){
          did.device_id = be_u32(pay);
          did.valid = 1;
        } else if (mid == MID_MTDATA2){
          parse_mtdata2(pay, pay_len);
        }
      }
      st = P_PRE;
      break;
  }
}

void MTI3_Init(UART_HandleTypeDef *huart){
  s_huart = huart;
  st = P_PRE; pkt_idx = 0; pay_len = 0;
  acc.valid = 0; did.valid = 0;
  rx_arm = 1;
  HAL_UART_Receive_IT(s_huart, &rx, 1);
}

void MTI3_Poll(void){
  if (rx_arm){ rx_arm = 0; HAL_UART_Receive_IT(s_huart, &rx, 1); }
}

const MTI3_Accel_t* MTI3_GetAccel(void){ return &acc; }
const MTI3_DeviceID_t* MTI3_GetDeviceID(void){ return &did; }

/* Simple command helpers */
static void send_frame(uint8_t mid, const uint8_t *data, uint8_t len){
  uint8_t buf[4 + PAY_MAX + 1];
  uint16_t n = 0;
  buf[n++] = XBUS_PRE;
  buf[n++] = XBUS_BID;
  buf[n++] = mid;
  buf[n++] = len;
  if (len && data) { memcpy(&buf[n], data, len); n += len; }
  buf[n++] = checksum(buf, n+1);
  HAL_UART_Transmit(s_huart, buf, n, 100);
}

void MTI3_SendGoToConfig(void){ send_frame(MID_GOTOCONFIG, NULL, 0); }
void MTI3_SendGoToMeasurement(void){ send_frame(MID_GOTOMEAS, NULL, 0); }
void MTI3_ReqDeviceID(void){ send_frame(MID_REQ_DID, NULL, 0); }

/* HAL weak callback override */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if (huart == s_huart){
    feed(rx);
    rx_arm = 1;
  }
}
