#include "MTi3.h"
#include <string.h>

static UART_HandleTypeDef *mhuart;
static uint8_t rx_dma_buf[MTI3_RX_BUF_SZ];
static volatile uint16_t dma_last = 0;

volatile float mti3_accel[3] = {0.f, 0.f, 0.f};

// --- helpers ---
static uint16_t dma_pos(void) {
    // Number of bytes already received by DMA in circular mode
    return (uint16_t)(MTI3_RX_BUF_SZ - __HAL_DMA_GET_COUNTER(mhuart->hdmarx));
}
static uint8_t checksum_xbus(const uint8_t *p, uint16_t len) {
    uint32_t s = 0;
    for (uint16_t i = 0; i < len; i++) s += p[i];
    return (uint8_t)(-((int8_t)(s & 0xFF)));
}
static float be_f32(const uint8_t *b) {
    uint32_t u = ((uint32_t)b[0]<<24)|((uint32_t)b[1]<<16)|((uint32_t)b[2]<<8)|b[3];
    float f;
    memcpy(&f, &u, 4);
    return f;
}

// Parse one Xbus packet if available. Returns bytes consumed.
static uint16_t parse_one(const uint8_t *buf, uint16_t avail) {
    // Xbus: 0xFA 0xFF MID LEN [LEN bytes] CHK
    if (avail < 5) return 0;
    // search preamble 0xFA 0xFF
    uint16_t i = 0;
    while (i + 5 <= avail) {
        if (buf[i] == 0xFA && buf[i+1] == 0xFF) break;
        i++;
    }
    if (i + 5 > avail) return i; // skip garbage

    uint16_t pos = i;
    if (pos + 5 > avail) return pos; // not enough

    uint8_t MID  = buf[pos+2];
    uint8_t LEN  = buf[pos+3];
    uint16_t need = 5 + LEN; // includes CHK
    if (pos + need > avail) return pos; // wait for more

    // Verify checksum on [0..3+LEN], CHK at [4+LEN]
    if (checksum_xbus(&buf[pos], 4 + LEN) != buf[pos + 4 + LEN]) {
        // bad frame, drop preamble
        return pos + 1;
    }

    // MID 0x36 = MTData2
    if (MID == 0x36 && LEN >= 2) {
        // MTData2 payload: repeating blocks: [XDI (2B)][FMTLEN(1B)][data...]
        uint16_t p = pos + 4;              // start of payload
        uint16_t end = pos + 4 + LEN;
        while (p + 3 <= end) {
            uint16_t XDI = ((uint16_t)buf[p]<<8) | buf[p+1];
            uint8_t  fl  = buf[p+2];       // format+length (Xsens packs length here)
            p += 3;
            // For Acceleration (m/s^2): XDI = 0x4020, length = 12 bytes (3x float32 BE)
            if (XDI == 0x4020 && p + 12 <= end) {
                mti3_accel[0] = be_f32(&buf[p+0]);
                mti3_accel[1] = be_f32(&buf[p+4]);
                mti3_accel[2] = be_f32(&buf[p+8]);
            }
            // Advance by declared data length. For floats block this is 12.
            // If unsure, use remaining-to-next-block = (fl & 0x1F) * 4 for float fields.
            // Minimal path: try common 12B, fall back to skip by (fl & 0x1F).
            uint8_t words = (fl & 0x1F);   // number of 32-bit words per Xsens doc
            uint16_t dlen = (uint16_t)words * 4;
            if (p + dlen > end) break;
            p += dlen;
        }
    }

    return pos + need; // consume whole frame
}

void MTI3_Init(UART_HandleTypeDef *huart) {
    mhuart = huart;
    __HAL_UART_FLUSH_DRREGISTER(mhuart);
    HAL_UART_Receive_DMA(mhuart, rx_dma_buf, MTI3_RX_BUF_SZ);
    // enable circular mode in CubeMX on DMA; no further config here
}

void MTI3_Poll(void) {
    uint16_t pos = dma_pos();
    if (pos == dma_last) return;

    // Linearize the ring into a small temp window and parse
    if (pos > dma_last) {
        // single span
        const uint8_t *span = &rx_dma_buf[dma_last];
        uint16_t avail = pos - dma_last;
        uint16_t used = 0;
        while (used < avail) {
            uint16_t c = parse_one(span + used, avail - used);
            if (c == 0) break;
            used += c; 
        }
        dma_last = (dma_last + used) % MTI3_RX_BUF_SZ;
    } else {
        // wrap-around: first tail
        const uint8_t *span1 = &rx_dma_buf[dma_last];
        uint16_t avail1 = MTI3_RX_BUF_SZ - dma_last;
        uint16_t used1 = 0;
        while (used1 < avail1) {
            uint16_t c = parse_one(span1 + used1, avail1 - used1);
            if (c == 0) break;
            used1 += c;
        }
        dma_last = (dma_last + used1) % MTI3_RX_BUF_SZ;

        // then head
        const uint8_t *span2 = &rx_dma_buf[0];
        uint16_t avail2 = pos;
        uint16_t used2 = 0;
        while (used2 < avail2) {
            uint16_t c = parse_one(span2 + used2, avail2 - used2);
            if (c == 0) break;
            used2 += c;
        }
        dma_last = (dma_last + used2) % MTI3_RX_BUF_SZ;
    }
}