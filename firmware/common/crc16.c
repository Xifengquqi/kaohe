#include "crc16.h"

uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;
    uint8_t  b;

    if (data == 0) {
        return crc;
    }

    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (b = 0; b < 8; b++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}
