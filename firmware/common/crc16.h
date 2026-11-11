#ifndef COMMON_CRC16_H
#define COMMON_CRC16_H

#include <stdint.h>

/*
 * CRC16-CCITT-FALSE
 *   多项式 0x1021, 初值 0xFFFF, 不反转, 结果不异或
 *   和协议文档 docs/02-通信协议.md 保持一致
 */
uint16_t crc16_ccitt(const uint8_t *data, uint16_t len);

#endif
