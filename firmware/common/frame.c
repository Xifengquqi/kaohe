#include "frame.h"
#include "crc16.h"

/* CRC 的增量计算: 每喂一个字节更新一次, 避免收完再从头算 */
static uint16_t crc16_update(uint16_t crc, uint8_t byte)
{
    uint8_t b;

    crc ^= (uint16_t)byte << 8;
    for (b = 0; b < 8; b++) {
        if (crc & 0x8000) {
            crc = (uint16_t)((crc << 1) ^ 0x1021);
        } else {
            crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint16_t frame_pack(uint8_t cmd, const uint8_t *data, uint8_t len,
                    uint8_t *out, uint16_t out_size)
{
    uint16_t total;
    uint16_t crc;
    uint8_t  i;

    if (out == 0) {
        return 0;
    }
    if (len > FRAME_MAX_PAYLOAD) {
        return 0;
    }
    if (data == 0 && len > 0) {
        return 0;
    }

    total = (uint16_t)len + FRAME_OVERHEAD;
    if (total > out_size) {
        return 0;
    }

    out[0] = FRAME_HEAD1;
    out[1] = FRAME_HEAD2;
    out[2] = len;
    out[3] = cmd;

    for (i = 0; i < len; i++) {
        out[4 + i] = data[i];
    }

    crc = crc16_ccitt(&out[2], (uint16_t)(len + 2));   /* LEN + CMD + DATA */
    out[4 + len]     = (uint8_t)(crc >> 8);            /* 大端 */
    out[4 + len + 1] = (uint8_t)(crc & 0xFF);

    return total;
}

void frame_parser_init(FrameParser *p)
{
    if (p == 0) {
        return;
    }
    p->state = FP_HEAD1;
    p->len   = 0;
    p->idx   = 0;
    p->crc   = 0xFFFF;
}

int frame_parser_feed(FrameParser *p, uint8_t byte, Frame *out)
{
    if (p == 0) {
        return -1;
    }

    switch (p->state) {
    case FP_HEAD1:
        if (byte == FRAME_HEAD1) {
            p->state = FP_HEAD2;
        }
        break;

    case FP_HEAD2:
        if (byte == FRAME_HEAD2) {
            p->state = FP_LEN;
        } else if (byte == FRAME_HEAD1) {
            p->state = FP_HEAD2;          /* 连续两个 AA, 重新等 55 */
        } else {
            p->state = FP_HEAD1;
        }
        break;

    case FP_LEN:
        if (byte > FRAME_MAX_PAYLOAD) {
            p->state = FP_HEAD1;
            return -1;
        }
        p->len   = byte;
        p->idx   = 0;
        p->crc   = 0xFFFF;
        p->crc   = crc16_update(p->crc, byte);
        p->state = FP_CMD;
        break;

    case FP_CMD:
        p->frame.cmd = byte;
        p->crc = crc16_update(p->crc, byte);
        p->state = (p->len == 0) ? FP_CRC_H : FP_DATA;
        break;

    case FP_DATA:
        p->frame.data[p->idx++] = byte;
        p->crc = crc16_update(p->crc, byte);
        if (p->idx >= p->len) {
            p->state = FP_CRC_H;
        }
        break;

    case FP_CRC_H:
        p->frame.len = p->len;
        p->crc = p->crc;                       /* 只是占位, 保持可读 */
        p->idx = byte;                         /* 先暂存高字节 */
        p->state = FP_CRC_L;
        break;

    case FP_CRC_L: {
        uint16_t rxcrc = (uint16_t)((p->idx << 8) | byte);

        p->state = FP_HEAD1;
        if (rxcrc != p->crc) {
            return -1;
        }
        if (out != 0) {
            *out = p->frame;
        }
        return 1;
    }

    default:
        p->state = FP_HEAD1;
        break;
    }

    return 0;
}
