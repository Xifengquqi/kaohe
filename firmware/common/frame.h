#ifndef COMMON_FRAME_H
#define COMMON_FRAME_H

#include <stdint.h>

/* 帧格式见 docs/02-通信协议.md
 *
 *   AA 55 | LEN | CMD | DATA[LEN] | CRC_H CRC_L
 *   CRC 覆盖 LEN/CMD/DATA, 大端存放
 */

#define FRAME_HEAD1         0xAAu
#define FRAME_HEAD2         0x55u
#define FRAME_MAX_PAYLOAD   48u
#define FRAME_OVERHEAD      6u
#define FRAME_MAX_SIZE      (FRAME_MAX_PAYLOAD + FRAME_OVERHEAD)

/* 下位机 -> 上位机 */
#define CMD_ATTITUDE        0x01u
#define CMD_RAW             0x02u
#define CMD_STATUS          0x03u
#define CMD_ACK             0x04u
/* 上位机 -> 下位机 */
#define CMD_SET_TARGET      0x10u
#define CMD_SET_PID         0x11u
#define CMD_SET_ALPHA       0x12u
#define CMD_CALIB           0x13u
#define CMD_ENABLE          0x14u

typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t data[FRAME_MAX_PAYLOAD];
} Frame;

/* 把一帧打包到 out 里, 返回总字节数; 返回 0 表示参数非法或缓冲区不够 */
uint16_t frame_pack(uint8_t cmd, const uint8_t *data, uint8_t len,
                    uint8_t *out, uint16_t out_size);

/* 逐字节解析状态机 */
typedef enum {
    FP_HEAD1 = 0,
    FP_HEAD2,
    FP_LEN,
    FP_CMD,
    FP_DATA,
    FP_CRC_H,
    FP_CRC_L
} FrameParserState;

typedef struct {
    FrameParserState state;
    uint8_t  len;
    uint8_t  idx;
    uint16_t crc;
    Frame    frame;
} FrameParser;

void frame_parser_init(FrameParser *p);

/*
 * 喂一个字节:
 *   返回  1  收到一帧完整且校验通过的帧 (已写入 out)
 *   返回  0  还没收完
 *   返回 -1  这一帧被丢弃 (长度非法 / CRC 错), 解析器已复位
 */
int frame_parser_feed(FrameParser *p, uint8_t byte, Frame *out);

#endif
