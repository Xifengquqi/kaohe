#ifndef INTERFACE_IUART_H
#define INTERFACE_IUART_H

/*
 * 串口接口层
 *
 * 上层(device / model / app)只认 iuart_t, 永远不出现 UART_HandleTypeDef,
 * 换芯片时只改 bsp/, 这一层不动。
 *
 * C 语言版"虚函数": 一个函数指针表 + 一个不透明句柄。
 */

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct iuart_s iuart_t;

typedef struct {
    /* 返回 0 成功, 负值失败 */
    int (*send)(iuart_t *self, const uint8_t *data, uint16_t len, uint32_t timeout_ms);
    int (*recv)(iuart_t *self, uint8_t *buf, uint16_t len, uint32_t timeout_ms);
} iuart_ops_t;

struct iuart_s {
    const iuart_ops_t *ops;   /* 具体实现的操作表 */
    void              *ctx;   /* 具体实现自己的数据 */
};

static inline int iuart_send(iuart_t *u, const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (u == NULL || u->ops == NULL || u->ops->send == NULL) {
        return -1;
    }
    return u->ops->send(u, data, len, timeout_ms);
}

static inline int iuart_recv(iuart_t *u, uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    if (u == NULL || u->ops == NULL || u->ops->recv == NULL) {
        return -1;
    }
    return u->ops->recv(u, buf, len, timeout_ms);
}

#ifdef __cplusplus
}
#endif

#endif /* INTERFACE_IUART_H */
