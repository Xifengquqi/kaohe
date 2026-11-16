#include "uart_stm32.h"

/* ---------------- 具体实现: STM32 HAL ---------------- */

static int stm32_send(iuart_t *self, const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    uart_stm32_t *me = (uart_stm32_t *)self;

    if (me->huart == NULL || data == NULL || len == 0) {
        return -1;
    }
    if (HAL_UART_Transmit(me->huart, (uint8_t *)data, len, timeout_ms) != HAL_OK) {
        return -1;
    }
    return 0;
}

static int stm32_recv(iuart_t *self, uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    uart_stm32_t *me = (uart_stm32_t *)self;

    if (me->huart == NULL || buf == NULL || len == 0) {
        return -1;
    }
    if (HAL_UART_Receive(me->huart, buf, len, timeout_ms) != HAL_OK) {
        return -1;
    }
    return 0;
}

static const iuart_ops_t s_uart_ops = {
    .send = stm32_send,
    .recv = stm32_recv,
};

/* ---------------- 对外接口 ---------------- */

void uart_stm32_init(uart_stm32_t *me, UART_HandleTypeDef *huart)
{
    if (me == NULL) {
        return;
    }
    me->base.ops = &s_uart_ops;
    me->base.ctx = huart;
    me->huart    = huart;
}

iuart_t *uart_stm32_as_iuart(uart_stm32_t *me)
{
    return (me == NULL) ? NULL : &me->base;
}
