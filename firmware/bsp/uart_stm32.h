#ifndef BSP_UART_STM32_H
#define BSP_UART_STM32_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * USART1: PA9 = TX, PA10 = RX
 * 只做最小封装, 给调试输出 / 上位机通信用
 */
void bsp_uart1_init(uint32_t baud);

/* 返回 0 成功, -1 失败(HAL 超时或忙) */
int bsp_uart1_send(const uint8_t *data, uint16_t len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* BSP_UART_STM32_H */
