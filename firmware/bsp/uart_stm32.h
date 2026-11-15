#ifndef BSP_UART_STM32_H
#define BSP_UART_STM32_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * USART1: PA9 = TX, PA10 = RX
 *
 * 初始化交给 CubeMX 生成的 MX_USART1_UART_Init(), 这里只负责"发数据"。
 * 这样只有一处配置, 不会出现两套句柄打架。
 */

/* 返回 0 成功, -1 失败(HAL 超时或忙) */
int bsp_uart1_send(const uint8_t *data, uint16_t len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* BSP_UART_STM32_H */
