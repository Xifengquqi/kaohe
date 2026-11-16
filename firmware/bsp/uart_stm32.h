#ifndef BSP_UART_STM32_H
#define BSP_UART_STM32_H

/*
 * BSP 层: iuart 接口的 STM32 实现
 *
 * 只在这里出现 HAL 的类型; 上层拿到的是 iuart_t*。
 * 换到 F407 只需要再写一个 uart_f4.c, 上层不用改。
 */

#include "iuart.h"
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    iuart_t             base;    /* 必须放第一个成员: 强转时依赖这一点 */
    UART_HandleTypeDef *huart;
} uart_stm32_t;

void uart_stm32_init(uart_stm32_t *me, UART_HandleTypeDef *huart);

/* 取接口指针, 给上层用 */
iuart_t *uart_stm32_as_iuart(uart_stm32_t *me);

#ifdef __cplusplus
}
#endif

#endif /* BSP_UART_STM32_H */
