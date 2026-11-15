#include "uart_stm32.h"
#include "stm32f1xx_hal.h"

/*
 * CubeMX 已经在 main.c 里生成了 huart1 和 MX_USART1_UART_Init(),
 * 这里直接借用那个句柄, 不再自己配 GPIO / 调 HAL_UART_Init,
 * 否则两套配置会互相覆盖。
 */
extern UART_HandleTypeDef huart1;

int bsp_uart1_send(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (data == 0 || len == 0) {
        return -1;
    }
    if (HAL_UART_Transmit(&huart1, (uint8_t *)data, len, timeout_ms) != HAL_OK) {
        return -1;
    }
    return 0;
}
