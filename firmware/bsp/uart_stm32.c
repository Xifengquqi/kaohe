#include "uart_stm32.h"
#include "stm32f1xx_hal.h"

static UART_HandleTypeDef s_uart1;

void bsp_uart1_init(uint32_t baud)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    /* PA9 = TX, 复用推挽输出 */
    gpio.Pin   = GPIO_PIN_9;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PA10 = RX, 上拉输入 */
    gpio.Pin  = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);

    s_uart1.Instance          = USART1;
    s_uart1.Init.BaudRate     = baud;
    s_uart1.Init.WordLength   = UART_WORDLENGTH_8B;
    s_uart1.Init.StopBits     = UART_STOPBITS_1;
    s_uart1.Init.Parity       = UART_PARITY_NONE;
    s_uart1.Init.Mode         = UART_MODE_TX_RX;
    s_uart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    s_uart1.Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init(&s_uart1);
}

int bsp_uart1_send(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (data == 0 || len == 0) {
        return -1;
    }
    if (HAL_UART_Transmit(&s_uart1, (uint8_t *)data, len, timeout_ms) != HAL_OK) {
        return -1;
    }
    return 0;
}
