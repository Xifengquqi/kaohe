#ifndef BSP_PWM_STM32_H
#define BSP_PWM_STM32_H

/*
 * BSP 层: ipwm 接口的 STM32 定时器实现
 *
 * 一个 pwm_stm32_t 对应"一个定时器的一个通道"。
 * 上层拿到的是 ipwm_t*, 看不到 TIM_HandleTypeDef。
 */

#include "ipwm.h"
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    ipwm_t              base;      /* 必须是第一个成员 */
    TIM_HandleTypeDef  *htim;
    uint32_t            channel;   /* TIM_CHANNEL_x */
    uint16_t            min_us;
    uint16_t            max_us;
} pwm_stm32_t;

/* min_us/max_us: 脉宽安全范围, 超出会被钳住, 防止把舵机顶死 */
void pwm_stm32_init(pwm_stm32_t *me, TIM_HandleTypeDef *htim, uint32_t channel,
                    uint16_t min_us, uint16_t max_us);

ipwm_t *pwm_stm32_as_ipwm(pwm_stm32_t *me);

#ifdef __cplusplus
}
#endif

#endif /* BSP_PWM_STM32_H */
