#include "pwm_stm32.h"

/* ---------------- 具体实现: STM32 TIM + HAL ---------------- */

static int stm32_set_pulse(ipwm_t *self, uint16_t pulse_us)
{
    pwm_stm32_t *me = (pwm_stm32_t *)self;

    if (me->htim == NULL) {
        return -1;
    }

    /* 限幅: 保证舵机不会被指令到机械极限之外 */
    if (pulse_us < me->min_us) pulse_us = me->min_us;
    if (pulse_us > me->max_us) pulse_us = me->max_us;

    __HAL_TIM_SET_COMPARE(me->htim, me->channel, pulse_us);
    return 0;
}

static int stm32_start(ipwm_t *self)
{
    pwm_stm32_t *me = (pwm_stm32_t *)self;

    if (me->htim == NULL) {
        return -1;
    }
    return (HAL_TIM_PWM_Start(me->htim, me->channel) == HAL_OK) ? 0 : -1;
}

static int stm32_stop(ipwm_t *self)
{
    pwm_stm32_t *me = (pwm_stm32_t *)self;

    if (me->htim == NULL) {
        return -1;
    }
    return (HAL_TIM_PWM_Stop(me->htim, me->channel) == HAL_OK) ? 0 : -1;
}

static const ipwm_ops_t s_pwm_ops = {
    .set_pulse = stm32_set_pulse,
    .start     = stm32_start,
    .stop      = stm32_stop,
};

/* ---------------- 对外接口 ---------------- */

void pwm_stm32_init(pwm_stm32_t *me, TIM_HandleTypeDef *htim, uint32_t channel,
                    uint16_t min_us, uint16_t max_us)
{
    if (me == NULL) {
        return;
    }
    me->base.ops   = &s_pwm_ops;
    me->base.ctx   = htim;
    me->htim       = htim;
    me->channel    = channel;
    me->min_us     = min_us;
    me->max_us     = max_us;
}

ipwm_t *pwm_stm32_as_ipwm(pwm_stm32_t *me)
{
    return (me == NULL) ? NULL : &me->base;
}
