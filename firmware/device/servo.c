#include "servo.h"

/* ---------------- 初始化 ---------------- */

void servo_init(servo_t *s, ipwm_t *pwm, int16_t sign,
                uint16_t mid_us, uint16_t half_range_us)
{
    if (s == NULL) {
        return;
    }
    s->pwm           = pwm;
    s->sign          = (sign >= 0) ? 1 : -1;
    s->mid_us        = mid_us;
    s->half_range_us = half_range_us;
}

/* ---------------- 控制 ---------------- */

int servo_set_pulse(servo_t *s, int16_t pulse_us)
{
    if (s == NULL || s->pwm == NULL) {
        return -1;
    }
    return ipwm_set_pulse(s->pwm, (uint16_t)pulse_us);
}

int servo_set_angle(servo_t *s, float deg)
{
    float   offset;
    int32_t pulse;

    if (s == NULL || s->pwm == NULL) {
        return -1;
    }

    /* 角度 → 相对中位的脉宽偏移: ±90 度 对应 ±half_range_us */
    offset = (deg / 90.0f) * (float)s->half_range_us;
    pulse  = (int32_t)s->mid_us + (int32_t)(offset * (float)s->sign);

    /* 再限一次, 防止角度过大把脉宽推到非法区间 */
    if (pulse < 500)  pulse = 500;
    if (pulse > 2500) pulse = 2500;

    return ipwm_set_pulse(s->pwm, (uint16_t)pulse);
}

int servo_center(servo_t *s)
{
    return servo_set_pulse(s, (int16_t)(s ? s->mid_us : 1500));
}
