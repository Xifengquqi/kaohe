#ifndef DEVICE_SERVO_H
#define DEVICE_SERVO_H

/*
 * Device 层: 舵机
 *
 * 职责: 把"角度"翻译成"脉宽", 并负责方向/行程/中位。
 * 它只依赖 ipwm 接口, 不关心底层是哪个定时器。
 *
 * 上层用法:
 *     servo_set_angle(&svo, 15.0f);   // 转 15 度
 *     servo_set_pulse(&svo, 1500);    // 或者直接给脉宽
 */

#include "ipwm.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    ipwm_t  *pwm;          /* 底层 PWM 接口 */
    int16_t  sign;         /* +1 / -1: 安装方向反了就用 -1 */
    uint16_t mid_us;       /* 中位脉宽, 一般是 1500 */
    uint16_t half_range_us;/* 从中位到一端的脉宽范围, 例如 500 → 1000~2000 */
} servo_t;

/* 初始化: pwm 已创建; mid_us 通常 1500; half_range_us 通常 500 */
void servo_init(servo_t *s, ipwm_t *pwm, int16_t sign,
                uint16_t mid_us, uint16_t half_range_us);

/* 设置角度(度), 0 度 = 中位。返回值 0 成功 */
int servo_set_angle(servo_t *s, float deg);

/* 直接设置脉宽(us), 内部会加中位? 不会, 这是绝对脉宽 */
int servo_set_pulse(servo_t *s, int16_t pulse_us);

/* 回中位 */
int servo_center(servo_t *s);

#ifdef __cplusplus
}
#endif

#endif /* DEVICE_SERVO_H */
