#ifndef INTERFACE_IPWM_H
#define INTERFACE_IPWM_H

/*
 * PWM 接口层
 *
 * 上层只说"给我一路 PWM, 设成 1500us", 不关心是哪块芯片、哪个定时器。
 * 换芯片只改 bsp/, 这一层不动。
 *
 * 和 iuart 一样用"函数指针表 + 不透明句柄"实现 C 版多态。
 */

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ipwm_s ipwm_t;

typedef struct {
    /* 设置脉宽(单位 us), 返回 0 成功, 负值失败 */
    int (*set_pulse)(ipwm_t *self, uint16_t pulse_us);
    /* 启动 PWM 输出 */
    int (*start)(ipwm_t *self);
    /* 停止 PWM 输出(舵机失去驱动, 不再出力) */
    int (*stop)(ipwm_t *self);
} ipwm_ops_t;

struct ipwm_s {
    const ipwm_ops_t *ops;
    void             *ctx;
};

static inline int ipwm_set_pulse(ipwm_t *p, uint16_t pulse_us)
{
    if (p == NULL || p->ops == NULL || p->ops->set_pulse == NULL) return -1;
    return p->ops->set_pulse(p, pulse_us);
}

static inline int ipwm_start(ipwm_t *p)
{
    if (p == NULL || p->ops == NULL || p->ops->start == NULL) return -1;
    return p->ops->start(p);
}

static inline int ipwm_stop(ipwm_t *p)
{
    if (p == NULL || p->ops == NULL || p->ops->stop == NULL) return -1;
    return p->ops->stop(p);
}

#ifdef __cplusplus
}
#endif

#endif /* INTERFACE_IPWM_H */
