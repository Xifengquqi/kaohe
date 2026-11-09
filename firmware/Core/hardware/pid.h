#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* 位置式 PID 控制器 */
typedef struct
{
    float Kp;              /* 比例系数 */
    float Ki;              /* 积分系数 */
    float Kd;              /* 微分系数 */

    float target;          /* 目标值(希望保持的姿态角) */
    float integral;        /* 积分累加项 */
    float prev_error;      /* 上一次偏差, 用于计算微分 */
    float output;          /* 最近一次的输出值 */

    float out_min;         /* 输出下限 */
    float out_max;         /* 输出上限 */
    float integral_limit;  /* 积分限幅, 防止积分饱和 */
} PID_t;

void  PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
               float out_min, float out_max);
void  PID_SetTarget(PID_t *pid, float target);
void  PID_Reset(PID_t *pid);
float PID_Compute(PID_t *pid, float feedback, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
