#include "pid.h"

/**
  * @brief  PID 初始化
  * @param  pid     : 控制器对象
  * @param  Kp/Ki/Kd: 三个系数
  * @param  out_min : 输出下限(云台用脉宽偏移量, 单位 us)
  * @param  out_max : 输出上限
  */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
              float out_min, float out_max)
{
    if (pid == 0) return;

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->target      = 0.0f;
    pid->integral    = 0.0f;
    pid->prev_error  = 0.0f;
    pid->output      = 0.0f;

    pid->out_min = out_min;
    pid->out_max = out_max;

    /* 积分限幅: 让积分项单独最多贡献一个输出上限, 避免"积分饱和" */
    if (Ki > 0.0001f || Ki < -0.0001f)
    {
        float lim = (out_max > -out_min) ? out_max : -out_min;
        pid->integral_limit = (lim > 0.0f) ? (lim / (Ki > 0.0f ? Ki : -Ki)) : 0.0f;
    }
    else
    {
        pid->integral_limit = 0.0f;
    }
}

/**
  * @brief 设置目标角度(度)
  */
void PID_SetTarget(PID_t *pid, float target)
{
    if (pid == 0) return;
    pid->target = target;
}

/**
  * @brief 清空积分和微分历史, 重新开始
  */
void PID_Reset(PID_t *pid)
{
    if (pid == 0) return;
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0.0f;
}

/**
  * @brief 位置式 PID 计算
  * @param  feedback : 当前测量值(度)
  * @param  dt       : 控制周期(秒), 本工程固定 0.01s
  * @retval 舵机脉宽偏移量(us)
  */
float PID_Compute(PID_t *pid, float feedback, float dt)
{
    float error, p_term, i_term, d_term, out;

    if (pid == 0) return 0.0f;
    if (dt <= 0.0f) dt = 0.001f;

    error = pid->target - feedback;

    /* 比例项 */
    p_term = pid->Kp * error;

    /* 积分项(带限幅) */
    pid->integral += error * dt;
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    i_term = pid->Ki * pid->integral;

    /* 微分项 */
    d_term = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    /* 输出限幅 */
    out = p_term + i_term + d_term;
    if (out > pid->out_max) out = pid->out_max;
    if (out < pid->out_min) out = pid->out_min;

    pid->output = out;
    return out;
}
