#ifndef __PID_H
#define __PID_H

#include "main.h"
#include <math.h>

/* 微分模式选择 */
typedef enum {
    PID_DERIV_MODE_MEASURE,   // 测量值微分（抗目标值突变）
    PID_DERIV_MODE_ERROR      // 误差微分（适合定距控制）
} PID_DerivMode;

/* PID控制器结构体（桌宠优化版） */
typedef struct {
    // 1. PID核心参数
    float Kp;                   // 比例系数
    float Ki;                   // 积分系数
    float Kd;                   // 微分系数

    // 2. 输出限幅
    float output_max;           // 输出上限
    float output_min;           // 输出下限

    // 3. 状态变量
    float integral;             // 积分项累加值
    float prev_error;           // 上一次误差值
    float prev_measure;         // 上一次测量值
    float output;               // 最终输出值
    float prev_deriv;           // 上一次滤波后的微分项
    uint32_t last_time;         // 上一次计算时间（HAL_GetTick()）
    float max_integral_ratio;   // 积分项最大输出占比

    // 4. 优化参数
    float integral_threshold;   // 积分分离阈值
    float deriv_filter;         // 微分滤波系数（0.7~0.9）
    float dead_zone;            // 死区补偿绝对值（非比例）
    float target_value;         // 目标值

    // 5. 新增参数
    PID_DerivMode deriv_mode;   // 微分计算模式
    float sample_period;        // 固定采样周期（默认0.01=10ms）
    uint8_t enabled;            // PID使能标志（1=使能，0=禁用）
    uint32_t call_count;        // 调用次数（调试用）
} PID_Controller;

/* 函数声明 */
void PID_Init(PID_Controller* pid, float kp, float ki, float kd,
              float output_min, float output_max,
              float integral_threshold, float deriv_filter,
              float dead_zone, PID_DerivMode deriv_mode);

float PID_Calculate_Fixed(PID_Controller* pid, float setpoint, float measure);
float PID_Calculate(PID_Controller* pid, float setpoint, float measure);
void PID_SetTarget(PID_Controller* pid, float target);
float PID_GetTarget(PID_Controller* pid);
void PID_Reset(PID_Controller* pid);
void PID_Enable(PID_Controller* pid, uint8_t enable);
void PID_SetParams(PID_Controller* pid, float kp, float ki, float kd);
void PID_SetSamplePeriod(PID_Controller* pid, float period);  // 新增函数声明

#endif /* __PID_H */
