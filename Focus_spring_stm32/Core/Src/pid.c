#include "pid.h"

/* 静态辅助函数：核心PID计算逻辑 */
static float PID_Compute_Core(PID_Controller* pid, float setpoint, float measure, float dt)
{
    if (!pid->enabled) {
        return 0.0f;
    }

    // 1. 计算当前误差
    float error = setpoint - measure;

    // 2. 比例项计算
    float proportional = pid->Kp * error;

    // 3. 积分项计算（积分分离+智能限幅）
    if (fabsf(error) < pid->integral_threshold) {
        pid->integral += error * dt;

        // 智能积分限幅：考虑比例项已占用的输出空间
        if (fabsf(pid->Ki) > 0.0001f) {
            float max_integral_contribution =
                (pid->output_max - pid->output_min) * pid->max_integral_ratio;

            // 考虑比例项已占用的空间（留出余量）
            float available_for_integral = max_integral_contribution - fabsf(proportional);
            available_for_integral = (available_for_integral > 0) ? available_for_integral : 0;

            // 计算积分限幅值
            float max_integral = available_for_integral / pid->Ki;
            float min_integral = -max_integral;

            // 应用积分限幅
            if (pid->integral > max_integral) pid->integral = max_integral;
            if (pid->integral < min_integral) pid->integral = min_integral;
        } else {
            // Ki太小，积分作用可忽略
            pid->integral = 0.0f;
        }
    }
    // 误差太大，积分分离（停止累积，但不清零避免突变）

    float integral = pid->Ki * pid->integral;

    // 4. 微分项计算（适配微分模式+低通滤波）
    float raw_deriv = 0.0f;

    if (pid->deriv_mode == PID_DERIV_MODE_ERROR) {
        // 误差微分：d(error)/dt = (error - prev_error)/dt
        // 优点：对设定值变化敏感，响应快速
        // 缺点：设定值突变时会产生微分冲击
        // 适合：桌宠定距控制（目标距离稳定）
        raw_deriv = (error - pid->prev_error) / dt;
    } else {
        // 测量值微分：-d(measure)/dt = (prev_measure - measure)/dt
        // 优点：抗设定值突变，避免微分冲击
        // 缺点：对设定值变化不敏感
        // 适合：目标值可能突变的场景
        raw_deriv = (pid->prev_measure - measure) / dt;
    }

    // 一阶低通滤波（平滑微分项）
    float deriv_filt = pid->deriv_filter * pid->prev_deriv +
                      (1.0f - pid->deriv_filter) * raw_deriv;
    float derivative = deriv_filt * pid->Kd;

    // 5. 保存状态
    pid->prev_deriv = deriv_filt;
    pid->prev_error = error;
    pid->prev_measure = measure;

    // 6. PID总和计算
    pid->output = proportional + integral + derivative;

    // 7. 智能死区补偿（先补偿后限幅）
    if (pid->dead_zone > 0.0f) {
        // 情况1：输出大于死区，补偿后需要二次限幅
        if (pid->output > pid->dead_zone) {
            pid->output += pid->dead_zone;
        }
        // 情况2：输出小于负死区，补偿后需要二次限幅
        else if (pid->output < -pid->dead_zone) {
            pid->output -= pid->dead_zone;
        }
        // 情况3：输出在死区内但有设定值，给最小启动PWM
        else if (fabsf(setpoint - measure) > 0.001f) {
            pid->output = (error > 0) ? pid->dead_zone : -pid->dead_zone;
        }
        // 情况4：输出在死区内且误差很小，保持原值（接近0）
        else {
            pid->output = 0.0f;
        }
    }

    // 8. 输出限幅（补偿后可能超限）
    if (pid->output > pid->output_max) pid->output = pid->output_max;
    if (pid->output < pid->output_min) pid->output = pid->output_min;

    return pid->output;
}

/**
  * @brief  PID控制器初始化（适配头文件参数）
  * @param  pid: PID控制器结构体指针
  * @param  kp: 比例系数（距离模式1.5-2.0）
  * @param  ki: 积分系数（0.01-0.1）
  * @param  kd: 微分系数（0.3-0.5）
  * @param  output_min: 输出最小值（-800）
  * @param  output_max: 输出最大值（800）
  * @param  integral_threshold: 积分分离阈值（15-20cm）
  * @param  deriv_filter: 微分滤波系数（0.7~0.9）
  * @param  dead_zone: 死区补偿绝对值（建议150-250）
  * @param  deriv_mode: 微分计算模式
  * @retval 无
  */
void PID_Init(PID_Controller* pid, float kp, float ki, float kd,
              float output_min, float output_max,
              float integral_threshold, float deriv_filter,
              float dead_zone, PID_DerivMode deriv_mode)
{
    // 1. PID核心参数
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    // 2. 输出限幅
    pid->output_min = output_min;
    pid->output_max = output_max;

    // 3. 优化参数
    pid->integral_threshold = integral_threshold;

    // 微分滤波系数限制在合理范围（0.7~0.9）
    pid->deriv_filter = (deriv_filter < 0.7f) ? 0.7f :
                       (deriv_filter > 0.9f) ? 0.9f : deriv_filter;

    // 死区补偿绝对值（直接使用传入值）
    pid->dead_zone = (dead_zone < 0.0f) ? 0.0f : dead_zone;

    // 4. 微分模式+新增参数
    pid->deriv_mode = deriv_mode;
    pid->sample_period = 0.01f;        // 默认10ms采样周期
    pid->max_integral_ratio = 0.3f;    // 积分项最大占比30%
    pid->target_value = 20.0f;         // 初始目标值20rpm
    pid->enabled = 1;                  // 默认使能PID
    pid->call_count = 0;               // 调用次数初始化

    // 5. 状态变量初始化
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measure = 0.0f;
    pid->output = 0.0f;
    pid->prev_deriv = 0.0f;
    pid->last_time = HAL_GetTick();
}

/**
  * @brief  PID计算（自适应周期，带完整优化）
  * @param  pid: PID控制器结构体指针
  * @param  setpoint: 设定值（目标值）
  * @param  measure: 测量值（反馈值）
  * @retval PID输出值
  */
float PID_Calculate(PID_Controller* pid, float setpoint, float measure)
{
    pid->call_count++;

    // 1. 时间间隔计算（带异常处理）
    uint32_t current_time = HAL_GetTick();
    float dt = (float)(current_time - pid->last_time) / 1000.0f;
    pid->last_time = current_time;

    // 异常时间间隔处理：使用配置的采样周期
    if (dt <= 0.0f || dt > 0.5f) {
        dt = pid->sample_period;
    }

    return PID_Compute_Core(pid, setpoint, measure, dt);
}

/**
  * @brief  PID计算（固定周期，适合定时器中断调用）
  * @param  pid: PID控制器结构体指针
  * @param  setpoint: 设定值
  * @param  measure: 测量值
  * @retval PID输出值
  */
float PID_Calculate_Fixed(PID_Controller* pid, float setpoint, float measure)
{
    pid->call_count++;

    // 使用固定采样周期
    float dt = pid->sample_period;

    return PID_Compute_Core(pid, setpoint, measure, dt);
}

/**
  * @brief  设置PID目标值
  * @param  pid: PID控制器结构体指针
  * @param  target: 目标值
  * @retval 无
  */
void PID_SetTarget(PID_Controller* pid, float target)
{
    pid->target_value = target;
}

/**
  * @brief  获取PID目标值
  * @param  pid: PID控制器结构体指针
  * @retval 当前目标值
  */
float PID_GetTarget(PID_Controller* pid)
{
    return pid->target_value;
}

/**
  * @brief  设置采样周期
  * @param  pid: PID控制器结构体指针
  * @param  period: 采样周期（秒）
  * @retval 无
  */
void PID_SetSamplePeriod(PID_Controller* pid, float period)
{
    if (period > 0.001f && period < 1.0f) {
        pid->sample_period = period;
    }
}

/**
  * @brief  PID状态重置
  * @param  pid: PID控制器结构体指针
  * @retval 无
  */
void PID_Reset(PID_Controller* pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measure = 0.0f;
    pid->output = 0.0f;
    pid->prev_deriv = 0.0f;
    pid->last_time = HAL_GetTick();
    pid->call_count = 0; // 重置调用次数
}

/**
  * @brief  PID使能/禁用
  * @param  pid: PID控制器结构体指针
  * @param  enable: 1=使能，0=禁用
  * @retval 无
  */
void PID_Enable(PID_Controller* pid, uint8_t enable)
{
    pid->enabled = enable;
    if (!enable) {
        pid->output = 0.0f; // 禁用时输出清零
    }
}

/**
  * @brief  动态修改PID参数
  * @param  pid: PID控制器结构体指针
  * @param  kp: 新的比例系数
  * @param  ki: 新的积分系数
  * @param  kd: 新的微分系数
  * @retval 无
  */
void PID_SetParams(PID_Controller* pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}
