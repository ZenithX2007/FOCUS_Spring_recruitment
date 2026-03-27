#include "encoder.h"
#include "tim.h"
#include <stdlib.h>

// 上次计数
static int32_t last_cnt = 0;
static int32_t total_cnt = 0;
static float current_rpm = 0.0f;
static int32_t prev_total = 0;

void ENCODER_Init()
{ 
    // 3.6 清零计数器
	  __HAL_TIM_SET_COUNTER(&htim3, 0);
	
    last_cnt = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    total_cnt = 0;
	  prev_total = 0;
   
}
void ENCODER_Update_Count(void)
{
    int32_t cnt_now = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);// 读取当前脉冲数
	  int32_t delta = cnt_now - last_cnt;   // 和上一次差值=一次累计计数

    // 处理计数器溢出（16位定时器）
    if (delta > 32767) delta -= 65536;
    else if (delta < -32768) delta += 65536;

    total_cnt += delta; // 累加到总脉冲数
    last_cnt = cnt_now; // 更新上一次脉冲数
	
	// 获取编码器实际转速(r/min)
  // 中断调用 10ms
  // cnt_now 当前累计计数
	  // 10ms内的脉冲差
    int32_t speed_delta = total_cnt - prev_total;
    prev_total = total_cnt;

    if (abs(speed_delta) < 2) {  // 提高滤波阈值
        current_rpm = 0.0f;
    } 
		else {
			// 每秒多少脉冲
        float pulses_per_sec = speed_delta / DELTA_T;
			// 计算转速 RPM
        current_rpm = (pulses_per_sec / PULSES_PER_REV /GEAR_RATIO) * 60.0f;
    }
	
}

// 获取累计脉冲
int32_t ENCODER_GetCount(void)
{  
    return total_cnt;
}

// 获取编码器实际转速(r/min)
// 中断调用 10ms
// cnt_now 当前累计计数
float ENCODER_GetSpeed_rpm(void)
{
    return current_rpm;
}
