#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"
#include "tim.h"

// ------------------------------------- 宏定义 -----------------------------------------------
#define GEAR_RATIO 48.0f                      // 电机减速比（实际硬件参数）
#define PPR 13                                // 基础脉冲数（实际硬件参数）
#define PULSES_PER_REV (PPR * 4) // 实际每圈脉冲数
#define TIM_MAX 0xFFFF                        // 定义定时器最大值
#define DELTA_T 0.01f    // 定时器更新周期，单位 s（例如 10ms）

         

// ------------------------------------- 函数声明 ---------------------------------------------
void ENCODER_Init(void);
void ENCODER_Update_Count(void);
int32_t ENCODER_GetCount(void);
float ENCODER_GetSpeed_rpm(void);


#endif
