#ifndef __LEDC_H
#define __LEDC_H

#include "driver/gpio.h"
#include "driver/ledc.h"

#define LEDC_PWM_TIMER	LEDC_TIMER_0
#define LEDC_PWM_CH0_GPIO	GPIO_NUM_1
#define LEDC_PWM_CH0_CHANNEL	LEDC_CHANNEL_0

typedef struct ledc
{
	ledc_clk_cfg_t clk_cfg;             /* 时钟源配置（LEDC_USE_XTAL_CLK\LEDC_USE_PLL_DIV_CLK\LEDC_USE_RC_FAST_CLK或者LEDC_AUTO_CLK(自动选择)） */
    ledc_timer_t  timer_num;            /* 定时器（LEDC_TIMER_0~LEDC_TIMER_3） */
    uint32_t freq_hz;                   /* 频率（系统自动计算分频系数） */
    ledc_timer_bit_t duty_resolution;   /* 占空比分辨率 */
    ledc_channel_t channel;             /* 通道（LEDC_CHANNEL_0~LEDC_CHANNEL_7） */
    uint32_t duty;                      /* 初始占空比 */
    int gpio_num;                          /* PWM输出管脚 */
}ledc_config_t;

void ledc_init(ledc_config_t *ledc_config);
void ledc_pwm_set_duty(ledc_config_t *ledc_config, uint16_t duty);

#endif