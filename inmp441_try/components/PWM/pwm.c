#include "pwm.h"

uint32_t ledc_duty_pow(uint32_t duty, uint8_t m, uint8_t n)
{
	uint32_t result = 1;
	while (n--)
	{
		result *= m;
	}

	return (result * duty) / 100;
	
}

void ledc_init(ledc_config_t *ledc_config)
{
	ledc_config->duty = ledc_duty_pow(ledc_config->duty, 2, ledc_config->duty_resolution);

	ledc_timer_config_t ledc_timer = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.duty_resolution = ledc_config->duty_resolution,
		.timer_num        = ledc_config->timer_num,         /* 定时器选择 */
        .freq_hz          = ledc_config->freq_hz,           /* 设置频率 */
        .clk_cfg          = ledc_config->clk_cfg            /* 设置时钟源 */
	};
	ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,      /* 低速模式 */
        .channel        = ledc_config->channel,     /* PWM输出通道 */
        .timer_sel      = ledc_config->timer_num,   /* 那个定时器提供计数值 */
        .intr_type      = LEDC_INTR_DISABLE,        /* 关闭LEDC中断 */
        .gpio_num       = ledc_config->gpio_num,    /* 输出GPIO管脚 */
        .duty           = ledc_config->duty,        /* 占空比 */
        .hpoint         = 0                         /* 设置hpoint数值 */
    };
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void ledc_pwm_set_duty(ledc_config_t *ledc_config, uint16_t duty)
{
    ledc_config->duty = ledc_duty_pow(duty, 2, ledc_config->duty_resolution);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ledc_config->channel, ledc_config->duty);    /* 设置占空比 */
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ledc_config->channel);                    /* 更新占空比 */
}
