#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "pwm.h"
#include "myuart.h"
#include <stdio.h>
#include <string.h>  // 字符串处理头文件

uint8_t dir = 1;// 1. 将呼吸灯控制变量定义为全局变量，保存状态
uint16_t ledpwmval = 0;
uint8_t breath_running = 0; // 运行标志
ledc_config_t *ledc_config = NULL;  


void app_main(void)
{
    uint8_t len = 0;
    uint16_t times = 0;
    unsigned char data[RX_BUF_SIZE] = {0};
          
	usart_init(115200); 
    
    ledc_config_t *ledc_config = malloc(sizeof(ledc_config_t));

    // LEDC(PWM)参数配置
    ledc_config->clk_cfg         = LEDC_AUTO_CLK;           // 自动选择时钟源
    ledc_config->timer_num       = LEDC_TIMER_0;            // 使用定时器0
    ledc_config->freq_hz         = 1000;                    // PWM频率：1KHz
    ledc_config->duty_resolution = LEDC_TIMER_14_BIT;      // 占空比分辨率：14位
    ledc_config->channel         = LEDC_CHANNEL_0;          // 使用通道0
    ledc_config->duty            = 0;                       // 初始占空比：0
    ledc_config->gpio_num        = LEDC_PWM_CH0_GPIO;       // PWM输出引脚
    ledc_init(ledc_config);      // 初始化PWM

    while(1)
    {
		uart_get_buffered_data_len(USART_UX, (size_t*) &len);                           

        if (len > 0) 
        {                                                      
            memset(data, 0, RX_BUF_SIZE);  // 清空接收缓冲区
            printf("The message you sent is:\n");
            uart_read_bytes(USART_UX, data, len, 100);  // 读取串口数据
            uart_write_bytes(USART_UX, (const char*)data, len);  // 回显数据

            if(strstr((const char*)data, "up") != NULL)  // 收到up指令
            {
                // 开启呼吸灯
                breath_running = 1; // 开启持续呼吸
                printf("breathing light on\n");
                
            }
            else if(strstr((const char*)data, "down") != NULL)  // 收到down指令
            {
                // 关闭呼吸灯
                breath_running = 0; // 停止呼吸
                ledc_pwm_set_duty(ledc_config, 0); // 占空比设为0，灯熄灭
                printf("breathing light off\n");
            }

        }
        else
        {
            times++;
            if (breath_running&&times % 2 == 0)
            {
                printf("30\n");
                if (dir == 1)
                    {
                        ledpwmval += 5; // 占空比递增
                    }
                    else
                    {
                        ledpwmval -= 5; // 占空比递减
                    }

                    if (ledpwmval == 100)
                    {
                        dir = 0;        // 达到最大值，切换为递减
                    }

                    if (ledpwmval == 0)
                    {
                        dir = 1;        // 达到最小值，切换为递增
                    }

                    // 设置PWM占空比
                    ledc_pwm_set_duty(ledc_config, ledpwmval);
            }
            vTaskDelay(10);
        }
    }
}


