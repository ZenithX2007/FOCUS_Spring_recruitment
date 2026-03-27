#include "iis_audio.h"
#include "driver/i2s_std.h"
#include "esp_err.h"
#include <stdlib.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

static int32_t *s_samples_32;
static i2s_chan_handle_t s_rx_ch;
static const char *TAG = "音频";

#define IIS_AUDIO_MAX_SAMPLES 1024

int iis_audio_read_16(int16_t *samples, int count)
{
    size_t bytes_read = 0;
    if (count > IIS_AUDIO_MAX_SAMPLES) {
        count = IIS_AUDIO_MAX_SAMPLES;
    }
    i2s_channel_read(s_rx_ch, (void *)s_samples_32, sizeof(int32_t) * count, &bytes_read, portMAX_DELAY);
    int samples_read = bytes_read / sizeof(int32_t);
    for (int i = 0; i < samples_read; i++) {
        int32_t v = s_samples_32[i] >> 11;
        if (v > INT16_MAX) v = INT16_MAX;
        else if (v < -INT16_MAX) v = -INT16_MAX;
        samples[i] = (int16_t)v;
    }
    return samples_read;
}

void iis_audio_convert_16_to_8(const int16_t *src, uint8_t *dst, uint8_t len)
{
    for (uint8_t i = 0; i < len; ++i) dst[i] = (uint16_t)(src[i] + 32768) >> 8;
}

void iis_audio_init(void)
{
    s_samples_32 = (int32_t *)malloc(sizeof(int32_t) * IIS_AUDIO_MAX_SAMPLES);
    i2s_chan_config_t ch_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_new_channel(&ch_cfg, NULL, &s_rx_ch);

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = 41, //BCKL BCK  SCK
            .ws = 40, //WS
            .dout = I2S_GPIO_UNUSED,
            .din = 42 //data SD
        },
    };
    i2s_channel_init_std_mode(s_rx_ch, &std_cfg);
    i2s_channel_enable(s_rx_ch);
    ESP_LOGI(TAG, "I2S接收通道已启用");
}