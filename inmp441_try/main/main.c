#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_dsp.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_afe_config.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"
#include "esp_wn_iface.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "iis_audio.h"

#include "model_path.h"
#include "nvs_flash.h"

#include "esp_netif.h"
#include "mqtt_client.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_mac.h"

#include "pwm.h"
#include "myuart.h"

// WiFi名称
#define DEFAULT_SSID        "summit"
// WiFi密码
#define DEFAULT_PWD         "xzjzxljmkjjs"

/* ===================== 事件标志定义 ===================== */
// WiFi事件组句柄
static EventGroupHandle_t   wifi_event;
// WiFi连接成功标志位
#define WIFI_CONNECTED_BIT  BIT0
// WiFi连接失败标志位
#define WIFI_FAIL_BIT       BIT1
// 日志TAG
static const char *TAG = "WIFI_MQTT_STA";



/* ===================== MQTT 配置 ===================== */
#define MQTT_BROKER_URI     "mqtt://192.168.43.168"
#define MQTT_CLIENT_ID      "esp32_client_001"
// 订阅主题
#define MQTT_SUB_TOPIC      "/esp32/sub"
// 发布主题
#define MQTT_PUB_TOPIC      "/esp32/pub"




// MQTT客户端句柄
esp_mqtt_client_handle_t mqtt_client;

/* ===================== WiFi默认配置结构体 ===================== */
#define WIFICONFIG()   {                            \
    .sta = {                                        \
        .ssid = DEFAULT_SSID,                       \
        .password = DEFAULT_PWD,                    \
        .threshold.authmode = WIFI_AUTH_WPA2_PSK,   \
    },                                              \
}


//呼吸灯设置
uint8_t dir = 1;
uint16_t ledpwmval = 0;
uint8_t breath_running = 0; 
ledc_config_t *ledc_config = NULL;  








#define PARTITION_LABEL_MODEL "model"

#define FFT_SIZE 128
#define LOW_FREQ_BIN_MAX 4
#define MN_WINDOW_MS 6000

#define CMD_ID_ON 0
#define CMD_ID_OFF 1
#define CMD_ID_BLINK 2
#define CMD_ID_DIM_DOWN 3

#define BASS_HIT_FRAMES 3
#define BASS_COOLDOWN_US (400 * 1000)
#define BASS_RELATIVE_RATIO 2.5f
#define BASS_MIN_ABS 35.0f
#define BASS_FLOOR_EMA_ALPHA 0.97f

static __attribute__((aligned(16))) float s_fftWind[FFT_SIZE];
static __attribute__((aligned(16))) float s_fftWork[FFT_SIZE * 2];
static int s_fftReady;
static volatile bool gSrBusy;



// 亮度阶梯控制FFT
void brightness_step_down(void)
{
    ledpwmval -= 20;
    if(ledpwmval > 100) ledpwmval = 100;
    if(ledpwmval < 0) ledpwmval = 0;
    ledc_pwm_set_duty(ledc_config, ledpwmval);
    ESP_LOGI(TAG, "FFT触发：亮度降低一档 => %d", ledpwmval);
}






static void applyCommandId(int commandId)
{
    switch (commandId) {
    case CMD_ID_ON:
        breath_running = 0;
        ledc_pwm_set_duty(ledc_config, 100);
        ESP_LOGI(TAG, "语音：开灯");
        break;
    case CMD_ID_OFF:
        breath_running = 0;
        ledc_pwm_set_duty(ledc_config, 0);
        ESP_LOGI(TAG, "语音：关灯");
        break;
    case CMD_ID_BLINK:
        breath_running = 1;
        ESP_LOGI(TAG, "语音：呼吸灯");
        break;
    case CMD_ID_DIM_DOWN:
        brightness_step_down();
        break;
    default:
        break;
    }
}





static void fftRunAndFillWork(const int16_t *samples16, int sampleCount)
{
    for (int i = 0; i < FFT_SIZE; ++i) {
        float x = 0.0f;
        if (i < sampleCount) {
            x = (float)samples16[i] / 32768.0f;
        }
        s_fftWork[2 * i] = x * s_fftWind[i];
        s_fftWork[2 * i + 1] = 0.0f;
    }
    dsps_fft2r_fc32(s_fftWork, FFT_SIZE);
    dsps_bit_rev_fc32(s_fftWork, FFT_SIZE);
}

static float fftLowBandEnergy(void)
{
    float lowEnergy = 0.0f;
    for (int k = 1; k <= LOW_FREQ_BIN_MAX; k++) {
        float re = s_fftWork[2 * k];
        float im = s_fftWork[2 * k + 1];
        float mag = sqrtf(re * re + im * im);
        lowEnergy += mag;
    }
    return lowEnergy;
}

static bool bassEventUpdate(float lowEnergy, int64_t nowUs)
{
    static float s_noiseFloor = 20.0f;
    static int s_consecutiveHits;
    static int64_t s_lastTriggerUs;

    s_noiseFloor = s_noiseFloor * BASS_FLOOR_EMA_ALPHA + lowEnergy * (1.0f - BASS_FLOOR_EMA_ALPHA);

    float threshold = s_noiseFloor * BASS_RELATIVE_RATIO;
    if (threshold < BASS_MIN_ABS) {
        threshold = BASS_MIN_ABS;
    }

    if (gSrBusy) {
        threshold *= 4.0f;
    }

    bool over = lowEnergy > threshold;
    if (over) {
        s_consecutiveHits++;
    } else {
        s_consecutiveHits = 0;
    }

    if (s_consecutiveHits < BASS_HIT_FRAMES) {
        return false;
    }
    s_consecutiveHits = 0;

    if ((nowUs - s_lastTriggerUs) < BASS_COOLDOWN_US) {
        return false;
    }
    s_lastTriggerUs = nowUs;
    return true;
}

static void audioTask(void *arg)
{
    (void)arg;
    srmodel_list_t *models = esp_srmodel_init(PARTITION_LABEL_MODEL);
    if (models == NULL) {
        ESP_LOGE(TAG, "esp_srmodel_init 失败，请确认已烧录 model 分区（srmodels.bin）");
    } else {
        for (int i = 0; i < models->num; i++) {
            ESP_LOGI(TAG, "model[%d]: %s", i, models->model_name[i]);
        }
    }

    const esp_afe_sr_iface_t *afeIface = NULL;
    esp_afe_sr_data_t *afeData = NULL;
    esp_mn_iface_t *multinet = NULL;
    model_iface_data_t *mnData = NULL;

    if (models != NULL) {
        afe_config_t *afeCfg = afe_config_init("M", models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
        afeCfg->aec_init = false;
        afeCfg->memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_INTERNAL;
        afe_config_check(afeCfg);
        afeIface = esp_afe_handle_from_config(afeCfg);
        afeData = afeIface->create_from_config(afeCfg);
        afe_config_free(afeCfg);
        if (afeData == NULL) {
            ESP_LOGE(TAG, "AFE create_from_config 失败");
            afeIface = NULL;
        }

        char *mnName = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_CHINESE);
        if (mnName == NULL) {
            ESP_LOGE(TAG, "未找到中文 MultiNet 模型");
        } else {
            multinet = esp_mn_handle_from_name(mnName);
            mnData = multinet->create(mnName, MN_WINDOW_MS);
            if (multinet->switch_loader_mode) {
                multinet->switch_loader_mode(mnData, ESP_MN_LOAD_FROM_FLASH);
            }

            ESP_ERROR_CHECK(esp_mn_commands_alloc(multinet, mnData));
            ESP_ERROR_CHECK(esp_mn_commands_add(CMD_ID_ON, "开灯"));
            ESP_ERROR_CHECK(esp_mn_commands_add(CMD_ID_OFF, "熄灯"));
            ESP_ERROR_CHECK(esp_mn_commands_add(CMD_ID_BLINK, "闪烁"));
            ESP_ERROR_CHECK(esp_mn_commands_add(CMD_ID_DIM_DOWN, "暗度调低一档"));
            esp_mn_error_t *mnErr = esp_mn_commands_update();
            if (mnErr != NULL && mnErr->num > 0) {
                ESP_LOGW(TAG, "MultiNet 词条部分未加入: %d", mnErr->num);
            }
            esp_mn_commands_print();
        }
    }

    int feedChunkSize = 512;
    if (afeIface && afeData) {
        feedChunkSize = afeIface->get_feed_chunksize(afeData);
        ESP_LOGI(TAG, "AFE feed_chunk=%d", feedChunkSize);
    }
    if (feedChunkSize > 1024) {
        feedChunkSize = 1024;
    }

    int16_t *feedBuffer = (int16_t *)heap_caps_malloc(feedChunkSize * sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (feedBuffer == NULL) {
        ESP_LOGE(TAG, "feedBuffer 分配失败");
        vTaskDelete(NULL);
        return;
    }

    int mnChunkSize = 0;
    if (multinet && mnData) {
        mnChunkSize = multinet->get_samp_chunksize(mnData);
    }

    bool mnListening = false;

    for (;;) {
        int total = 0;
        while (total < feedChunkSize) {
            int n = iis_audio_read_16(feedBuffer + total, feedChunkSize - total);
            if (n <= 0) {
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
            total += n;
        }

        int16_t *fftSrc = feedBuffer;
        int fftN = FFT_SIZE;
        if (feedChunkSize >= FFT_SIZE) {
            fftSrc = feedBuffer + (feedChunkSize - FFT_SIZE);
        }

        if (s_fftReady) {
            fftRunAndFillWork(fftSrc, fftN);
            float lowEnergy = fftLowBandEnergy();
            int64_t nowUs = esp_timer_get_time();

            if (bassEventUpdate(lowEnergy, nowUs)) {
                ESP_LOGI(TAG, "低频冲击触发 -> 暗度调低一档");

            }
        }

        if (afeIface == NULL || afeData == NULL) {
            continue;
        }

        afeIface->feed(afeData, feedBuffer);
        afe_fetch_result_t *fetchResult = afeIface->fetch(afeData);
        if (fetchResult == NULL) {
            continue;
        }

        gSrBusy = mnListening || (fetchResult->wakeup_state != WAKENET_NO_DETECT);

        if (fetchResult->wakeup_state == WAKENET_DETECTED) {
            ESP_LOGI(TAG, "唤醒词检测到");
            if (multinet && mnData) {
                multinet->clean(mnData);
            }
            mnListening = true;
        }

        if (multinet && mnData && mnListening && fetchResult->data != NULL && fetchResult->data_size > 0 && mnChunkSize > 0) {
            int bytes = fetchResult->data_size;
            int samples = bytes / (int)sizeof(int16_t);
            if (samples >= mnChunkSize) {
                esp_mn_state_t mnState = multinet->detect(mnData, fetchResult->data);
                if (mnState == ESP_MN_STATE_DETECTED) {
                    esp_mn_results_t *mr = multinet->get_results(mnData);
                    if (mr && mr->num > 0) {
                        int cmd = mr->command_id[0];
                        float p = mr->prob[0];
                        ESP_LOGI(TAG, "命令 id=%d prob=%.3f", cmd, p);
                        applyCommandId(cmd);
                    }
                    mnListening = false;
                } else if (mnState == ESP_MN_STATE_TIMEOUT) {
                    mnListening = false;
                }
            }
        }

        taskYIELD();
    }
}

void connet_display(uint8_t flag)
{
    if(flag == 2)
    {
        
    }
    else if (flag == 1)
    {
        
    }
    else
    {
       
    }
}






static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT 连接成功！");
            esp_mqtt_client_subscribe(mqtt_client, MQTT_SUB_TOPIC, 0);
            ESP_LOGI(TAG, "已订阅主题 %s", MQTT_SUB_TOPIC);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "收到MQTT消息：%.*s", event->data_len, event->data);
            
            // up开呼吸灯，down关
            if(strstr((char*)event->data, "up")) {
                breath_running = 1;
                printf("WIFI控制：呼吸灯开启\n");
            }
            if(strstr((char*)event->data, "down")) {
                breath_running = 0;
                ledc_pwm_set_duty(ledc_config, 0);
                printf("WIFI控制：呼吸灯关闭\n");
            }
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT 断开连接");
            break;
        default:
            break;
    }
}


void mqtt_client_init(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .credentials.client_id = MQTT_CLIENT_ID,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    static int s_retry_num = 0;
    
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        connet_display(0);
        
        esp_wifi_connect();
    }
    
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        connet_display(2);
    }
    
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        
        if (s_retry_num < 20)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "正在重试连接WiFi...(%d/20)", s_retry_num);
        }
        else
        {
            
            xEventGroupSetBits(wifi_event, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "WiFi连接失败");
    }
    
    else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "获取IP地址" IPSTR, IP2STR(&event->ip_info.ip));
        
        s_retry_num = 0;
        
        xEventGroupSetBits(wifi_event, WIFI_CONNECTED_BIT);
        
        mqtt_client_init();
    }
}


void wifi_sta_init(void)
{
    static esp_netif_t *sta_netif = NULL;
   
    wifi_event = xEventGroupCreate();
    
   
    ESP_ERROR_CHECK(esp_netif_init());
    
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    
    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL) );
    
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL) );
    
   
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));    
    
    wifi_config_t  wifi_config = WIFICONFIG();
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
   
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    
    ESP_ERROR_CHECK(esp_wifi_start());

    
    EventBits_t bits = xEventGroupWaitBits(wifi_event,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "WiFi连接成功：SSID:%s", DEFAULT_SSID);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        connet_display(1);
        ESP_LOGE(TAG, "WiFi连接失败：SSID:%s", DEFAULT_SSID);
    }
    else
    {
        ESP_LOGE(TAG, "未查事件");
    }
}



// ==================== 【设备工作模式】2种 ====================
typedef enum {
    MODE_WIFI,      // WiFi 控制模式
  
    MODE_VOICE      // 语音控制模式
} device_mode_t;

// ==================== 【触发事件】外部信号 ====================
typedef enum {
    EVENT_SWITCH_TO_WIFI,
  
    EVENT_SWITCH_TO_VOICE
} device_event_t;

// ==================== 当前模式 ====================
static volatile device_mode_t current_mode = MODE_WIFI;

// ==================== 切换模式（核心） ====================
void set_device_mode(device_mode_t new_mode)
{
    current_mode = new_mode;
    printf("模式已切换：%d\n", current_mode);
}

// ==================== 事件入口（外部触发） ====================
void post_event(device_event_t event)
{
    switch(event)
    {
        case EVENT_SWITCH_TO_WIFI:
            set_device_mode(MODE_WIFI);
            break;

        case EVENT_SWITCH_TO_VOICE:
            set_device_mode(MODE_VOICE);
            break;
    }
}

// ==================== 【 if 模式判断】主逻辑 ====================
void device_mode_run(void)
{
    static bool wifi_inited = false;
    static bool voice_inited = false;
    if(current_mode == MODE_WIFI)
    {
        if (!wifi_inited) {
            wifi_inited = true;
            voice_inited = false;

            esp_err_t ret;
            ret = nvs_flash_init();
            if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
            {
                ESP_ERROR_CHECK(nvs_flash_erase());
                ESP_ERROR_CHECK(nvs_flash_init());
            }
            wifi_sta_init();
        }


    }
    else if(current_mode == MODE_VOICE)
    {
        if (!voice_inited) {
            voice_inited = true;
            wifi_inited = false;

            esp_err_t err = nvs_flash_init();
            if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
                ESP_ERROR_CHECK(nvs_flash_erase());
                err = nvs_flash_init();
            }
            ESP_ERROR_CHECK(err);

            iis_audio_init();

            dsps_wind_hann_f32(s_fftWind, FFT_SIZE);
            esp_err_t fftErr = dsps_fft2r_init_fc32(NULL, FFT_SIZE);
            if (fftErr != ESP_OK) {
                ESP_LOGE(TAG, "FFT 初始化失败: %s", esp_err_to_name(fftErr));
                s_fftReady = 0;
            } else {
                s_fftReady = 1;
            }

            xTaskCreate(audioTask, "audio_task", 40960, NULL, 5, NULL);
            ESP_LOGI(TAG, "audio_task 已启动");
        }
    }  
}

// ==================== 状态机任务 ====================
void device_task(void *arg)
{
    while(1)
    {
        device_mode_run();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==================== 入口 ====================
void app_main(void)
{

    uint8_t len = 0;
    uint16_t times = 0;
    unsigned char data[RX_BUF_SIZE] = {0};
          
	usart_init(115200); 
    
    ledc_config = malloc(sizeof(ledc_config_t));

    // LEDC(PWM)
    ledc_config->clk_cfg         = LEDC_AUTO_CLK;           
    ledc_config->timer_num       = LEDC_TIMER_0;           
    ledc_config->freq_hz         = 1000;                  
    ledc_config->duty_resolution = LEDC_TIMER_14_BIT;      
    ledc_config->channel         = LEDC_CHANNEL_0;          
    ledc_config->duty            = 0;                      
    ledc_config->gpio_num        = LEDC_PWM_CH0_GPIO;       
    ledc_init(ledc_config);      






    xTaskCreate(device_task, "device_task", 4096, NULL, 5, NULL);



   while(1)
    {
		uart_get_buffered_data_len(USART_UX, (size_t*) &len);                           

        if (len > 0) 
        {                                                      
            memset(data, 0, RX_BUF_SIZE);  
            printf("The message you sent is:\n");
            uart_read_bytes(USART_UX, data, len, 100);  
            uart_write_bytes(USART_UX, (const char*)data, len); 

            if(strstr((const char*)data, "WIFI") != NULL){  
               post_event(EVENT_SWITCH_TO_WIFI);
                
            }
            else if(strstr((const char*)data, "VOICE") != NULL)  
            {
                post_event(EVENT_SWITCH_TO_VOICE);
            }  

        }
        else
        {
            
            if (breath_running== 1)
            {
               
                if (dir == 1)
                    {
                        ledpwmval += 5;
                    }
                    else
                    {
                        ledpwmval -= 5; 
                    }

                    if (ledpwmval == 100)
                    {
                        dir = 0;       
                    }

                    if (ledpwmval == 0)
                    {
                        dir = 1;     
                    }   

                    // 璁剧疆PWM鍗犵┖姣?
                    ledc_pwm_set_duty(ledc_config, ledpwmval);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
   
   


}
