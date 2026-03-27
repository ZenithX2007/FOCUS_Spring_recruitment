#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include "esp_netif.h"
#include "mqtt_client.h"  // MQTT客户端头文件

/* ===================== WiFi 配置 ===================== */
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
// LCD显示缓冲区
char lcd_buff[100] = {0};

/* ===================== MQTT 配置（修改为你电脑的IP） ===================== */
// 【重要】改成你电脑的IP地址（电脑开MQTT服务器用）
#define MQTT_BROKER_URI     "mqtt://192.168.43.168"
// MQTT客户端名称（随便写，唯一即可）
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

/*
 * 函数：connet_display
 * 功能：WiFi状态显示接口（可接LCD/OLED）
 * 参数：flag=0 初始化  flag=1 连接失败  flag=2 连接成功
 */
void connet_display(uint8_t flag)
{
    if(flag == 2)
    {
        sprintf(lcd_buff, "ssid:%s", DEFAULT_SSID);
        sprintf(lcd_buff, "psw:%s", DEFAULT_PWD);
    }
    else if (flag == 1)
    {
        // 连接失败可在这里写显示逻辑
    }
    else
    {
        // 初始化状态
    }
}

/*
 * 函数：mqtt_event_handler
 * 功能：MQTT事件回调（处理连接、订阅、收消息）
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT 连接成功！");
            // 自动订阅主题
            esp_mqtt_client_subscribe(mqtt_client, MQTT_SUB_TOPIC, 0);
            ESP_LOGI(TAG, "已订阅主题: %s", MQTT_SUB_TOPIC);
            break;
        case MQTT_EVENT_DATA:
            // 收到电脑发来的MQTT消息
            ESP_LOGI(TAG, "收到MQTT消息：%.*s", event->data_len, event->data);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT 断开连接");
            break;
        default:
            break;
    }
}

/*
 * 函数：mqtt_client_init
 * 功能：初始化MQTT客户端
 */
void mqtt_client_init(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .credentials.client_id = MQTT_CLIENT_ID,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    // 注册MQTT事件
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    // 启动MQTT客户端
    esp_mqtt_client_start(mqtt_client);
}

/*
 * 函数：wifi_event_handler
 * 功能：WiFi事件回调处理（连接、断开、获取IP）
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    static int s_retry_num = 0;

    // WiFi STA启动事件
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        connet_display(0);
        // 自动连接WiFi
        esp_wifi_connect();
    }
    // WiFi已连接AP
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        connet_display(2);
    }
    // WiFi断开连接事件
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        // 重试连接（最多20次）
        if (s_retry_num < 20)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "正在重试连接WiFi...(%d/20)", s_retry_num);
        }
        else
        {
            // 重试次数用完，标记连接失败
            xEventGroupSetBits(wifi_event, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "WiFi连接失败");
    }
    // STA获取到IP地址
    else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "获取IP地址：" IPSTR, IP2STR(&event->ip_info.ip));
        // 重置重试次数
        s_retry_num = 0;
        // 标记连接成功
        xEventGroupSetBits(wifi_event, WIFI_CONNECTED_BIT);
        
        // ========== WiFi连上后，自动启动MQTT ==========
        mqtt_client_init();
    }
}

/*
 * 函数：wifi_sta_init
 * 功能：WiFi STA模式初始化（网络、事件、驱动、配置）
 */
void wifi_sta_init(void)
{
    static esp_netif_t *sta_netif = NULL;
    // 创建WiFi事件组
    wifi_event = xEventGroupCreate();
    
    // 初始化网卡
    ESP_ERROR_CHECK(esp_netif_init());
    // 创建默认事件循环
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // 创建WiFi STA网卡
    sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    // WiFi默认初始化配置
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    // 注册WiFi事件
    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL) );
    // 注册IP获取事件
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL) );
    
    // 初始化WiFi驱动
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));    
    // 加载WiFi配置
    wifi_config_t  wifi_config = WIFICONFIG();
    // 设置为STA模式
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    // 应用WiFi配置
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    // 启动WiFi
    ESP_ERROR_CHECK(esp_wifi_start());

    // 等待WiFi连接结果
    EventBits_t bits = xEventGroupWaitBits(wifi_event,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    // 判断连接状态
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
        ESP_LOGE(TAG, "未知事件");
    }
}

/*
 * 函数：app_main
 * 功能：程序入口
 */
void app_main(void)
{
    esp_err_t ret;

    // 初始化NVS（WiFi必须依赖）
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // 异常则擦除重新初始化
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // 启动WiFi STA
    wifi_sta_init();

    // 主循环
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
