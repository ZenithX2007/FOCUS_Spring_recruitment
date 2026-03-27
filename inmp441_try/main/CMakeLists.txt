idf_component_register(SRCS "main.c" 
                    INCLUDE_DIRS "."
                    REQUIRES iis_audio PWM MYUART espressif__esp-sr espressif__esp-dsp nvs_flash esp_driver_ledc esp_timer heap esp_wifi esp_netif mqtt driver )
