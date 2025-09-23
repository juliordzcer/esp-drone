idf_component_register(SRCS "src/ledseq.c"
                            "src/imu.c" 
                            "src/uart.c" 
                            "src/wifilink.c" 
                       INCLUDE_DIRS "interface"
                       REQUIRES drivers utils esp_wifi nvs_flash esp_event esp_netif)