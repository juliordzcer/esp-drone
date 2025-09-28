# components/bsp/CMakeLists.txt
idf_component_register(SRCS  
                            "src/imu.c" 
                            "src/ledseq.c"
                            "src/pm.c" 
                         #    "src/uart.c" 
                            "src/wifilink.c" 
                       INCLUDE_DIRS "interface"
                       REQUIRES drivers modules utils esp_wifi nvs_flash esp_event esp_netif)