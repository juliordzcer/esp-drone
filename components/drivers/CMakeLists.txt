idf_component_register(SRCS "src/motors.c"
                            "src/led.c"
                       INCLUDE_DIRS "interface"
                       REQUIRES driver esp_adc )