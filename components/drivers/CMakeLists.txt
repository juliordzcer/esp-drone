idf_component_register(SRCS "src/motors.c"
                            "src/led.c"
                            "src/adc.c"
                       INCLUDE_DIRS "interface"
                       REQUIRES driver esp_adc )