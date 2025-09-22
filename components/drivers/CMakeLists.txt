idf_component_register(SRCS "src/motors.c"
                            "src/led.c"
                            "src/adc.c"
                            "src/i2c_dev.c"
                            "src/mpu6050.c"
                       INCLUDE_DIRS "interface"
                       REQUIRES driver esp_adc)