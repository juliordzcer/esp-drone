idf_component_register(SRCS "src/sensor_fusion.c"
                            "src/console.c"
                            "src/crtp.c"
                       INCLUDE_DIRS "interface" "src"
                       REQUIRES bsp)