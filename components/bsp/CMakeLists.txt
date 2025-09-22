idf_component_register(SRCS "src/ledseq.c"
                            "src/imu.c" 
                       INCLUDE_DIRS "interface"
                       REQUIRES drivers utils )