idf_component_register(SRCS 
                            "src/cfassert.c"
                            "src/configblock.c"
                            "src/cpuid.c"
                            "src/crc.c"
                            "src/debug.c"
                            "src/eprintf.c"
                            "src/filter.c"
                            "src/fp16.c"
                       INCLUDE_DIRS "interface"
                       REQUIRES modules)