# components/utils/CMakeLists.txt
idf_component_register(SRCS 
                            "src/cfassert.c"
                            "src/configblock.c"
                            "src/cpuid.c"
                            "src/crc.c"
                            "src/debug.c"
                            "src/eprintf.c"
                            "src/filter.c"
                            "src/fp16.c"
                            "src/version.c"
                       INCLUDE_DIRS "interface"
                       # Remove `REQUIRES modules` to break the circular dependency.
                       # The main component should handle all top-level dependencies.
                       REQUIRES modules  )
                       # )