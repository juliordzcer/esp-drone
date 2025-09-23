# components/modules/CMakeLists.txt
idf_component_register(SRCS 
                            "src/commander.c"
                            "src/console.c"
                            "src/controller.c"
                            "src/crtp.c"
                            "src/crtpservice.c"
                            "src/info.c"
                            "src/log.c"
                            "src/param.c"
                            "src/pid.c"
                            "src/pidctrl.c"
                            "src/sensor_fusion.c"
                            "src/stabilizer.c"
                            "src/system.c"
                            "src/worker.c"
                            "src/comm.c" # <-- This was missing
                         #    "src/version.c" # <-- This was missing, from utils
                       INCLUDE_DIRS "interface" "src"
                       REQUIRES bsp drivers config utils)