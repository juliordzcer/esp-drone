# ~/esp-drone/esp-drone/CMakeLists.txt

# El boilerplate estándar de ESP-IDF
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp-drone)

# --- ELIMINA ESTE BLOQUE COMPLETO ---
# set(CMAKE_EXE_LINKER_FLAGS 
#     "${CMAKE_EXE_LINKER_FLAGS} 
#      -Wl,--whole-archive 
#      ${CMAKE_CURRENT_BINARY_DIR}/esp-idf/modules/libmodules.a 
#      -Wl,--no-whole-archive"
# )
# ------------------------------------