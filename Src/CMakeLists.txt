cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(Src C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()
add_executable(Src
   main.c
)
target_link_libraries(Src pico_stdlib)
target_link_libraries(Src pico_stdlib hardware_adc)
pico_add_extra_outputs(Src)
