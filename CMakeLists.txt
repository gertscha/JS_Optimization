cmake_minimum_required (VERSION 3.19)

# this is the project that contains main() and that generates the executable
project(JS_Optimizer VERSION 1.0)

add_compile_options(/utf-8)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# optinally force lower cpp version for loguru, seems to work with c++20
#set(loguru_USE_CXX14 ON)
# add loguru library
add_library(loguru "loguru/loguru.hpp" "loguru/loguru.cpp")

# all the project source code is here
add_subdirectory(JS_Optimizer)

# make JS_Optimizer the entry point for the soluton
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT JS_Optimizer)
