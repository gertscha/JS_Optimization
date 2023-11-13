cmake_minimum_required (VERSION 3.19)

# this is the project that contains main() and that generates the executable
project(JS_Optimization VERSION 1.1)

add_compile_options(/utf-8)

message(STATUS "Setting CXX_STANDARD 20")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# optinally force lower cpp version for loguru, might forestall some warnings
option(LOGURU_BUILD_USING_CPP14 "force compilation with CXX14 for loguru" OFF)
if (LOGURU_BUILD_USING_CPP14)
    message(STATUS "Compiling loguru using CXX14")
    set(loguru_USE_CXX14 ON)
else()
    message(STATUS "Compiling loguru using CXX_STANDARD 20")
endif()

# add loguru library
add_library(loguru "loguru/loguru.hpp" "loguru/loguru.cpp")

# the project source
add_subdirectory(JS_Lib)
add_subdirectory(JS_Optimizer)

# make JS_Optimizer the entry point for the soluton
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT JS_Optimizer)
