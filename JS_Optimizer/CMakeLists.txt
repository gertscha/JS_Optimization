# define the executable
add_executable(JS_Optimizer main.cpp)

# link the libraries for the executable, i.e. main.cpp
target_link_libraries(JS_Optimizer loguru)
target_link_libraries(JS_Optimizer JS_Lib)

target_include_directories(JS_Optimizer PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
