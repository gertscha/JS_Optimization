# all the sources of the project are placed in a library
add_library(JS_Lib OBJECT "")

add_subdirectory(problem)
add_subdirectory(optimizer_base)
add_subdirectory(optimizers)
add_subdirectory(utility)

# add to include to make include directives in the code shorter
target_include_directories(JS_Lib PUBLIC ${CMAKE_CURRENT_LIST_DIR}/problem)
target_include_directories(JS_Lib PUBLIC ${CMAKE_CURRENT_LIST_DIR}/optimizer_base)
target_include_directories(JS_Lib PUBLIC ${CMAKE_CURRENT_LIST_DIR}/optimizers)
target_include_directories(JS_Lib PUBLIC ${CMAKE_CURRENT_LIST_DIR}/utility)

# define the executable
add_executable(JS_Optimizer main.cpp)

# add to include for the executable
target_link_libraries(JS_Optimizer loguru)
target_link_libraries(JS_Optimizer JS_Lib)

# add loguru to include path
target_include_directories(JS_Optimizer PUBLIC ${CMAKE_SOURCE_DIR}/loguru)
target_include_directories(JS_Lib PUBLIC ${CMAKE_SOURCE_DIR}/loguru)

# add python
find_package(Python REQUIRED COMPONENTS Interpreter Development)

target_include_directories(JS_Lib PRIVATE ${Python_INCLUDE_DIRS})

target_link_libraries(JS_Lib PRIVATE ${Python_LIBRARIES})
