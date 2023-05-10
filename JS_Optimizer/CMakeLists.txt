# all the sources of the project are placed in a library
add_library(JS_Lib OBJECT "")

add_subdirectory(include)
add_subdirectory(problem)
add_subdirectory(optimizer_base)
add_subdirectory(optimizers)
add_subdirectory(utility)

# add to include to make include directives in the code shorter
target_include_directories(JS_Lib PUBLIC
    ${CMAKE_SOURCE_DIR}/loguru
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/problem
    ${CMAKE_CURRENT_LIST_DIR}/optimizer_base
    ${CMAKE_CURRENT_LIST_DIR}/optimizers
    ${CMAKE_CURRENT_LIST_DIR}/utility
)

# set filters for Visual Studio solution
set(JS_Lib_Path "${PROJECT_SOURCE_DIR}/JS_Optimizer")
file(GLOB_RECURSE JS_Lib_header_files "${JS_Lib_Path}/*.h")
file(GLOB_RECURSE JS_Lib_source_files "${JS_Lib_Path}/*.cpp")
source_group(TREE "${JS_Lib_Path}/" PREFIX "Header Files" FILES ${JS_Lib_header_files})
source_group(TREE "${JS_Lib_Path}/" PREFIX "Source Files" FILES ${JS_Lib_source_files})

# define the executable
add_executable(JS_Optimizer main.cpp)

# link the libraries for the executable, i.e. main.cpp
target_link_libraries(JS_Optimizer loguru)
target_link_libraries(JS_Optimizer JS_Lib)

# add python to JS_Lib
find_package (Python REQUIRED COMPONENTS Interpreter Development)
target_include_directories(JS_Lib PRIVATE ${Python_INCLUDE_DIRS})
target_link_libraries(JS_Lib PRIVATE ${Python_LIBRARIES})

# create a #define in config.h with the absolute path
set(SOLUTION_DIR_STRING "${CMAKE_SOURCE_DIR}")
configure_file(config.h.in config.h @ONLY)
target_include_directories(JS_Optimizer PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# pre-compiled header (PCH) support
option(JS_Lib_BUILD_USING_PCH "Build using pre-compile header support" ON)
if (CMAKE_VERSION VERSION_GREATER_EQUAL 3.16 AND KDV_BUILD_USING_PCH) # support for PCHs in CMake was added in 3.16
    message(STATUS "Compiling using pre-compiled header support")
    target_precompile_headers(JS_Lib PUBLIC JS_Lib_pch.h) # With PUBLIC they will be used by targets using this target
    set(CMAKE_PCH_INSTANTIATE_TEMPLATES ON) # very benefital if available, should be no-op if it is not
else()
    message(STATUS "Disabled pre-compiled header support")
endif()