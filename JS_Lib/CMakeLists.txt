# all the sources of the project are placed in a library
add_library(JS_Lib STATIC "")

add_subdirectory(include)
add_subdirectory(problem_base)
add_subdirectory(optimizer_base)
add_subdirectory(optimizers)
add_subdirectory(utility)

# add to include to make include directives in the code shorter
target_include_directories(JS_Lib PUBLIC
    ${CMAKE_SOURCE_DIR}/loguru
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/problem_base
    ${CMAKE_CURRENT_LIST_DIR}/optimizer_base
    ${CMAKE_CURRENT_LIST_DIR}/optimizers
    ${CMAKE_CURRENT_LIST_DIR}/utility
)

# create filters for Visual Studio solution
set(JS_Lib_Path "${PROJECT_SOURCE_DIR}/JS_Lib")
file(GLOB_RECURSE JS_Lib_header_files "${JS_Lib_Path}/*.h")
file(GLOB_RECURSE JS_Lib_source_files "${JS_Lib_Path}/*.cpp")
source_group(TREE "${JS_Lib_Path}/" PREFIX "Header Files" FILES ${JS_Lib_header_files})
source_group(TREE "${JS_Lib_Path}/" PREFIX "Source Files" FILES ${JS_Lib_source_files})
