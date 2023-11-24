# define the names of dependency projects
set(rust_folder CLI_Frontend)
set(dll_name JS_dynlib)
set(dll_output ${CMAKE_SOURCE_DIR}/${rust_folder}/target/)

# create a new project for the dll
add_library(${dll_name} SHARED "JS_dynlib.h" "JS_dynlib.cpp")

# link to JS_Lib and Loguru
target_link_libraries(${dll_name} JS_Lib)
target_link_libraries(${dll_name} loguru)

# provide options to build the DLL for the Rust application only or to copy them
# from inside the CMake build folder or do neither
# this only works for the Debug and Release configuration
option(COPY_DLL_FOR_RUST "Copy the DLL to the Rust application" OFF)
option(OUTPUT_DLL_ONLY_FOR_RUST "Only output the DLL to the Rust application" ON)

if(OUTPUT_DLL_ONLY_FOR_RUST)
  # set the output paths into the Cargo build folders
  message(STATUS "INFO: ${dll_name} output only to the ${rust_folder} application")
  set_target_properties(${dll_name} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${dll_output}/debug
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${dll_output}/debug
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${dll_output}/release
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${dll_output}/release
  )
elseif(COPY_DLL_FOR_RUST)
  message(STATUS "INFO: ${dll_name} will be copied from the CMake build to ${rust_folder}")
  # copy the output of the build into the Cargo build folders
  add_custom_command(TARGET ${dll_name} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
            ${dll_output}/"$<IF:$<CONFIG:Debug>,debug,$<IF:$<CONFIG:Release>,release,OtherCMakeConfig>>"
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_BINARY_DIR}/"$<CONFIG>"/${dll_name}.dll
            ${dll_output}/"$<IF:$<CONFIG:Debug>,debug,$<IF:$<CONFIG:Release>,release,OtherCMakeConfig>>"/${dll_name}.dll
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_BINARY_DIR}/"$<CONFIG>"/${dll_name}.lib
            ${dll_output}/"$<IF:$<CONFIG:Debug>,debug,$<IF:$<CONFIG:Release>,release,OtherCMakeConfig>>"/${dll_name}.lib
    COMMENT "Copying ${dll_name}.dll and .lib to ${rust_folder} directory"
  )
else()
  message(STATUS "INFO: Output for ${dll_name} is not available in ${rust_folder}")
endif()
