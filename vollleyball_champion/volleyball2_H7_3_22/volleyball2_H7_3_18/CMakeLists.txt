cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME volleyball2_H7)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # User sources are discovered from dedicated application module folders.
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # User include directories are appended below.
)

# User application modules (outside STM32CubeMX generated tree)
set(USER_MODULE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/can
    ${CMAKE_CURRENT_SOURCE_DIR}/control
    ${CMAKE_CURRENT_SOURCE_DIR}/planner
    ${CMAKE_CURRENT_SOURCE_DIR}/ramp
    ${CMAKE_CURRENT_SOURCE_DIR}/unitree_motor
    ${CMAKE_CURRENT_SOURCE_DIR}/cybergear_motor
)

# Collect all user-owned sources from module folders.
set(USER_APP_SOURCES)
foreach(module_dir IN LISTS USER_MODULE_DIRS)
    file(GLOB module_sources CONFIGURE_DEPENDS
        ${module_dir}/*.c
        ${module_dir}/*.cpp
        ${module_dir}/*.s
    )
    list(APPEND USER_APP_SOURCES ${module_sources})
endforeach()

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_APP_SOURCES})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${USER_MODULE_DIRS})

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
