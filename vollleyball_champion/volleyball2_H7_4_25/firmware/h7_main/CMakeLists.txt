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

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(volleyball2_H7 LANGUAGES C ASM)
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(H7_PROJECT_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(H7_SHARED_DIR ${H7_PROJECT_ROOT}/../../shared)
set(H7_LINKER_SCRIPT ${H7_PROJECT_ROOT}/STM32H723XG_FLASH.ld)

set(H7_USER_SOURCES
    ${H7_PROJECT_ROOT}/chassis_local/h7_vesc.c
    ${H7_PROJECT_ROOT}/chassis_local/h7_vesc_rx.c
    ${H7_PROJECT_ROOT}/chassis_local/local_chassis_control.c
    ${H7_PROJECT_ROOT}/chassis_local/local_kinematics.c
    ${H7_PROJECT_ROOT}/chassis_local/local_safety_guardian.c
    ${H7_PROJECT_ROOT}/can/chassis_path_task.c
    ${H7_PROJECT_ROOT}/can/chassis_task.c
    ${H7_PROJECT_ROOT}/can/dji_motor.c
    ${H7_PROJECT_ROOT}/can/fdcan_bsp.c
    ${H7_PROJECT_ROOT}/can/hmi_panel.c
    ${H7_PROJECT_ROOT}/can/mechanism_task.c
    ${H7_PROJECT_ROOT}/can/pid.c
    ${H7_PROJECT_ROOT}/can/planner_tuning_protocol.c
    ${H7_PROJECT_ROOT}/can/remote_driver.c
    ${H7_PROJECT_ROOT}/can/robot_data.c
    ${H7_PROJECT_ROOT}/can/Task_command.c
    ${H7_PROJECT_ROOT}/control/chassis_cybergear.c
    ${H7_PROJECT_ROOT}/control/Pan_Tilt_control.c
    ${H7_PROJECT_ROOT}/cybergear_motor/cybergear_motor.c
    ${H7_PROJECT_ROOT}/cybergear_motor/cybergear_speed_planner.c
    ${H7_PROJECT_ROOT}/cybergear_motor/trajectory_planner.c
    ${H7_PROJECT_ROOT}/ramp/ramp.c
    ${H7_PROJECT_ROOT}/unitree_motor/crc_ccitt.c
    ${H7_PROJECT_ROOT}/unitree_motor/gom_protocol.c
    ${H7_PROJECT_ROOT}/unitree_motor/unitree_motor.c
)

set(H7_USER_INCLUDE_DIRS
    ${H7_PROJECT_ROOT}/chassis_local
    ${H7_PROJECT_ROOT}/can
    ${H7_PROJECT_ROOT}/control
    ${H7_PROJECT_ROOT}/cybergear_motor
    ${H7_PROJECT_ROOT}/ramp
    ${H7_PROJECT_ROOT}/unitree_motor
    ${H7_SHARED_DIR}
)

set(_default_openocd_executable "")
if(EXISTS "D:/robocon/DevEnv/DevEnv/openocd-v0.12.0-i686-w64-mingw32/bin/openocd.exe")
    set(_default_openocd_executable "D:/robocon/DevEnv/DevEnv/openocd-v0.12.0-i686-w64-mingw32/bin/openocd.exe")
endif()

set(_default_openocd_scripts_dir "")
if(EXISTS "D:/robocon/DevEnv/DevEnv/openocd-v0.12.0-i686-w64-mingw32/share/openocd/scripts")
    set(_default_openocd_scripts_dir "D:/robocon/DevEnv/DevEnv/openocd-v0.12.0-i686-w64-mingw32/share/openocd/scripts")
endif()

set(
    OPENOCD_EXECUTABLE
    "${_default_openocd_executable}"
    CACHE FILEPATH
    "Path to the OpenOCD executable"
)

set(
    OPENOCD_SCRIPTS_DIR
    "${_default_openocd_scripts_dir}"
    CACHE PATH
    "Path to the OpenOCD scripts directory"
)

set(
    OPENOCD_FLASH_CONFIG
    "${CMAKE_SOURCE_DIR}/stm32h7x_daplink.cfg"
    CACHE FILEPATH
    "OpenOCD configuration used for flashing"
)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES SUFFIX ".elf")

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${H7_USER_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${H7_USER_INCLUDE_DIRS}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    USE_FREERTOS
)

if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
        -ffunction-sections
        -fdata-sections
    )
endif()

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        -T${H7_LINKER_SCRIPT}
        -Wl,-Map=${CMAKE_PROJECT_NAME}.map
        -Wl,--gc-sections
        -Wl,--print-memory-usage
    )
    set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
        LINK_DEPENDS ${H7_LINKER_SCRIPT}
    )
endif()

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m

    # Add user defined libraries
)

if(CMAKE_OBJCOPY AND CMAKE_SIZE)
    add_custom_command(
        TARGET ${CMAKE_PROJECT_NAME}
        POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
        COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
        BYPRODUCTS
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
        COMMENT "Generating firmware binary, hex and size report"
    )
endif()

if(OPENOCD_EXECUTABLE AND OPENOCD_SCRIPTS_DIR AND OPENOCD_FLASH_CONFIG)
    add_custom_target(
        flash
        COMMAND ${OPENOCD_EXECUTABLE}
            -s ${OPENOCD_SCRIPTS_DIR}
            -f ${OPENOCD_FLASH_CONFIG}
            -c "program $<TARGET_FILE:${CMAKE_PROJECT_NAME}> verify reset exit"
        DEPENDS ${CMAKE_PROJECT_NAME}
        USES_TERMINAL
        COMMENT "Flashing ${CMAKE_PROJECT_NAME}.elf with OpenOCD"
    )
else()
    add_custom_target(
        flash
        COMMAND ${CMAKE_COMMAND} -E echo "OpenOCD is not configured. Set OPENOCD_EXECUTABLE, OPENOCD_SCRIPTS_DIR and OPENOCD_FLASH_CONFIG."
        COMMAND ${CMAKE_COMMAND} -E false
        DEPENDS ${CMAKE_PROJECT_NAME}
    )
endif()
