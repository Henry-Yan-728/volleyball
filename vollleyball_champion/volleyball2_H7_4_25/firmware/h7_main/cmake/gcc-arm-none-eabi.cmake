set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_C_COMPILER_WORKS TRUE CACHE INTERNAL "")
set(CMAKE_ASM_COMPILER_WORKS TRUE CACHE INTERNAL "")
set(CMAKE_C_ABI_COMPILED TRUE CACHE INTERNAL "")
set(CMAKE_C_SIZEOF_DATA_PTR 4 CACHE INTERNAL "")
set(CMAKE_C_COMPILER_ABI ELF CACHE INTERNAL "")
set(CMAKE_C_BYTE_ORDER LITTLE_ENDIAN CACHE INTERNAL "")

set(TARGET_FLAGS "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard")

set(_default_toolchain_bin_dir "")
if(EXISTS "D:/robocon/DevEnv/DevEnv/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe")
    set(_default_toolchain_bin_dir "D:/robocon/DevEnv/DevEnv/GNU-tools-for-STM32/bin")
endif()

set(
    STM32_GNU_TOOLS_BIN_DIR
    "${_default_toolchain_bin_dir}"
    CACHE PATH
    "GNU Arm Embedded toolchain bin directory"
)

if(CMAKE_HOST_WIN32)
    set(_tool_exe_suffix ".exe")
else()
    set(_tool_exe_suffix "")
endif()

if(STM32_GNU_TOOLS_BIN_DIR)
    file(TO_CMAKE_PATH "${STM32_GNU_TOOLS_BIN_DIR}" STM32_GNU_TOOLS_BIN_DIR)
    set(CMAKE_C_COMPILER "${STM32_GNU_TOOLS_BIN_DIR}/arm-none-eabi-gcc${_tool_exe_suffix}")
    set(CMAKE_ASM_COMPILER "${STM32_GNU_TOOLS_BIN_DIR}/arm-none-eabi-gcc${_tool_exe_suffix}")
    set(CMAKE_OBJCOPY "${STM32_GNU_TOOLS_BIN_DIR}/arm-none-eabi-objcopy${_tool_exe_suffix}")
    set(CMAKE_SIZE "${STM32_GNU_TOOLS_BIN_DIR}/arm-none-eabi-size${_tool_exe_suffix}")
    set(CMAKE_AR "${STM32_GNU_TOOLS_BIN_DIR}/arm-none-eabi-ar${_tool_exe_suffix}")
    set(CMAKE_RANLIB "${STM32_GNU_TOOLS_BIN_DIR}/arm-none-eabi-ranlib${_tool_exe_suffix}")
else()
    set(CMAKE_C_COMPILER "arm-none-eabi-gcc${_tool_exe_suffix}")
    set(CMAKE_ASM_COMPILER "arm-none-eabi-gcc${_tool_exe_suffix}")
    set(CMAKE_OBJCOPY "arm-none-eabi-objcopy${_tool_exe_suffix}")
    set(CMAKE_SIZE "arm-none-eabi-size${_tool_exe_suffix}")
    set(CMAKE_AR "arm-none-eabi-ar${_tool_exe_suffix}")
    set(CMAKE_RANLIB "arm-none-eabi-ranlib${_tool_exe_suffix}")
endif()

set(CMAKE_C_FLAGS_INIT "${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS_INIT "${TARGET_FLAGS} -x assembler-with-cpp")

unset(_default_toolchain_bin_dir)
unset(_tool_exe_suffix)
