# groups.cmake

# group Application/MDK-ARM
add_library(Group_Application_MDK-ARM OBJECT
  "${SOLUTION_ROOT}/startup_stm32g474xx.s"
)
target_include_directories(Group_Application_MDK-ARM PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Application_MDK-ARM PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Application_MDK-ARM_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Application_MDK-ARM_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Application_MDK-ARM PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Application_MDK-ARM PUBLIC
  Group_Application_MDK-ARM_ABSTRACTIONS
)
set(COMPILE_DEFINITIONS
  __MICROLIB
  STM32G474xx
  _RTE_
)
cbuild_set_defines(AS_ARM COMPILE_DEFINITIONS)
set_source_files_properties("${SOLUTION_ROOT}/startup_stm32g474xx.s" PROPERTIES
  COMPILE_FLAGS "${COMPILE_DEFINITIONS}"
)
set_source_files_properties("${SOLUTION_ROOT}/startup_stm32g474xx.s" PROPERTIES
  COMPILE_OPTIONS "-masm=auto;-x assembler"
)

# group Application/User/Core
add_library(Group_Application_User_Core OBJECT
  "${SOLUTION_ROOT}/../Core/Src/main.c"
  "${SOLUTION_ROOT}/../Core/Src/gpio.c"
  "${SOLUTION_ROOT}/../Core/Src/dma.c"
  "${SOLUTION_ROOT}/../Core/Src/fdcan.c"
  "${SOLUTION_ROOT}/../Core/Src/usart.c"
  "${SOLUTION_ROOT}/../Core/Src/stm32g4xx_it.c"
  "${SOLUTION_ROOT}/../Core/Src/stm32g4xx_hal_msp.c"
)
target_include_directories(Group_Application_User_Core PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Application_User_Core PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Application_User_Core_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Application_User_Core_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Application_User_Core PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Application_User_Core PUBLIC
  Group_Application_User_Core_ABSTRACTIONS
)

# group Drivers/STM32G4xx_HAL_Driver
add_library(Group_Drivers_STM32G4xx_HAL_Driver OBJECT
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c"
)
target_include_directories(Group_Drivers_STM32G4xx_HAL_Driver PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Drivers_STM32G4xx_HAL_Driver PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Drivers_STM32G4xx_HAL_Driver_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Drivers_STM32G4xx_HAL_Driver_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Drivers_STM32G4xx_HAL_Driver PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Drivers_STM32G4xx_HAL_Driver PUBLIC
  Group_Drivers_STM32G4xx_HAL_Driver_ABSTRACTIONS
)

# group Drivers/CMSIS
add_library(Group_Drivers_CMSIS OBJECT
  "${SOLUTION_ROOT}/../Core/Src/system_stm32g4xx.c"
)
target_include_directories(Group_Drivers_CMSIS PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Drivers_CMSIS PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Drivers_CMSIS_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Drivers_CMSIS_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Drivers_CMSIS PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Drivers_CMSIS PUBLIC
  Group_Drivers_CMSIS_ABSTRACTIONS
)

# group motor
add_library(Group_motor OBJECT
  "${SOLUTION_ROOT}/../Motor_control/vesc.c"
  "${SOLUTION_ROOT}/../Motor_control/vesc_rx.c"
  "${SOLUTION_ROOT}/../Motor_control/pid.c"
  "${SOLUTION_ROOT}/../Motor_control/dji_motor.c"
)
target_include_directories(Group_motor PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  "${SOLUTION_ROOT}/../Motor_control"
)
target_compile_definitions(Group_motor PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_motor_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_motor_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_motor PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_motor PUBLIC
  Group_motor_ABSTRACTIONS
)

# group chassis_task
add_library(Group_chassis_task OBJECT
  "${SOLUTION_ROOT}/../chassis_task/chassis_task.c"
)
target_include_directories(Group_chassis_task PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  "${SOLUTION_ROOT}/../chassis_task"
)
target_compile_definitions(Group_chassis_task PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_chassis_task_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_chassis_task_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_chassis_task PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_chassis_task PUBLIC
  Group_chassis_task_ABSTRACTIONS
)

# group fdcan_bsp
add_library(Group_fdcan_bsp OBJECT
  "${SOLUTION_ROOT}/../BSP/fdcan_bsp.c"
)
target_include_directories(Group_fdcan_bsp PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  "${SOLUTION_ROOT}/../BSP"
)
target_compile_definitions(Group_fdcan_bsp PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_fdcan_bsp_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_fdcan_bsp_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_fdcan_bsp PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_fdcan_bsp PUBLIC
  Group_fdcan_bsp_ABSTRACTIONS
)

# group control
add_library(Group_control OBJECT
  "${SOLUTION_ROOT}/../control/control.c"
)
target_include_directories(Group_control PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  "${SOLUTION_ROOT}/../control"
)
target_compile_definitions(Group_control PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_control_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_control_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_control PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_control PUBLIC
  Group_control_ABSTRACTIONS
)
