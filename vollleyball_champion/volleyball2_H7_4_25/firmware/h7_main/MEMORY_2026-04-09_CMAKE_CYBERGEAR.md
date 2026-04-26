# 2026-04-09 工作记忆

## 这次任务

1. 完成使用 CMake 进行编译和烧录，完善 CMake 相关文件。
2. 打通 `cybergear_MOTOR` 电机的云台控制程序链路。

## 今天已经确认的事实

### 工具链与环境

- 本机 `cmake` 不在 `PATH`，但可用：
  - `C:\Users\Lenovo\AppData\Local\stm32cube\bundles\cmake\4.0.1+st.3\bin\cmake.exe`
  - `C:\Users\Lenovo\.vcpkg\artifacts\2139c4c6\tools.kitware.cmake\3.31.5\bin\cmake.exe`
- 本机 `ninja` 可用：
  - `C:\Users\Lenovo\AppData\Local\stm32cube\bundles\ninja\1.13.1+st.1\bin\ninja.exe`
  - `C:\Users\Lenovo\.vcpkg\artifacts\2139c4c6\tools.ninja.build.ninja\1.13.2\ninja.exe`
- GNU Arm 工具链可用：
  - `D:\robocon\DevEnv\DevEnv\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe`
- OpenOCD 可用：
  - `D:\robocon\DevEnv\DevEnv\openocd-v0.12.0-i686-w64-mingw32\bin\openocd.exe`
  - scripts 目录：
    `D:\robocon\DevEnv\DevEnv\openocd-v0.12.0-i686-w64-mingw32\share\openocd\scripts`

### 当前 CMake 状态

- 根目录已有这些文件：
  - `CMakeLists.txt`
  - `CMakePresets.json`
  - `cmake/gcc-arm-none-eabi.cmake`
  - `cmake/stm32cubemx/CMakeLists.txt`
- 当前 `CMakePresets.json` 里的 `build/Debug` 缓存是旧工程路径拷贝过来的，直接 `cmake --preset Debug` 会报 source/binary 路径不匹配。
- 用全新目录 `build/codex-debug` 手动 `configure` 是成功的。
- `configure` 时识别到了 `arm-none-eabi-gcc 13.3.1`。

### 当前构建现象

- 使用 `cmake` 生成 `build/codex-debug` 后，`ninja` 构建行为异常，不是立即报错，而是表现得非常不稳定：
  - `STM32_Drivers` 的对象文件能够生成一部分。
  - `FreeRTOS`/应用层对象文件没有完整落盘。
  - 直接 shell 中跑 `ninja` 常常超时且无明显输出。
- 但手工执行 `compile_commands.json` 里的单条 `arm-none-eabi-gcc` 编译命令是可以成功的，包括：
  - `Middlewares/Third_Party/FreeRTOS/Source/croutine.c`
  - `Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c`
  - `Core/Src/main.c`
  - `control/chassis_cybergear.c`
- 因此现在更像是“CMake/Ninja 组织方式或缓存问题”，不太像源码本身普遍编不过。

### 烧录配置

- `stm32h7x_daplink.cfg` 内容正常，使用的是：
  - `interface/cmsis-dap.cfg`
  - `target/stm32h7x.cfg`
- 还没真正执行烧录。

## cybergear 云台链路现状

### 已存在的调用链

- 上位机 CAN 输入：
  - `can/robot_data.c`
  - `CAN_ID_PC_PAN_TILT (0x200)`
  - 回调 `Callback_PC_Pan_Tilt_Handler()`
  - 会更新两个全局量：
    - `current_yaw_speed`
    - `current_pitch_target_deg`
- 机械任务循环：
  - `can/mechanism_task.c`
  - `Mechanism_Loop_1ms()` 中调用：
    - `gimbal_set_speed(current_yaw_speed, current_pitch_target_deg);`
- 云台控制封装：
  - `control/Pan_Tilt_control.c`
  - `yaw` 走 DJI：
    - `dji_motor_get_instance(GIMBAL_MOTOR_YAW_ID)`
    - `dji_motor_set_speed(...)`
  - `pitch` 已经往 CyberGear 封装走：
    - `chassis_control(pitch_target_deg);`
- CyberGear pitch 封装：
  - `control/chassis_cybergear.c`
  - `chassis_control()` 会：
    - 角度限幅
    - 角度转弧度
    - `trajectory_planner_set_target(target_rad);`
- 轨迹规划器：
  - `cybergear_motor/trajectory_planner.c`
  - 任务里最终调用：
    - `cybergear_motor_set_position(g_motor, g_current_ramped_pos, MOTOR_INTERNAL_SPD_LIMIT);`

### 真正没打通的地方

- `Core/Src/freertos.c` 里 CyberGear 启动链被注释了：
  - `cybergear_motors_init();` 被注释
  - `trajectory_planner_init();` 被注释
  - `StartDefaultTask()` 中的
    `chassis_cybergear_init(0U)` 启动流程被注释
- 结果是：
  - `gimbal_set_speed()` 虽然会调用 `chassis_control(...)`
  - 但 `s_motor == NULL`，CyberGear pitch 实际不会真正动作

## 明天建议的修改顺序

### 第一部分：先补 CMake

建议重点改以下文件：

1. `CMakeLists.txt`
2. `CMakePresets.json`
3. `cmake/gcc-arm-none-eabi.cmake`
4. `cmake/stm32cubemx/CMakeLists.txt`

建议方向：

- 把 `project(... LANGUAGES C ASM)` 补齐。
- 让工具链文件同时处理：
  - `CMAKE_C_COMPILER`
  - `CMAKE_ASM_COMPILER`
  - `CMAKE_OBJCOPY`
  - `CMAKE_SIZE`
  - `CMAKE_AR`
  - `CMAKE_RANLIB`
- 把编译输出补齐：
  - `.elf`
  - `.bin`
  - `.hex`
- 增加 `flash` 目标，调用 OpenOCD 烧录。
- `CMakePresets.json` 的 `binaryDir` 改到一个新的稳定目录，避开旧 `build/Debug` 脏缓存。
- 最好再加 `workflowPresets`，例如：
  - configure
  - build
  - flash
- `cmake/stm32cubemx/CMakeLists.txt` 里当前启动文件是用 `custom_command` 手工拼进去的，后续可以考虑改成直接 ASM 源参与目标构建，减少复杂度。

### 第二部分：打通 CyberGear 云台链路

优先改：

1. `Core/Src/freertos.c`
2. 如有必要，再补 `control/Pan_Tilt_control.c`
3. 如有必要，再补 `control/chassis_cybergear.c`

建议动作：

- 在 `MX_FREERTOS_Init()` 中恢复并整理：
  - `cybergear_motors_init();`
  - `trajectory_planner_init();`
  - 根据返回值设置 `s_cybergear_boot_ok`
- 在 `StartDefaultTask()` 中恢复一次性初始化：
  - `if ((s_cybergear_boot_ok == 1U) && (s_cybergear_chassis_ready == 0U))`
  - `chassis_cybergear_init(0U)`
  - 成功后置位 `s_cybergear_chassis_ready`
- 保持时序：
  - 先 `fdcan_bsp_init()`
  - 再注册 CyberGear 分发
  - 再 `trajectory_planner_init()`
  - 再 `fdcan_bsp_start(...)`
  - 调度器起来后再 `chassis_cybergear_init(...)`
- 视需要增加周期性角度请求：
  - `chassis_request_angle_feedback()`
  - 让 `pitch` 反馈更稳定，`gimbal_get_angles()` 也更可靠

## 特别注意

- `cybergear_motor.h`、`Pan_Tilt_control.h`、`chassis_cybergear.h` 里注释有明显编码问题，但目前不影响我手动单文件编译；后续如果要顺手清理，尽量只修注释编码，不改接口语义。
- `cybergear_motors_init()` 现在把唯一电机配置成：
  - index `0`
  - CAN ID `11`
  - `hfdcan1`
- 这个映射先不要随便改，除非硬件接线确认不是 `FDCAN1 / ID11`。

## 今天没有正式提交的源码修改

- 今天主要做了结构梳理、CMake/工具链定位、构建现象验证、以及 CyberGear 控制链排查。
- 还没有对业务源码做正式功能性提交。
- 当前仓库本身是一个很脏的 worktree，`git status` 涉及很多其他工程，不要误清理。

## 2026-04-12 继续推进记录（进行中）

### 新确认的事实

- 旧的 `Debug` preset 仍然直指 `build/Debug`，这里的缓存来自更早的工程路径；直接 `cmake --preset Debug` 依旧会报 source/binary 路径不匹配。
- 手动在 `build/codex-debug` 下重新 `configure` 后，`ninja -n -v` 可以完整展开 `72` 条构建命令，说明当前工程的构建图本身是完整的。
- 之前 shell 里直接跑 `ninja` 看起来像“卡住”，现在更像是完整编译耗时长、默认输出不明显，而不是构建系统已经坏掉。

### 本轮已落地的修改

- `CMakeLists.txt`
  - `project()` 已补到 `LANGUAGES C ASM`
  - 增加 `.bin`、`.hex` 产物生成
  - 增加 `flash` 目标，调用 OpenOCD 烧录 `.elf`
- `CMakePresets.json`
  - preset 输出目录改到新的稳定路径，避开旧 `build/Debug`
  - 增加 `workflowPresets`
- `cmake/gcc-arm-none-eabi.cmake`
  - 已补齐 `CMAKE_ASM_COMPILER`
  - 已补齐 `CMAKE_OBJCOPY`
  - 已补齐 `CMAKE_SIZE`
  - 已补齐 `CMAKE_AR`
  - 已补齐 `CMAKE_RANLIB`
- `cmake/stm32cubemx/CMakeLists.txt`
  - 启动文件改为直接参与 `ASM` 构建，不再走手工 `custom_command` 拼装对象文件
- `Core/Src/freertos.c`
  - 已恢复 `cybergear_motors_init();`
  - 已恢复 `trajectory_planner_init();`
  - 已恢复 `StartDefaultTask()` 中的一次性 `chassis_cybergear_init(0U)`
  - 已增加周期性 `chassis_request_angle_feedback()`

### 下一步

- 用新的 preset 重新 `configure + build`
- 观察是否还有链接或汇编阶段问题
- 若构建通过，再把本次结果和剩余风险补充回本文件

## 2026-04-12 CAN 分发统一化（进行中）

### 目标

- 不再让 `robot_data`、`DJI motor`、`CyberGear` 各自在自己的 `*_init()` 里分散做注册。
- 统一改为由 `fdcan_bsp_register_all_dispatches()` 作为单一入口加载全部接收分发。

### 已完成

- `robot_data`
  - `Robot_Data_Init()` 已改成只负责状态清零
  - 新增 `Robot_Data_Register_Dispatches()` 负责注册 `pose/pc target/pan tilt`
- `dji_motor`
  - `dji_motors_init()` 保留实例配置
  - 新增 `dji_motors_register_dispatches()` 统一注册全部已配置电机反馈 ID
- `cybergear_motor`
  - `cybergear_motors_init()` 保留实例配置和锁初始化
  - 新增 `cybergear_motors_register_dispatches()` 统一注册扩展帧反馈规则
- `fdcan_bsp`
  - 新增 `fdcan_bsp_register_all_dispatches()`
  - 由 `fdcan_bsp` 统一调度加载上述三类注册
- `Core/Src/freertos.c`
  - 已在 `cybergear_motors_init()` 之后、`fdcan_bsp_start(...)` 之前插入统一注册入口

### 待确认

- 需要重新编译确认头文件依赖和新注册入口没有遗漏
- 需要确认 `FDCAN1/FDCAN2/FDCAN3` 的接收链路都仍然能匹配到原有回调

### 当前验证结果

- 已对以下关键文件执行 `arm-none-eabi-gcc -fsyntax-only` 语法校验，全部通过：
  - `can/robot_data.c`
  - `can/dji_motor.c`
  - `cybergear_motor/cybergear_motor.c`
  - `can/fdcan_bsp.c`
  - `Core/Src/freertos.c`
- 目前未发现统一注册入口带来的明显编译错误。
- 整包 `ninja` 仍然表现为长时间静默运行，当前更适合先用单文件语法校验和后续实机联调来确认分发行为。
