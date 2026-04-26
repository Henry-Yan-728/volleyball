# volleyball2_H7_4_6 替换评估与提交说明

## 当前基线

- 仓库根目录：`E:/2026volleyball/volleyball`
- 当前工作目录：`vollleyball_champion/volleyball2_H7_4_20`
- 基线提交：`e764ce1 checkpoint volleyball H7 replacement sources`
- 本次纳入版本控制的范围：
  - `volleyball2_H7_4_6/Core/Inc`
  - `volleyball2_H7_4_6/Core/Src`
  - `volleyball2_H7_4_6/can`
  - `volleyball2_H7_4_6/control`
  - `volleyball2_H7_4_6/cybergear_motor`
  - `volleyball2_H7_4_6/unitree_motor`
  - `volleyball2_H7_4_6/volleyball2_H7.ioc`
  - `big-success-2026-4-21` 中用于对照的 `Core/Inc`、`Core/Src`、`control`、`cybergear_motor`、`planner`、`ramp`、`.ioc`

## 暂不纳入的内容

以下内容暂时不作为后续小步提交对象，除非明确需要修改：

- `Drivers/`
- `Middlewares/`
- `build/`
- `out/`
- `MDK-ARM/` 中的编译输出
- `.idea/`、`.vscode/`
- 其他与本次 H7 替换评估无关的工程目录

这样做是为了避免一次提交混入大量生成文件和无关历史改动，方便回退和审查。

## 后续提交规则

每次修改程序时按以下顺序执行：

1. 先执行 `git status --short`，确认当前改动范围。
2. 只修改本次目标相关文件。
3. 修改后检查 `git diff -- <file>`。
4. 只 `git add` 本次修改过的目标文件。
5. 使用清晰提交信息提交，例如：
   - `tune H7 DJI parameters from big-success`
   - `align H7 planner angle unit handling`
   - `document H7 replacement risks`
6. 提交后再执行 `git status --short -- <target paths>`，确认本次修改已经保存。

## 替换评估结论

`volleyball2_H7_4_6` 不能整包直接替换 `big-success-2026-4-21`，因为两者 MCU 平台不同：

- `volleyball2_H7_4_6` 是 STM32H7 工程。
- `big-success-2026-4-21` 是 STM32G4 工程。

因此应保留 H7 的外设初始化、启动文件、中断文件、CubeMX 生成框架，只把 `big-success-2026-4-21` 中确认可复用的控制参数和协议行为迁移到 H7 工程。

## 本轮明确范围

- 宇树电机相关接口和实现暂不处理。
- 优先检查和迁移非宇树部分：
  - DJI 电机参数
  - 云台控制参数
  - 底盘控制参数
  - 定位和上位机 CAN 协议
  - 遥控输入和任务调度逻辑
  - 光电门传感器逻辑

## 已知注意点

- 遥控串口不同：`big-success-2026-4-21` 使用 `USART2`，H7 工程使用 `USART10`。
- 光电门引脚不同：`big-success-2026-4-21` 使用 `GPIOB PIN11`，H7 工程使用 `PHOTO_GATE = GPIOE PIN14`。
- 定位角度单位需要确认：`big-success-2026-4-21` 更像按弧度使用，H7 当前逻辑中存在 `cur_yaw *= DEG_TO_RAD`，如果定位板仍发弧度，这里需要修正。
- 自动规划速度单位需要确认：`big-success-2026-4-21` 有 `MM_TO_M` 缩放，H7 当前路径规划调用没有完全相同的单位处理，不能直接照搬大速度参数。
