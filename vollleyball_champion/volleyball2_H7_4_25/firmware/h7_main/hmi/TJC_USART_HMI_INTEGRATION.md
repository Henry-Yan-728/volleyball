# TJC USART HMI Integration

本项目对接 `D:\下载\YAOKONGQIPING.HMI`，参考 `E:\2026volleyball\volleyball\vollleyball_champion\TJC_usart_demo` 的通信方式，并贴合 `h7_main` 当前的控制逻辑。

## 串口选择

- 串口屏连接到 `UART4`。
- H7 引脚：`PD1/UART4_TX -> 屏幕 RX`，`PD0/UART4_RX <- 屏幕 TX`，共地。
- 波特率：`115200, 8N1`。
- 保留 `USART10` 给遥控器/调参帧，保留 `USART1` 给 `printf` 调试。

## 固件功能

已新增 `can/hmi_panel.c` 和 `can/hmi_panel.h`：

- 屏幕发 `start\r\n`：固件调用 `Set_System_Doit_State(START)`。
- 屏幕发 `end\r\n`：固件调用 `Set_System_Doit_State(OVER)`。
- 屏幕发 `X:800,Y:500\r\n`：固件写入 `Robot_Data_SetTarget(800, 500)`，自动模式会读到新的目标点。
- 固件每 `200 ms` 主动刷新屏幕状态控件。

## HMI 页面要求

在 `E:\USARTHMI\USART HMI.exe` 中打开：

```text
D:\下载\YAOKONGQIPING.HMI
```

建议保留或创建页面名：

```text
main
```

在 `main` 页面放置这些文本控件，名称必须一致：

```text
t_mode
t_state
t_pose
t_vel
t_target
t_rc
t_tick
t_msg
```

固件会发送类似以下指令刷新它们：

```text
t_mode.txt="AUTO" + FF FF FF
t_state.txt="START" + FF FF FF
t_state.pco=2016 + FF FF FF
t_pose.txt="X:800 Y:500 A:0" + FF FF FF
t_msg.txt="Target updated" + FF FF FF
```

`t_state.pco` 会按状态改变颜色：

- `2016`：正常
- `65504`：动作中
- `63488`：故障

## 屏幕按钮事件

开始按钮的“弹起事件”：

```text
prints "start",0
printh 0D 0A
```

结束按钮的“弹起事件”：

```text
prints "end",0
printh 0D 0A
```

目标点发送按钮的“弹起事件”，假设坐标输入框为 `t2` 和 `t3`：

```text
prints "X:",0
prints t2.txt,0
prints ",Y:",0
prints t3.txt,0
printh 0D 0A
```

这与 demo 中解析的格式一致：`X:12.34,Y:56.78\r\n`。

## 参考

- TJC 指令集：`https://wiki.tjc1688.com/commands/index.html`
- 文本控件：`https://wiki.tjc1688.com/widgets/Text.html`
- 数字控件：`https://wiki.tjc1688.com/widgets/Number.html`
