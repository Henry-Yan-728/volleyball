# YAOKONGQIPING.HMI UI Optimization

目标工程：

```text
D:\下载\YAOKONGQIPING.HMI
```

编辑软件：

```text
E:\USARTHMI\USART HMI.exe
```

我已按这个工程给 H7 固件做了控制层适配。下面是建议在 HMI 软件里落地的优化版页面配置，保持原来的坐标输入思路，但把机器人状态、目标点和动作反馈做成更像比赛现场调试屏的界面。

## 页面结构

页面名保持：

```text
main
```

布局建议按 800x480 横屏设计；如果你的屏幕不是这个分辨率，保持相对区域即可。

```text
顶部栏      队名/机器人名 + 模式 + 系统状态
左侧大区    当前坐标、速度、目标点
右侧大区    启动、结束、发送目标点、快速目标点
底部栏      遥控输入、运行秒数、消息提示
```

## 控件命名

固件会自动刷新的文本控件：

```text
t_mode      当前模式：STANDBY/AUTO/MANUAL/SERVE
t_state     系统状态：START/OVER/START_BUSY/OVER_BUSY/ERR_SENSOR/ERR_MOTOR
t_pose      当前位姿：X/Y/A
t_vel       当前速度：Vx/Vy/Vr
t_target    自动目标点：X/Y
t_rc        遥控输入摘要
t_tick      运行秒数
t_msg       屏幕命令回显或错误提示
```

已补充一份可直接照着配置的控件表：

```text
E:\2026volleyball\volleyball\vollleyball_champion\volleyball2_H7_4_25\firmware\h7_main\hmi\YAOKONGQIPING_H7_OPTIMIZED_CONTROLS.csv
```

并已复制出待编辑副本：

```text
D:\下载\YAOKONGQIPING_H7_OPTIMIZED.HMI
```

坐标输入框沿用 demo 风格：

```text
t2          目标 X
t3          目标 Y
```

按钮建议：

```text
b_start     开始
b_end       结束
b_send      发送目标点
b_home      目标点 0,0
b_front     目标点 800,500
b_back      目标点 1000,1000
```

## 视觉建议

背景用深灰或黑色，不用大面积纯蓝。比赛现场光线复杂，深底高对比更稳。

推荐颜色：

```text
背景      0       black
主文字    65535   white
正常绿    2016
忙碌黄    65504
故障红    63488
分割线    33808   medium gray
按钮蓝    31
```

顶部栏高度约 56 px，`t_mode` 和 `t_state` 放右侧，字号比普通数据大一级。`t_state` 的 `pco` 由固件自动改变颜色。

左侧数据区域做成三行：

```text
t_pose
t_vel
t_target
```

右侧按钮使用 2 列网格，按钮高度不小于 54 px，避免现场误触。`b_start` 用绿色边框，`b_end` 用红色边框，`b_send` 用蓝色。

底部消息栏放：

```text
t_rc
t_tick
t_msg
```

`t_msg` 建议放在最右侧，颜色由固件按回显内容设置。

## 触摸事件代码

`b_start` 弹起事件：

```text
prints "start",0
printh 0D 0A
```

`b_end` 弹起事件：

```text
prints "end",0
printh 0D 0A
```

`b_send` 弹起事件：

```text
prints "X:",0
prints t2.txt,0
prints ",Y:",0
prints t3.txt,0
printh 0D 0A
```

`b_home` 弹起事件：

```text
prints "X:0,Y:0",0
printh 0D 0A
```

`b_front` 弹起事件：

```text
prints "X:800,Y:500",0
printh 0D 0A
```

`b_back` 弹起事件：

```text
prints "X:1000,Y:1000",0
printh 0D 0A
```

## 通信约定

屏幕发给 H7：

```text
start\r\n
end\r\n
X:800,Y:500\r\n
```

H7 发给屏幕：

```text
t_mode.txt="AUTO" FF FF FF
t_state.txt="START" FF FF FF
t_state.pco=2016 FF FF FF
```

固件已经做了差分刷新：只有显示内容变化时才发送，减少 UART4 阻塞和屏幕刷新压力。

## 保存建议

在 HMI 软件里完成上述控件和事件后，建议另存为：

```text
D:\下载\YAOKONGQIPING_H7_OPTIMIZED.HMI
```

这样可以保留原始 `YAOKONGQIPING.HMI`，比赛前回退也方便。
