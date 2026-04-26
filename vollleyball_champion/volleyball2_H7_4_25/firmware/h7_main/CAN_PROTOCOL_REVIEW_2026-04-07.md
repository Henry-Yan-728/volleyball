# CAN 通信协议检查建议

日期：2026-04-07

## 检查范围

本文档用于整理本项目中 CAN/FDCAN 相关代码的协议检查结果。

本次检查涉及：

- `Core/Src/fdcan.c`
- `Core/Src/freertos.c`
- `can/fdcan_bsp.c`
- `can/fdcan_bsp.h`
- `can/robot_data.c`
- `can/robot_data.h`
- `can/chassis_task.c`
- `can/chassis_task.h`
- `can/dji_motor.c`
- `can/dji_motor.h`

本次未纳入：

- `unitree_motor/`

原因：

- 该部分使用的是 `RS485/UART`，不是 CAN 总线。

## 总体结论

从当前代码实现来看，没有发现“同一条物理 CAN 总线上的直接 ID 冲突”。

当前总线分工大致如下：

- `FDCAN1`：底盘控制
- `FDCAN2`：DJI 电机和一帧位姿回传
- `FDCAN3`：定位数据与上位机通信

但是，项目中存在多处“协议定义”和“实际实现”不一致的问题。其中有些属于文档、注释、头文件没有同步更新，有些则已经是会影响通信正确性的硬错误。

## 当前 CAN ID 分配情况

### FDCAN1

| ID | 方向 | DLC | 格式 | 用途 |
| --- | --- | --- | --- | --- |
| `0x11` | 发送 | 8 | Classic CAN | 底盘速度指令 |

### FDCAN2

| ID | 方向 | DLC | 格式 | 用途 |
| --- | --- | --- | --- | --- |
| `0x101` | 发送 | 12 | FD CAN + BRS | `StartTask03` 发送的位姿反馈 |
| `0x1FF` | 发送 | 8 | Classic CAN | DJI 电机第 2 组电流指令 |
| `0x200` | 发送 | 8 | Classic CAN | DJI 电机第 1 组电流指令 |
| `0x201` | 接收 | 8 | Classic CAN | DJI 电机反馈 |
| `0x202` | 接收 | 8 | Classic CAN | DJI 电机反馈 |
| `0x203` | 接收 | 8 | Classic CAN | DJI 电机反馈 |
| `0x205` | 接收 | 8 | Classic CAN | DJI 电机反馈 |
| `0x206` | 接收 | 8 | Classic CAN | DJI 电机反馈 |

### FDCAN3

| ID | 方向 | DLC | 格式 | 用途 |
| --- | --- | --- | --- | --- |
| `0xAA` | 接收 | 8 | Classic CAN | 定位数据第 1 帧：`x`、`y` |
| `0xAB` | 接收 | 8 | Classic CAN | 定位数据第 2 帧：`angle`、`vx` |
| `0xAC` | 接收 | 8 | Classic CAN | 定位数据第 3 帧：`vy`、`vr` |
| `0x100` | 接收 | 8 | Classic CAN | 上位机目标点 |
| `0x200` | 接收 | 4 或 8 | Classic CAN | 上位机云台指令 |
| `0x300` | 发送 | 8 | Classic CAN | 发给上位机的反馈 |

## 发现的问题

### 1. 严重问题：`FDCAN2` 上的 `0x101` 报文被配置成 12 字节 FD CAN，但底层初始化仍是 8 字节 Classic CAN

涉及文件：

- `Core/Src/freertos.c`
- `Core/Src/fdcan.c`

现象：

- `StartTask03` 中发送的 `0x101` 报文配置为：
  - `FDCAN_DLC_BYTES_12`
  - `FDCAN_FD_CAN`
  - `FDCAN_BRS_ON`
- 但 `MX_FDCAN2_Init()` 仍然配置为：
  - `FrameFormat = FDCAN_FRAME_CLASSIC`
  - `TxElmtSize = FDCAN_DATA_BYTES_8`
  - RX/TX 消息 RAM 元素大小仍然是 8 字节

影响：

- 这不是简单的注释问题，而是实际运行风险。
- HAL 在发送时会按照 DLC 对应的长度拷贝数据。
- 当前代码里 DLC 是 12 字节，但消息 RAM 分配只有 8 字节，配置前后不一致。
- 这类问题应视为确定性的高风险错误。

建议：

- 必须统一为一种协议方案：

方案 A：

- 保持 Classic CAN 8 字节
- 把 12 字节数据拆成多帧发送

方案 B：

- 完整改成 FD CAN
- 同步修改：
  - `FrameFormat`
  - `TxElmtSize`
  - 消息 RAM 配置
  - 对端节点协议能力

优先级：`P0`

### 2. 严重问题：定位协议定义前后不一致，存在“24 字节长帧”和“3 个 8 字节短帧”两套说法

涉及文件：

- `can/robot_data.h`
- `can/robot_data.c`
- `can/fdcan_bsp.h`
- `can/fdcan_bsp.c`
- `Core/Src/fdcan.c`

现象：

- `robot_data.h` 中写的是：
  - `CAN2`
  - ID `0xAA`
  - `24 Bytes`
- `fdcan_bsp` 的注释也提到“长帧定位数据”
- 但实际 FDCAN 初始化仍然全部是 `8 字节`
- 实际解析代码也不是收一个 24 字节长帧
- 实际逻辑是收三帧：
  - `0xAA`
  - `0xAB`
  - `0xAC`
- 实际注册监听的总线是 `hfdcan3`，不是 `hfdcan2`

影响：

- 当前代码里实际上同时存在两套协议理解：
  - 一套是“单帧 24 字节”
  - 一套是“拆成 3 个 8 字节短帧”
- 如果外部定位板按“单帧 24 字节”发送，当前程序一定解析不对。
- 如果外部已经按三帧发送，那么头文件和注释就是错误的，会误导后续开发。

建议：

- 必须冻结一份唯一的正式协议定义。
- 建议按当前实现统一为：
  - 总线：`FDCAN3`
  - 定位数据拆分为 `0xAA`、`0xAB`、`0xAC`
  - 每帧 8 字节
  - 使用 Classic CAN

优先级：`P0`

### 3. 高风险问题：上位机云台指令中 `pitch` 的缩放与 `yaw` 不一致

涉及文件：

- `can/robot_data.c`

现象：

- `yaw_cdeg` 被转换为：
  - `yaw / 100.0f`
- `pitch_cdeg` 却被直接赋值，没有除以 `100.0f`

当前代码含义：

- `yaw` 按百分之一度解释
- `pitch` 按原始整数度解释

影响：

- 如果协议定义中 `yaw` 和 `pitch` 本来单位相同，那么 `pitch` 现在会错 100 倍。
- 这会直接影响云台控制量。

建议：

- 先确认协议中 `yaw`、`pitch` 的单位是否一致。
- 如果两者都是百分之一度，则两者都应除以 `100.0f`。
- 如果两者单位本来就不同，需要在代码和协议文档中明确标注。

优先级：`P1`

### 4. 中风险问题：`fdcan_bsp` 的分发表只按 ID 建索引，没有区分总线实例

涉及文件：

- `can/fdcan_bsp.c`
- `can/fdcan_bsp.h`

现象：

- `fdcan_bsp_register()` 传入了 `hfdcan`
- 但内部建立索引时并没有把总线实例纳入 key
- 防重注册只判断 `hash_table[id]`
- 结果是：同一个 ID 无法在不同总线上独立注册

当前项目状态：

- 目前还没有看到这件事已经造成现网故障
- 但设计上已经埋下隐患

例如：

- `0x200` 在 `FDCAN2` 上用于 DJI 电机指令
- `0x200` 在 `FDCAN3` 上用于上位机云台指令

说明：

- 这本身不是“总线层面的 ID 冲突”，因为不是同一条 CAN 总线
- 真正的问题在于软件分发层没有把“总线”作为一等维度

建议：

- 将分发表索引从：
  - `ID`
- 改为：
  - `总线实例 + ID + ID类型`

优先级：`P1`

### 5. 中风险问题：`0x300` 发给上位机的反馈在头文件里写成 12 字节，但实际发送是 8 字节

涉及文件：

- `can/robot_data.h`
- `Core/Src/freertos.c`

现象：

- `robot_data.h` 中定义 `CAN_ID_PC_FEEDBACK` 为 `12 Bytes`
- 实际发送代码中 `DataLength = FDCAN_DLC_BYTES_8`
- 当前实际载荷为：
  - 4 字节时间戳
  - 2 字节 yaw
  - 2 字节 pitch

影响：

- 如果上位机按 12 字节协议解包，就会解错。
- 如果上位机已经按 8 字节在收，那么当前头文件定义就是错误的，会误导后续维护。

建议：

- 明确统一 `0x300` 的正式协议。
- 按当前代码实现，建议定义为：
  - ID `0x300`
  - DLC `8`
  - 载荷 = `timestamp[4] + yaw_i16[2] + pitch_i16[2]`

优先级：`P2`

### 6. 中风险问题：底盘初始化时先发了停止帧，但那时 FDCAN 还未启动

涉及文件：

- `can/chassis_task.c`
- `Core/Src/freertos.c`

现象：

- `Chassis_Init()` 中调用了 `Chassis_Stop()`
- 但 FDCAN 启动在后面才执行

影响：

- 初始化阶段发出的第一帧停止命令不一定真正发出。
- 这不是协议冲突，但会造成初始化行为与代码意图不一致。

建议：

- 二选一：

方案 A：

- 先启动 FDCAN，再执行 `Chassis_Init()`

方案 B：

- 初始化阶段不直接发 CAN
- 在 CAN 启动完成后再补发第一帧停止命令

优先级：`P2`

## 当前不算直接冲突的地方

### `0x200` 同时出现在 FDCAN2 和 FDCAN3

这不是物理总线层面的冲突，因为它们不在同一条总线上：

- `FDCAN2`：DJI 电机组播控制
- `FDCAN3`：上位机云台控制

但是，这个情况仍然需要在协议表里明确写清楚，否则后续维护很容易误判。

## 推荐修复顺序

1. 先修 `0x101` 的 FD CAN / Classic CAN 配置冲突
2. 统一定位协议定义：
   - 所属总线
   - 使用的 ID
   - 单帧还是多帧
   - 每帧长度
3. 确认并修正 `yaw/pitch` 的缩放单位
4. 重构 `fdcan_bsp` 的分发表，使其区分不同总线
5. 修正文档中 `0x300` 的帧长度定义
6. 调整底盘初始化与 CAN 启动顺序

## 建议采用的正式协议基线

如果当前外部设备协议还没有完全固化，建议后续将协议统一成下面这种更清晰的结构：

- `FDCAN1`
  - 底盘控制
  - 全部采用 Classic CAN
  - 固定 8 字节

- `FDCAN2`
  - DJI 电机
  - 全部采用 Classic CAN
  - 主要使用 `0x1FF`、`0x200`、`0x201` 到 `0x208`

- `FDCAN3`
  - 定位板
  - 上位机交互
  - 若无明确必要，建议继续使用 Classic CAN

这样可以减少 Classic/FD 混用带来的复杂度。

## 后续建议

- 建议补一份项目级协议总表，例如：
  - `CAN_PROTOCOL.md`
- 建议增加一个简单脚本或静态检查工具，用来扫描：
  - 同总线重复 ID
  - DLC 定义不一致
  - 注释和代码实现不一致
- 建议对外部节点分别做联调检查单：
  - 底盘板
  - 定位板
  - 上位机
  - DJI 电机总线

## 最终结论

本次检查结果如下：

- 没有发现同一条 CAN 总线上的直接 ID 重号
- 发现了多处协议定义与实现不一致的问题
- 其中有 2 项应视为优先修复的严重问题：
  - `0x101` 的 FD CAN 与底层 Classic CAN 配置冲突
  - 定位协议同时存在“24 字节长帧”和“3 个 8 字节短帧”两套定义

建议在后续继续扩展 CAN 功能之前，先把这两项问题收敛到统一协议版本。
