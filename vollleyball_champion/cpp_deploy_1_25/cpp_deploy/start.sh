#!/bin/bash
set -e

# ================= 配置区域 =================
APP_NAME="yolo_app"
TRT_ROOT="/home/robot/TensorRT-10.14.1.48"

# --- 配置 1: can0 (PEAK 原生卡) ---
# PEAK 驱动通常会自动将其命名为 can0
NATIVE_IF="can0"

# --- 配置 2: can1 (旧版 ACM 串口卡) ---
# 这里填你用 ls /dev/ttyACM* 看到的那个设备名
# 只有旧版卡会显示为 ACM，PEAK 不会显示在这里，所以通常是 ACM0
SLCAN_DEV="/dev/ttyACM1"
SLCAN_IF="can1"  # 我们强制把它叫 can1

BITRATE=1000000 # 1Mbps
SLCAN_SPEED="-s8" # SLCAN 协议的 1Mbps
# ===========================================

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}[System] 启动流程 (PEAK + SLCAN 混合模式)...${NC}"

# 1. 加载所有驱动
sudo modprobe can
sudo modprobe can-raw
sudo modprobe peak_usb  # PEAK 驱动
sudo modprobe slcan     # 旧卡驱动

# ================= 函数定义 =================

# 函数 A: 配置原生 PEAK 卡 -> can0
setup_native_peak() {
    local IF_NAME=$1
    echo -e "${GREEN}[Setup] 配置 PEAK 原生接口: $IF_NAME${NC}"
    
    # 检查接口是否存在 (PEAK 插上后内核会自动创建网络接口)
    if ! ip link show "$IF_NAME" > /dev/null 2>&1; then
        echo -e "${RED}[Error] 找不到原生接口 $IF_NAME !${NC}"
        echo -e "${YELLOW}可能原因:${NC}"
        echo -e "  1. PEAK 卡没插好。"
        echo -e "  2. 系统把它识别成了 can1 (请运行 'ip link' 确认名字)。"
        exit 1
    fi

    # 检查是否是 SLCAN 冒充的 (防止冲突)
    # 原生卡的 qlen 通常很大，或者通过 ethtool 可查，这里简单重置
    sudo ip link set "$IF_NAME" down
    sudo ip link set "$IF_NAME" type can bitrate $BITRATE
    sudo ip link set "$IF_NAME" up
    
    echo -e "${GREEN}[Success] PEAK ($IF_NAME) 已就绪${NC}"
}

# 函数 B: 配置旧版 SLCAN 卡 -> can1
setup_slcan_legacy() {
    local TTY_DEV=$1
    local IF_NAME=$2
    
    echo -e "${GREEN}[Setup] 配置旧版 SLCAN 接口: $TTY_DEV -> $IF_NAME${NC}"

    # 1. 清理旧接口 (防止 File exists 错误)
    if ip link show "$IF_NAME" > /dev/null 2>&1; then
        echo -e "${YELLOW}[Info] 清理残留接口 $IF_NAME...${NC}"
        sudo ip link set "$IF_NAME" down 2>/dev/null
        sudo ip link delete "$IF_NAME" 2>/dev/null || true
    fi
    
    # 2. 检查串口
    if [ ! -e "$TTY_DEV" ]; then
        echo -e "${RED}[Error] 找不到串口设备: $TTY_DEV${NC}"
        echo -e "${YELLOW}请运行 'ls /dev/ttyACM*' 确认旧卡的路径。${NC}"
        exit 1
    fi
    
    # 3. 强制清理串口占用
    sudo fuser -k "$TTY_DEV" > /dev/null 2>&1 || true

    # 4. 挂载
    # -o: open once, -c: close exit, -s8: 1Mbps
    sudo slcand -o -c $SLCAN_SPEED $TTY_DEV $IF_NAME
    sleep 0.5
    sudo ip link set up $IF_NAME
    
    if ip link show "$IF_NAME" | grep -q "state UP"; then
        echo -e "${GREEN}[Success] 旧版卡 ($IF_NAME) 已就绪${NC}"
    else
        echo -e "${RED}[Error] SLCAN 挂载失败!${NC}"
        exit 1
    fi
}

# ================= 执行逻辑 =================

# 1. 先配置 PEAK (can0)
# 这一步必须成功，它是你的定位核心
setup_native_peak "$NATIVE_IF"

# 2. 再配置旧卡 (can1)
setup_slcan_legacy "$SLCAN_DEV" "$SLCAN_IF"


# ================= 编译运行 =================
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${TRT_ROOT}/lib:/usr/local/cuda/lib64

echo -e "${GREEN}[Build] 编译...${NC}"
mkdir -p build
cd build
cmake ..
make -j$(nproc)

if [ ! -f "../best.engine" ]; then
    echo -e "${RED}[Error] 缺少 best.engine${NC}"; exit 1;
fi
ln -sf ../best.engine best.engine

echo -e "${GREEN}[Run] 启动主程序...${NC}"
./$APP_NAME