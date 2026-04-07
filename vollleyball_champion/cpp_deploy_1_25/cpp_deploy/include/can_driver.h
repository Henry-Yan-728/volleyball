#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <linux/can.h>
#include <linux/can/raw.h>

// 机器人姿态结构体
struct RobotPoseData {
    float x;
    float y;
    float angle; // 弧度
    float vx;
    float vy;
    float vr;
    uint64_t timestamp_ms;
};

class CanDriver {
public:
    // 构造函数传入接口名 (如 "can0" 或 "can1")
    CanDriver(const std::string& interface_name);
    ~CanDriver();

    // 初始化 SocketCAN
    bool init();

    // 1. 发送目标落点指令 (ID 0x100) -> 给主控板
    bool sendTarget(float x, float y);

    // 2. 发送/转发机器人自身位姿 (ID 0xAA, 0xAB, 0xAC) -> 给主控板
    bool sendRobotPose(float x, float y, float angle, float vx, float vy, float vr);

    // 3. 发送校准指令 (ID 0x50, 0x51) -> 给定位板
    bool sendCalibration(float x, float y, float angle);

    // 获取最新接收到的机器人位姿 (线程安全)
    bool getRobotPose(RobotPoseData& out_pose);

private:
    // 接收线程主循环
    void receiveLoop();

    std::string ifname_;
    int socket_fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread rx_thread_;

    // 接收数据保护
    std::mutex pose_mutex_;
    RobotPoseData current_pose_;
    bool has_pose_data_ = false;
};