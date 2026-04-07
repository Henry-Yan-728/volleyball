#include "../include/can_driver.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <chrono>

CanDriver::CanDriver(const std::string& interface_name) : ifname_(interface_name) {}

CanDriver::~CanDriver() {
    running_ = false;
    if (socket_fd_ >= 0) close(socket_fd_);
    if (rx_thread_.joinable()) rx_thread_.join();
}

bool CanDriver::init() {
    if ((socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror(("[CAN " + ifname_ + "] Socket create failed").c_str());
        return false;
    }

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, ifname_.c_str());
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        perror(("[CAN " + ifname_ + "] Locate interface failed").c_str());
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // 开启 CAN FD 支持 (即使只用经典帧，开启通常兼容性更好)
    int enable_fd = 1; 
    setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd));

    if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror(("[CAN " + ifname_ + "] Bind failed").c_str());
        return false;
    }

    std::cout << "[CAN] Initialized " << ifname_ << " successfully." << std::endl;
    running_ = true;
    rx_thread_ = std::thread(&CanDriver::receiveLoop, this);
    return true;
}

// 发送落点预测 (ID: 0x100)
bool CanDriver::sendTarget(float x, float y) {
    struct can_frame frame; 
    std::memset(&frame, 0, sizeof(frame));

    frame.can_id = 0x100;
    frame.can_dlc = 8;
    std::memcpy(frame.data, &x, 4);
    std::memcpy(frame.data + 4, &y, 4);

    if (write(socket_fd_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        return false;
    }
    return true;
}

// 发送/转发机器人位姿 (模仿定位板协议: 0xAA, 0xAB, 0xAC)
bool CanDriver::sendRobotPose(float x, float y, float angle, float vx, float vy, float vr) {
    struct can_frame frame;
    
    // --- 包 1: ID 0xAA (X, Y) ---
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = 0xAA;
    frame.can_dlc = 8;
    std::memcpy(frame.data, &x, 4);
    std::memcpy(frame.data + 4, &y, 4);
    if (write(socket_fd_, &frame, sizeof(frame)) != sizeof(frame)) return false;

    // --- 包 2: ID 0xAB (Angle, Vx) ---
    // 微小延时，防止 STM32 接收中断过载
    std::this_thread::sleep_for(std::chrono::microseconds(100)); 
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = 0xAB;
    frame.can_dlc = 8;
    std::memcpy(frame.data, &angle, 4);
    std::memcpy(frame.data + 4, &vx, 4);
    if (write(socket_fd_, &frame, sizeof(frame)) != sizeof(frame)) return false;

    // --- 包 3: ID 0xAC (Vy, Vr) ---
    std::this_thread::sleep_for(std::chrono::microseconds(100)); 
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = 0xAC;
    frame.can_dlc = 8;
    std::memcpy(frame.data, &vy, 4);
    std::memcpy(frame.data + 4, &vr, 4);
    if (write(socket_fd_, &frame, sizeof(frame)) != sizeof(frame)) return false;

    return true;
}

// 发送校准指令 (ID: 0x50, 0x51)
bool CanDriver::sendCalibration(float x, float y, float angle) {
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    
    // Part 1: X, Y
    frame.can_id = 0x50;
    frame.can_dlc = 8;
    std::memcpy(frame.data, &x, 4);
    std::memcpy(frame.data + 4, &y, 4);
    if (write(socket_fd_, &frame, sizeof(frame)) != sizeof(frame)) return false;

    std::this_thread::sleep_for(std::chrono::microseconds(200)); 

    // Part 2: Angle
    frame.can_id = 0x51;
    frame.can_dlc = 8;
    std::memcpy(frame.data, &angle, 4);
    if (write(socket_fd_, &frame, sizeof(frame)) != sizeof(frame)) return false;

    return true;
}

bool CanDriver::getRobotPose(RobotPoseData& out_pose) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!has_pose_data_) return false;
    out_pose = current_pose_;
    return true;
}

void CanDriver::receiveLoop() {
    struct can_frame frame;
    uint8_t temp_buffer[24]; 
    bool got_AA = false, got_AB = false, got_AC = false;

    while (running_) {
        int nbytes = read(socket_fd_, &frame, sizeof(struct can_frame));
        if (nbytes < 0) { 
            std::this_thread::sleep_for(std::chrono::milliseconds(2)); 
            continue; 
        }

        // === 拼包逻辑 (对应下位机 0xAA+i) ===
        if (frame.can_id == 0xAA) {
            std::memcpy(temp_buffer, frame.data, 8);
            got_AA = true;
        }
        else if (frame.can_id == 0xAB) {
            std::memcpy(temp_buffer + 8, frame.data, 8);
            got_AB = true;
        }
        else if (frame.can_id == 0xAC) {
            std::memcpy(temp_buffer + 16, frame.data, 8);
            got_AC = true;
        }

        // 集齐三颗龙珠，召唤神龙
        if (got_AA && got_AB && got_AC) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            float* f_ptr = (float*)temp_buffer;
            
            current_pose_.x     = f_ptr[0];
            current_pose_.y     = f_ptr[1];
            current_pose_.angle = f_ptr[2];
            current_pose_.vx    = f_ptr[3];
            current_pose_.vy    = f_ptr[4];
            current_pose_.vr    = f_ptr[5];
            
            struct timeval tv;
            gettimeofday(&tv, NULL);
            current_pose_.timestamp_ms = tv.tv_sec * 1000 + tv.tv_usec / 1000;
            has_pose_data_ = true;

            // 重置标志位
            got_AA = false; got_AB = false; got_AC = false;
        }
    }
}