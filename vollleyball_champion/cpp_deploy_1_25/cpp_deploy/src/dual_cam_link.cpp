#include "../include/dual_cam_link.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

DualCamLink::DualCamLink(const std::string& peer_ip, int peer_port, int local_port)
    : peer_ip_(peer_ip), peer_port_(peer_port), local_port_(local_port) {}

DualCamLink::~DualCamLink() {
    running_ = false;
    if (socket_fd_ >= 0) close(socket_fd_);
    if (rx_thread_.joinable()) rx_thread_.join();
}

bool DualCamLink::init() {
    // 1. 创建 UDP Socket
    if ((socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("[DualLink] Socket creation failed");
        return false;
    }

    // 2. 绑定本机端口 (接收用)
    struct sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(local_port_);

    if (bind(socket_fd_, (const struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        perror("[DualLink] Bind failed");
        return false;
    }

    // 3. 配置对机地址 (发送用)
    memset(&peer_addr_, 0, sizeof(peer_addr_));
    peer_addr_.sin_family = AF_INET;
    peer_addr_.sin_port = htons(peer_port_);
    if (inet_pton(AF_INET, peer_ip_.c_str(), &peer_addr_.sin_addr) <= 0) {
        perror("[DualLink] Invalid Peer IP");
        return false;
    }

    // 启动接收线程
    running_ = true;
    rx_thread_ = std::thread(&DualCamLink::receiveLoop, this);
    
    std::cout << "[DualLink] Listening on port " << local_port_ << ", Target: " << peer_ip_ << ":" << peer_port_ << std::endl;
    return true;
}

void DualCamLink::sendRay(double timestamp, double ox, double oy, double oz, double dx, double dy, double dz) {
    RayPacket packet;
    packet.timestamp = timestamp;
    packet.origin_x = ox; packet.origin_y = oy; packet.origin_z = oz;
    packet.dir_x = dx;    packet.dir_y = dy;    packet.dir_z = dz;

    sendto(socket_fd_, &packet, sizeof(packet), 0, (const struct sockaddr *)&peer_addr_, sizeof(peer_addr_));
}

void DualCamLink::receiveLoop() {
    RayPacket packet;
    struct sockaddr_in sender_addr;
    socklen_t len = sizeof(sender_addr);

    while (running_) {
        int n = recvfrom(socket_fd_, &packet, sizeof(packet), 0, (struct sockaddr *)&sender_addr, &len);
        if (n == sizeof(RayPacket)) {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            rx_queue_.push_back(packet);
        }
    }
}

std::vector<RayPacket> DualCamLink::popReceivedRays() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    std::vector<RayPacket> temp = rx_queue_;
    rx_queue_.clear();
    return temp;
}