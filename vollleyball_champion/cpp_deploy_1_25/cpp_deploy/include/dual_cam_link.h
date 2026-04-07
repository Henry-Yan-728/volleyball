#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <vector>
#include <atomic>
#include <netinet/in.h>

// [修正] 使用 #pragma pack(push, 1) 强制字节对齐
// 防止不同编译器/架构产生的内存布局差异导致通信乱码
#pragma pack(push, 1)
struct RayPacket {
    double timestamp;   // 8 bytes
    double origin_x;    // 8 bytes
    double origin_y;
    double origin_z;
    double dir_x;
    double dir_y;
    double dir_z;
}; // Total: 56 bytes strictly
#pragma pack(pop)

class DualCamLink {
public:
    // peer_ip: 对机的 IP 地址
    // peer_port: 对机的端口 (发送目标)
    // local_port: 本机监听端口
    DualCamLink(const std::string& peer_ip, int peer_port, int local_port);
    ~DualCamLink();

    bool init();

    // 发送本机的射线给对机
    void sendRay(double timestamp, double ox, double oy, double oz, double dx, double dy, double dz);

    // 获取接收到的对机射线队列 (并清空缓存)
    // 线程安全
    std::vector<RayPacket> popReceivedRays();

private:
    void receiveLoop();

    std::string peer_ip_;
    int peer_port_;
    int local_port_;
    int socket_fd_ = -1;
    struct sockaddr_in peer_addr_;

    std::atomic<bool> running_{false};
    std::thread rx_thread_;

    std::mutex queue_mutex_;
    std::vector<RayPacket> rx_queue_;
};