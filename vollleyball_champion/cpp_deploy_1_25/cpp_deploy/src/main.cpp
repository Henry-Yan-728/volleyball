#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <csignal>
#include <vector>
#include <deque>

// 引入所有组件头文件
#include "../include/inference.h"
#include "../include/can_driver.h"
#include "../include/dual_cam_link.h"
#include "../include/camera_solver.h"
#include "../include/parabola_fitter_clib.hpp" 

// ================= 配置参数 =================

// 1. 网络配置 (双机互联)
const std::string PEER_IP = "192.168.1.102"; 
const int PEER_PORT = 8888;
const int LOCAL_PORT = 8888;

// 2. CAN 接口名称 (双 CAN 模式)
const std::string CAN_LOC_IF = "can0"; // 连接定位板 (收)
const std::string CAN_CTRL_IF = "can1"; // 连接主控板 (发)

// 3. 相机内参 (K) - 必须标定！
const double K_DATA[9] = {
    1000.0, 0.0,    320.0,
    0.0,    1000.0, 240.0,
    0.0,    0.0,    1.0
};

// 4. 相机外参 (相机光心相对于车体中心)
// 这里的 T_CR_DATA 必须拿卷尺实测！不要填 0！
const double R_CR_DATA[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
const double T_CR_DATA[3] = {0.0, 0.0, 1.5}; // 假设安装高度1.5米

// 5. 预测参数
const int MIN_FIT_POINTS = 6;       // 至少要有6个点才开始拟合
const int MAX_BUFFER_SIZE = 30;     // 轨迹缓冲区最大长度
const double GROUND_Z = 0.0;        // 地面高度

// ================= 全局变量 =================
std::atomic<bool> g_running{true};

// [修正] 严重警告：原此处定义的 struct Point3D 违反 ODR (One Definition Rule)
// 已删除，强制使用 parabola_fitter_clib.hpp 中的定义

void signalHandler(int signum) {
    std::cout << "\n[System] Interrupt signal received. Stopping...\n";
    g_running = false;
}

// 获取系统当前时间 (Unix Timestamp, 秒)
double getTimestamp() {
    using namespace std::chrono;
    // 必须确保两台工控机均安装了 chrony 或 ntp 并且已同步
    auto now = system_clock::now();
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()).count();
    return ms / 1000.0; // 返回秒 (double)
}

int main() {
    signal(SIGINT, signalHandler);

    std::cout << "[System] Initializing..." << std::endl;

    // ================= 1. 初始化 CAN (双网关模式) =================
    // 驱动 A: 负责听定位板
    CanDriver loc_driver(CAN_LOC_IF);
    if (!loc_driver.init()) {
        std::cerr << "[Error] Locator CAN (can0) Init Failed!" << std::endl;
        return -1;
    }
    // 驱动 B: 负责指挥主控板
    CanDriver ctrl_driver(CAN_CTRL_IF);
    if (!ctrl_driver.init()) {
        std::cerr << "[Error] Controller CAN (can1) Init Failed!" << std::endl;
        return -1;
    }

    // ================= 2. 初始化其他模块 =================
    DualCamLink dual_link(PEER_IP, PEER_PORT, LOCAL_PORT);
    if (!dual_link.init()) {
        std::cerr << "[Error] DualLink Init Failed!" << std::endl;
        return -1;
    }

    cv::Mat K = cv::Mat(3, 3, CV_64F, (void*)K_DATA).clone();
    cv::Mat R_cr = cv::Mat(3, 3, CV_64F, (void*)R_CR_DATA).clone();
    cv::Mat T_cr = cv::Mat(3, 1, CV_64F, (void*)T_CR_DATA).clone();
    CameraSolver cam_solver(K, R_cr, T_cr);

    ParabolaFitter fitter; 

    std::cout << "[System] Loading AI Model..." << std::endl;
    // 确保 engine 文件路径正确
    YOLOInference yolo("best.engine");

    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "[Error] Camera Open Failed!" << std::endl;
        return -1;
    }
    
    // === 关键：手动曝光防止拖影 ===
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 640);
    cap.set(cv::CAP_PROP_FPS, 60);
    // V4L2 只有在特定驱动下才支持 1=Manual, 需根据实际相机调整
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1); 
    cap.set(cv::CAP_PROP_EXPOSURE, 100);     // 需实验调整

    std::cout << "[System] Loop Start." << std::endl;

    // ================= 3. 运行时变量 =================
    cv::Mat frame;
    RobotPoseData robot_pose;
    
    // 轨迹缓冲池 (使用 header 中定义的 Point3D)
    std::vector<Point3D> traj_buffer; 
    bool target_lock = false; 

    while (g_running) {
        // ---------------------------------------------------------
        // A. [网关核心] 获取定位并转发
        // ---------------------------------------------------------
        if (loc_driver.getRobotPose(robot_pose)) {
            // [修正] 核心时间逻辑修复
            // 原错误代码: cam_solver.updateRobotPose(getTimestamp(), ...);
            // 错误原因: 这里的 getTimestamp() 是"处理该行代码的时间"，比"CAN接收时间"滞后，
            //          更是比"定位板实际采样时间"滞后。这会导致图像与位姿在时间轴上无法对齐。
            // 修正方案: 使用 CAN 驱动层打上的接收时间戳 (timestamp_ms)。
            
            double pose_timestamp_sec = static_cast<double>(robot_pose.timestamp_ms) / 1000.0;
            
            cam_solver.updateRobotPose(pose_timestamp_sec, 
                                       robot_pose.x, 
                                       robot_pose.y, 
                                       robot_pose.angle);

            // 2. 转发给主控板
            ctrl_driver.sendRobotPose(
                robot_pose.x, robot_pose.y, robot_pose.angle,
                robot_pose.vx, robot_pose.vy, robot_pose.vr
            );
        }

        // ---------------------------------------------------------
        // B. 视觉感知
        // ---------------------------------------------------------
        cap >> frame;
        if (frame.empty()) break;
        
        // 记录帧获取时间 (近似为曝光结束时间)
        double frame_time = getTimestamp(); 

        auto detections = yolo.infer(frame);
        
        bool ball_detected = false;
        float ball_u = 0, ball_v = 0;

        for (const auto& det : detections) {
            if ((int)det[0] == 0) { 
                // [注意] 这里的 det[2/3] 已经是修正后的 Center_X, Center_Y
                ball_u = det[2]; 
                ball_v = det[3]; 
                ball_detected = true;
                // 画点确认
                cv::circle(frame, cv::Point(ball_u, ball_v), 5, cv::Scalar(0,255,0), -1);
                break; 
            }
        }

        // ---------------------------------------------------------
        // C. 双机射线交换
        // ---------------------------------------------------------
        if (ball_detected) {
            // 计算像素射线 
            // cam_solver 内部会在 pose_buffer 中寻找 frame_time 时刻的机器人位姿进行插值
            TimedRay local_ray = cam_solver.pixelToWorldRay(frame_time, cv::Point2f(ball_u, ball_v));
            
            dual_link.sendRay(local_ray.timestamp, 
                              local_ray.origin.x, local_ray.origin.y, local_ray.origin.z,
                              local_ray.direction.x, local_ray.direction.y, local_ray.direction.z);
        }

        auto ext_rays = dual_link.popReceivedRays();
        for (const auto& pkt : ext_rays) {
            cam_solver.addExternalRay(pkt.timestamp, 
                                      cv::Point3d(pkt.origin_x, pkt.origin_y, pkt.origin_z),
                                      cv::Point3d(pkt.dir_x, pkt.dir_y, pkt.dir_z));
        }

        // ---------------------------------------------------------
        // D. 3D 解算与缓冲
        // ---------------------------------------------------------
        cv::Point3d ball_3d;
        double ball_t;

        if (cam_solver.compute3DPoint(ball_3d, ball_t)) {
            // 只有当球在一定高度以上才记录 (过滤地面噪声)
            if (ball_3d.z > 0.3) {
                // 存入缓冲区
                // 注意：Point3D 的 t 字段在 main.cpp 逻辑中用于后续卡尔曼滤波扩展
                // 当前 ParabolaFitter 可能不使用 t，但保留数据是好的
                traj_buffer.push_back({ball_3d.x, ball_3d.y, ball_3d.z, ball_t});
                
                if (traj_buffer.size() > MAX_BUFFER_SIZE) {
                    traj_buffer.erase(traj_buffer.begin());
                }
            }

            // 可视化
            std::string info = "Z: " + std::to_string(ball_3d.z).substr(0,4);
            cv::putText(frame, info, cv::Point(20, 50), 0, 1, cv::Scalar(0,255,255), 2);
        } 
        else {
            // 可选：超时清空逻辑
            if (!ball_detected && !traj_buffer.empty()) {
                // 这里暂时保持原逻辑，依靠 buffer 滚动
            }
        }

        // ---------------------------------------------------------
        // E. 轨迹拟合与控制指令发送
        // ---------------------------------------------------------
        if (traj_buffer.size() >= MIN_FIT_POINTS && !target_lock) {
            
            fitter.setPoints(traj_buffer); 

            // 拟合
            if (fitter.fit()) {
                // 预测落点
                auto landing = fitter.predictLandingPoint(GROUND_Z); 
                float tx = landing.x;
                float ty = landing.y;

                // 场地范围保护 (假设场地半径 5 米)
                if (std::abs(tx) < 5.0 && std::abs(ty) < 5.0) {
                    std::cout << "[CMD] Landing at: " << tx << ", " << ty << std::endl;
                    
                    ctrl_driver.sendTarget(tx, ty);
                    
                    target_lock = true; 
                    cv::putText(frame, "CMD SENT", cv::Point(20, 100), 0, 1, cv::Scalar(0,0,255), 2);
                }
            }
        }
        
        // 解锁逻辑：如果缓冲被清空（例如新的一轮开始），解除锁定
        if (traj_buffer.empty()) {
            target_lock = false;
        }

        cv::imshow("Pilot", frame);
        if (cv::waitKey(1) == 27) break;
    }

    return 0;
}