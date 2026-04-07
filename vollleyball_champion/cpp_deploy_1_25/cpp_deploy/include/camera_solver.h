#pragma once
#include <vector>
#include <deque> // [修正] 新增 deque 用于维护历史数据
#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>
#include <iostream>

struct TimedRay {
    double timestamp;
    cv::Point3d origin;
    cv::Point3d direction;
};

class CameraSolver {
public:
    // 构造函数传入内参 K，和 外参 (Cam相对于Robot)
    // T_c_r: 相机在机器人坐标系下的位置
    // R_c_r: 相机在机器人坐标系下的旋转
    CameraSolver(const cv::Mat& K, const cv::Mat& R_c_r, const cv::Mat& T_c_r);

    // 1. 更新机器人当前的世界位姿
    // [严谨] angle 必须单位为 弧度 (Rad)，且 timestamp 需与相机时间戳同源 (Unix秒)
    void updateRobotPose(double timestamp, float robot_x, float robot_y, float robot_angle);

    // 2. 输入像素点 (u, v)，计算出世界坐标系下的射线
    // 内部会自动进行位姿插值
    TimedRay pixelToWorldRay(double timestamp, const cv::Point2f& pixel_pt);

    // 3. 加入外部射线 (来自对机)
    void addExternalRay(double timestamp, const cv::Point3d& origin, const cv::Point3d& dir);

    // 4. 双线交会求中点 (核心算法)
    bool compute3DPoint(cv::Point3d& out_point, double& out_timestamp);

private:
    cv::Mat K_;
    cv::Mat K_inv_; 
    
    cv::Mat R_cam_robot_;
    cv::Mat T_cam_robot_;

    struct RobotPose {
        double timestamp;
        float x, y, angle;
    };
    
    // [修正] 使用队列替代单一变量，用于时间对齐插值
    std::deque<RobotPose> pose_buffer_;

    // 射线缓存
    std::vector<TimedRay> local_rays_; 
    std::vector<TimedRay> ext_rays_;   
    
    // 辅助函数：根据时间戳获取插值后的位姿
    RobotPose getInterpolatedPose(double timestamp);
};