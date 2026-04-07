#include "../include/camera_solver.h"

CameraSolver::CameraSolver(const cv::Mat& K, const cv::Mat& R_c_r, const cv::Mat& T_c_r) {
    K_ = K.clone();
    K_inv_ = K.inv(); 
    R_cam_robot_ = R_c_r.clone();
    T_cam_robot_ = T_c_r.clone();
}

void CameraSolver::updateRobotPose(double timestamp, float robot_x, float robot_y, float robot_angle) {
    // [修正] 存入队列并限制长度
    pose_buffer_.push_back({timestamp, robot_x, robot_y, robot_angle});
    
    // 保留最近 1.0 秒的数据用于插值即可 (假设数据频率 > 10Hz)
    while (pose_buffer_.size() > 200 || 
          (!pose_buffer_.empty() && pose_buffer_.front().timestamp < timestamp - 1.0)) {
        pose_buffer_.pop_front();
    }
}

// [新增] 插值核心逻辑
CameraSolver::RobotPose CameraSolver::getInterpolatedPose(double timestamp) {
    if (pose_buffer_.empty()) {
        return {timestamp, 0, 0, 0}; // 无数据时的默认值，实际应报警
    }

    // 1. 如果时间戳在队列范围外，取最近的
    if (timestamp <= pose_buffer_.front().timestamp) return pose_buffer_.front();
    if (timestamp >= pose_buffer_.back().timestamp) return pose_buffer_.back();

    // 2. 二分查找找到时间戳前后的两个点
    auto it = std::lower_bound(pose_buffer_.begin(), pose_buffer_.end(), timestamp,
        [](const RobotPose& p, double t) { return p.timestamp < t; });
    
    // lower_bound 返回第一个 >= timestamp 的元素 (it)，那么前一个 (it-1) 就是 < timestamp 的
    const auto& p_next = *it;
    const auto& p_prev = *(it - 1);

    // 3. 线性插值
    double dt = p_next.timestamp - p_prev.timestamp;
    if (std::abs(dt) < 1e-6) return p_prev; // 防止除零

    double ratio = (timestamp - p_prev.timestamp) / dt;

    RobotPose res;
    res.timestamp = timestamp;
    res.x = p_prev.x + (p_next.x - p_prev.x) * ratio;
    res.y = p_prev.y + (p_next.y - p_prev.y) * ratio;
    
    // 角度插值 (注意：简单的线性插值无法处理 -PI 到 PI 的跳变)
    // 如果机器人只是在局部范围内转动，不跨越 +/-PI 分界线，可以直接线性插值
    // 严谨做法是先转为向量求平均再转回角度，这里保持简单，假设不跨越切点
    res.angle = p_prev.angle + (p_next.angle - p_prev.angle) * ratio;

    return res;
}

TimedRay CameraSolver::pixelToWorldRay(double timestamp, const cv::Point2f& pixel_pt) {
    // 1. 像素 -> 相机坐标系 (归一化平面)
    // P_cam = K_inv * [u, v, 1]
    cv::Mat p_img = (cv::Mat_<double>(3, 1) << pixel_pt.x, pixel_pt.y, 1.0);
    cv::Mat p_cam = K_inv_ * p_img; 

    // 2. [修正] 获取插值后的机器人位姿
    RobotPose pose = getInterpolatedPose(timestamp);

    // 3. 构建机器人位姿矩阵 (World_T_Robot)
    // [修正] 假定 pose.angle 已经是弧度，不再乘以 PI/180
    float theta = pose.angle; 
    
    cv::Mat R_world_robot = (cv::Mat_<double>(3, 3) << 
        cos(theta), -sin(theta), 0,
        sin(theta),  cos(theta), 0,
        0,           0,          1
    );
    cv::Mat T_world_robot = (cv::Mat_<double>(3, 1) << pose.x, pose.y, 0);

    // 4. 计算相机在世界坐标系的位姿
    cv::Mat R_wc = R_world_robot * R_cam_robot_;
    cv::Mat T_wc = R_world_robot * T_cam_robot_ + T_world_robot;

    // 5. 将射线方向转到世界坐标系
    cv::Mat d_world = R_wc * p_cam;

    cv::Point3d dir(d_world.at<double>(0), d_world.at<double>(1), d_world.at<double>(2));
    cv::Point3d origin(T_wc.at<double>(0), T_wc.at<double>(1), T_wc.at<double>(2));

    // 归一化
    double n = cv::norm(dir);
    if (n > 1e-6) dir *= (1.0 / n);

    TimedRay ray = {timestamp, origin, dir};
    
    local_rays_.push_back(ray);
    if (local_rays_.size() > 50) local_rays_.erase(local_rays_.begin());

    return ray;
}

void CameraSolver::addExternalRay(double timestamp, const cv::Point3d& origin, const cv::Point3d& dir) {
    ext_rays_.push_back({timestamp, origin, dir});
    if (ext_rays_.size() > 50) ext_rays_.erase(ext_rays_.begin());
}

bool CameraSolver::compute3DPoint(cv::Point3d& out_point, double& out_timestamp) {
    if (local_rays_.empty() || ext_rays_.empty()) return false;

    int best_local = -1;
    int best_ext = -1;
    double min_diff = 0.05; // 50ms 容差

    // 简单的双循环匹配
    for (int i = local_rays_.size() - 1; i >= 0; i--) {
        for (int j = ext_rays_.size() - 1; j >= 0; j--) {
            double diff = std::abs(local_rays_[i].timestamp - ext_rays_[j].timestamp);
            if (diff < min_diff) {
                min_diff = diff;
                best_local = i;
                best_ext = j;
            }
        }
        if (best_local != -1) break; 
    }

    if (best_local == -1) return false;

    // 异面直线公垂线中点算法
    const auto& r1 = local_rays_[best_local];
    const auto& r2 = ext_rays_[best_ext];

    cv::Point3d p1 = r1.origin;
    cv::Point3d d1 = r1.direction;
    cv::Point3d p2 = r2.origin;
    cv::Point3d d2 = r2.direction;

    cv::Point3d r = p1 - p2;
    double a = d1.dot(d1);
    double b = d1.dot(d2);
    double c = d2.dot(d2);
    double e = d1.dot(r);
    double f = d2.dot(r);

    double denom = a * c - b * b;
    if (std::abs(denom) < 1e-6) return false; // 平行

    double s = (b * f - c * e) / denom;
    double t = (a * f - b * e) / denom;

    cv::Point3d pa = p1 + d1 * s;
    cv::Point3d pb = p2 + d2 * t;

    out_point = (pa + pb) * 0.5;
    out_timestamp = (r1.timestamp + r2.timestamp) * 0.5;

    return true;
}