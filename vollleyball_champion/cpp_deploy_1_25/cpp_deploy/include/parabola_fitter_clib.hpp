#pragma once

#include <vector>
#include <utility>
#include <Eigen/Dense>

// 保持与 main.cpp 一致的点结构
// (如果在 main.cpp 里已经定义了 Point3D，这里可以删除定义，或者把 main.cpp 里的删掉改用这里的)
// 建议：保留这里的定义，确保 main.cpp include 这个头文件
// 注意：main.cpp 里的 Point3D 多了一个 double t，这里可以兼容或者修改
struct Point3D {
    double x;
    double y;
    double z;
    double t; // 为了兼容 main.cpp 的接口
};

struct Point2D {
    double x;
    double y;
};

class ParabolaFitter {
public:
    ParabolaFitter() = default;
    ~ParabolaFitter() = default;

    // 功能 1: 初始化点集
    void addPoint(double x, double y, double z);
    // 兼容 main.cpp 的接口
    void setPoints(const std::vector<Point3D>& points);

    // 拟合流程
    bool fit();

    // 是否已经拟合
    bool hasFitted() const;

    // *** 核心修复：添加 main.cpp 需要的接口 ***
    Point2D predictLandingPoint(double z_target) const;

    // 返回拟合出的直线 XY 平面参数 (k,b)。如果垂直于 X 轴，k=inf，b 表示 X 常量
    std::pair<double, double> getLineXY() const;

    // 功能 2: 给定 z，求对应 (x,y) 最多 两组
    std::vector<std::pair<double, double>> solveXYforZ(double z_query) const;

    // 预测某个 (x,y) 的 z 值
    double predictZ(double x, double y) const;

    // 清除状态
    void clear();

private:
    std::vector<Point3D> pts;

    // 直线 XY
    double line_k = 0;
    double line_b = 0;
    bool vertical_line = false;

    // 平面内抛物线系数
    double para_a = 0;
    double para_b = 0;
    double para_c = 0;

    bool fitted = false;

    bool fitLineXY();
    bool fitParabolaInPlane();
};