#include "../include/parabola_fitter_clib.hpp"
#include <limits>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <iostream>

// 严格检查 Eigen 是否可用
#ifndef EIGEN_MAJOR_VERSION
#error "Eigen3 library is required but not found."
#endif

void ParabolaFitter::addPoint(double x, double y, double z) {
    // 简单的单点添加实现，兼容 Point3D
    pts.push_back({x, y, z, 0.0}); 
    fitted = false;
}

void ParabolaFitter::setPoints(const std::vector<Point3D>& points) {
    pts = points;
    fitted = false;
    vertical_line = false;
    line_k = 0; line_b = 0;
    para_a = 0; para_b = 0; para_c = 0;
}

bool ParabolaFitter::fit() {
    if (pts.size() < 3) {
        return false;
    }
    
    // 1. 拟合 XY 平面投影直线 (决定运动方位)
    if (!fitLineXY()) return false;
    
    // 2. 拟合平面内的垂直抛物线 (决定高度变化)
    if (!fitParabolaInPlane()) return false;
    
    fitted = true;
    return true;
}

bool ParabolaFitter::hasFitted() const {
    return fitted;
}

// [修正] 新增：实现头文件中声明的预测落地函数
Point2D ParabolaFitter::predictLandingPoint(double z_target) const {
    if (!fitted) return {0.0, 0.0};

    // 1. 获取所有高度为 z_target 的可能解
    std::vector<std::pair<double, double>> candidates = solveXYforZ(z_target);

    if (candidates.empty()) {
        return {0.0, 0.0};
    }

    // 2. 筛选逻辑：如果是 2 个解，通常一个是起飞点（或反向延长的虚拟点），一个是落地点
    // 简单的启发式策略：取距离轨迹起点最远的那个点作为落地点
    double start_x = pts.front().x;
    double start_y = pts.front().y;

    double max_dist_sq = -1.0;
    Point2D best_p = {0.0, 0.0};

    for (const auto& p : candidates) {
        double dx = p.first - start_x;
        double dy = p.second - start_y;
        double dist_sq = dx*dx + dy*dy;
        
        if (dist_sq > max_dist_sq) {
            max_dist_sq = dist_sq;
            best_p = {p.first, p.second};
        }
    }

    return best_p;
}

std::pair<double, double> ParabolaFitter::getLineXY() const {
    if (!fitted) return {0.0, 0.0};
    if (vertical_line) {
        return { std::numeric_limits<double>::infinity(), line_b };
    }
    return { line_k, line_b };
}

void ParabolaFitter::clear() {
    pts.clear();
    fitted = false;
    vertical_line = false;
    line_k = 0;
    line_b = 0;
    para_a = 0;
    para_b = 0;
    para_c = 0;
}

bool ParabolaFitter::fitLineXY() {
    size_t n = pts.size();
    if (n < 2) return false;

    Eigen::VectorXd X(n), Y(n);
    for (size_t i = 0; i < n; i++) {
        X(i) = pts[i].x;
        Y(i) = pts[i].y;
    }

    double meanX = X.mean();
    double meanY = Y.mean();

    double num = 0.0, den = 0.0;
    for (size_t i = 0; i < n; i++) {
        double dx = X(i) - meanX;
        double dy = Y(i) - meanY;
        num += dx * dy;
        den += dx * dx;
    }

    if (std::abs(den) < 1e-9) {
        vertical_line = true;
        line_b = meanX; 
        line_k = std::numeric_limits<double>::infinity();
        return true;
    }

    vertical_line = false;
    line_k = num / den;
    line_b = meanY - line_k * meanX;
    return true;
}

bool ParabolaFitter::fitParabolaInPlane() {
    Eigen::Vector3d e1;

    if (vertical_line) {
        e1 = Eigen::Vector3d(0, 1, 0); 
    } else {
        Eigen::Vector3d v_xy(1.0, line_k, 0.0);
        v_xy.normalize();
        e1 = v_xy;
    }

    size_t n = pts.size();
    Eigen::MatrixXd A(n, 3);
    Eigen::VectorXd Z(n);

    for (size_t i = 0; i < n; i++) {
        Eigen::Vector3d p(pts[i].x, pts[i].y, pts[i].z);
        double xi = p.dot(e1); 
        double zi = p.z(); 

        A(i, 0) = xi * xi;
        A(i, 1) = xi;
        A(i, 2) = 1.0;
        Z(i)     = zi;
    }

    Eigen::Vector3d sol = A.colPivHouseholderQr().solve(Z);
    
    para_a = sol(0);
    para_b = sol(1);
    para_c = sol(2);

    return true;
}

double ParabolaFitter::predictZ(double x, double y) const {
    if (!fitted) throw std::runtime_error("ParabolaFitter: Model not fitted.");

    Eigen::Vector3d e1(0, 0, 0);
    if (vertical_line) {
        e1 = {0, 1, 0};
    } else {
        Eigen::Vector3d v_xy(1.0, line_k, 0.0);
        v_xy.normalize();
        e1 = v_xy;
    }

    Eigen::Vector3d p(x, y, 0);
    double xi = p.dot(e1);

    return para_a * xi * xi + para_b * xi + para_c;
}

std::vector<std::pair<double, double>> ParabolaFitter::solveXYforZ(double z_query) const {
    if (!fitted) throw std::runtime_error("ParabolaFitter: Model not fitted.");

    std::vector<std::pair<double, double>> results;

    double c_prime = para_c - z_query;
    double delta = para_b * para_b - 4.0 * para_a * c_prime;

    if (delta < -1e-9) { 
        return results; 
    }

    std::vector<double> xi_solutions;
    if (std::abs(para_a) < 1e-9) {
        if (std::abs(para_b) > 1e-9) {
            xi_solutions.push_back(-c_prime / para_b);
        }
    } else {
        if (delta < 0) delta = 0; 
        double sqrt_delta = std::sqrt(delta);
        xi_solutions.push_back((-para_b + sqrt_delta) / (2.0 * para_a));
        xi_solutions.push_back((-para_b - sqrt_delta) / (2.0 * para_a));
    }

    // 还原回世界坐标
    Eigen::Vector3d e1;
    Eigen::Vector3d p0(0,0,0); 

    if (vertical_line) {
        e1 = {0, 1, 0};
        p0 = {line_b, 0, 0}; 
    } else {
        Eigen::Vector3d v_xy(1.0, line_k, 0.0);
        double norm = v_xy.norm();
        e1 = v_xy / norm;

        double denom = line_k * line_k + 1.0;
        p0.x() = -(line_k * line_b) / denom;
        p0.y() = line_b / denom;
        p0.z() = 0.0;
    }

    for (double xi : xi_solutions) {
        Eigen::Vector3d p_world = p0 + xi * e1;
        results.push_back({ p_world.x(), p_world.y() });
    }

    return results;
}