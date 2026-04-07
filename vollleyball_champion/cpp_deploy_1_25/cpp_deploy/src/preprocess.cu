#include "cuda_runtime.h"
#include <cstdint>

// CUDA Kernel: Letterbox Resize + Normalize + HWC->CHW + BGR->RGB
// src: 原图 (Device Memory, uint8, BGR)
// dst: 目标 Tensor (Device Memory, float, RGB, Planar)
// scale: 缩放比例
// dx, dy: 目标图像上的 x, y 偏移量 (用于居中)
__global__ void preprocess_kernel(
    uint8_t* src, float* dst, 
    int src_width, int src_height, int src_step,
    int dst_width, int dst_height,
    float scale, int dx, int dy
) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= dst_width || y >= dst_height) return;

    // 1. 目标坐标 -> 原图坐标映射
    // x_dst = x_src * scale + dx  ==>  x_src = (x_dst - dx) / scale
    float src_x_f = (x - dx) / scale;
    float src_y_f = (y - dy) / scale;

    // 最近邻插值 (Nearest Neighbor) - 速度最快
    // 对于高分辨原图下采样，最近邻通常足够。若需更高精度可用双线性。
    int src_x = (int)src_x_f;
    int src_y = (int)src_y_f;

    // 目标索引 (CHW)
    int area = dst_width * dst_height;
    int dst_idx = y * dst_width + x;

    // 2. 边界判断与填充
    if (src_x >= 0 && src_x < src_width && src_y >= 0 && src_y < src_height) {
        // 在原图范围内：读取并归一化
        int src_idx = src_y * src_step + src_x * 3;
        
        uint8_t b = src[src_idx + 0];
        uint8_t g = src[src_idx + 1];
        uint8_t r = src[src_idx + 2];

        // BGR -> RGB & /255.0
        dst[dst_idx]            = r / 255.0f; // R
        dst[dst_idx + area]     = g / 255.0f; // G
        dst[dst_idx + 2 * area] = b / 255.0f; // B
    } else {
        // padding 区域：填充 114 (YOLO 标准灰色)
        float pad_val = 114.0f / 255.0f;
        dst[dst_idx]            = pad_val;
        dst[dst_idx + area]     = pad_val;
        dst[dst_idx + 2 * area] = pad_val;
    }
}

void launch_preprocess_kernel(
    uint8_t* src, float* dst, 
    int src_width, int src_height, int src_step,
    int dst_width, int dst_height,
    float scale, int dx, int dy,
    cudaStream_t stream
) {
    dim3 block(32, 32);
    dim3 grid((dst_width + block.x - 1) / block.x, (dst_height + block.y - 1) / block.y);

    preprocess_kernel<<<grid, block, 0, stream>>>(
        src, dst, 
        src_width, src_height, src_step, 
        dst_width, dst_height,
        scale, dx, dy
    );
}