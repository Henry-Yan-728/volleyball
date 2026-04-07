#pragma once
#include <cuda_runtime.h>
#include <cstdint>

void launch_preprocess_kernel(
    uint8_t* src, float* dst, 
    int src_width, int src_height, int src_step,
    int dst_width, int dst_height,
    float scale, int dx, int dy,
    cudaStream_t stream
);