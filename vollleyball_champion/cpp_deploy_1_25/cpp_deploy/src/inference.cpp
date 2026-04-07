#include "../include/inference.h"
#include "../include/preprocess.h" // 必须包含，用于调用 CUDA Kernel
#include <fstream>
#include <iostream>
#include <algorithm> // for std::min
#include <cuda_runtime_api.h>

// 简单的日志类
class Logger : public ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        // 只打印警告和错误
        if (severity <= Severity::kWARNING) std::cout << "[TRT] " << msg << std::endl;
    }
} logger;

YOLOInference::YOLOInference(const std::string& engine_path) {
    loadEngine(engine_path);
    
    // 计算 TensorRT 输入输出所需的显存大小
    // 输入: float 3x640x640 (Planar RGB)
    size_t input_bytes = 1 * 3 * INPUT_W * INPUT_H * sizeof(float);
    // 输出: float 1x(4+cls)x8400
    size_t output_bytes = 1 * OUTPUT_CHANNELS * OUTPUT_CANDIDATES * sizeof(float);

    // 1. 申请 TensorRT 推理用的 GPU 显存
    cudaMalloc(&buffers[0], input_bytes);
    cudaMalloc(&buffers[1], output_bytes);

    // 2. 申请 CPU 锁页内存 (只用于接收输出结果)
    // 使用 cudaMallocHost (Pinned Memory) 加速 D2H 传输
    cudaMallocHost((void**)&output_buffer_host, output_bytes);

    // 3. 申请 GPU 端“原图缓冲区”
    // 为了适应不同分辨率的输入，分配一个足够大的显存 (例如支持到 1080p)
    // 1920 * 1080 * 3 = 6,220,800 bytes (~6MB)
    size_t max_src_size = 1920 * 1080 * 3 * sizeof(uint8_t);
    cudaMalloc(&input_src_device, max_src_size);

    // 创建 CUDA 流
    cudaStreamCreate(&stream);
}

YOLOInference::~YOLOInference() {
    // 释放 GPU 显存
    cudaFree(buffers[0]);
    cudaFree(buffers[1]);
    cudaFree(input_src_device);
    
    // 释放 CPU 锁页内存
    cudaFreeHost(output_buffer_host);
    
    // 销毁流和 TensorRT 对象
    cudaStreamDestroy(stream);
    delete context;
    delete engine;
    delete runtime;
}

void YOLOInference::loadEngine(const std::string& engine_path) {
    std::ifstream file(engine_path, std::ios::binary);
    if (!file.good()) {
        std::cerr << "CRITICAL ERROR: Could not read engine file: " << engine_path << std::endl;
        exit(-1);
    }
    file.seekg(0, file.end);
    size_t size = file.tellg();
    file.seekg(0, file.beg);
    char* trtModelStream = new char[size];
    file.read(trtModelStream, size);
    file.close();

    runtime = createInferRuntime(logger);
    engine = runtime->deserializeCudaEngine(trtModelStream, size);
    context = engine->createExecutionContext();
    delete[] trtModelStream;
}

// =========================================================================
// 核心优化函数：GPU 预处理 (Letterbox)
// =========================================================================
void YOLOInference::preprocess(cv::Mat& img) {
    int src_w = img.cols;
    int src_h = img.rows;
    int src_step = img.step; // OpenCV 每一行的字节数
    size_t img_data_size = src_step * src_h;

    // 1. 将原始图像数据 (uint8, BGR) 极速上传到 GPU
    cudaMemcpyAsync(input_src_device, img.data, img_data_size, cudaMemcpyHostToDevice, stream);

    // 2. 计算 Letterbox 参数 (保持长宽比)
    // 目标尺寸 INPUT_W x INPUT_H (通常是 640x640)
    float r = std::min((float)INPUT_W / src_w, (float)INPUT_H / src_h);
    
    // 缩放后的有效区域宽高
    int new_unpad_w = (int)(src_w * r);
    int new_unpad_h = (int)(src_h * r);

    // 计算 Padding 偏移量 (居中放置)
    int dw = (INPUT_W - new_unpad_w) / 2;
    int dh = (INPUT_H - new_unpad_h) / 2;

    // --- 保存参数到类成员，供 postprocess 使用 ---
    this->cur_scale = r;
    this->cur_dw = dw;
    this->cur_dh = dh;

    // 3. 启动 CUDA 核函数
    // 在 GPU 上一次性完成: Letterbox Resize + Pad + Normalize + HWC2CHW
    launch_preprocess_kernel(
        input_src_device,       // 源数据 (GPU)
        (float*)buffers[0],     // 目标 Tensor (GPU)
        src_w, src_h, src_step, // 原图参数
        INPUT_W, INPUT_H,       // 目标尺寸
        r, dw, dh,              // Letterbox 变换参数
        stream
    );
}

// =========================================================================
// 后处理：解析 TensorRT 输出并还原坐标
// =========================================================================
void YOLOInference::postprocess(std::vector<std::vector<float>>& results) {
    std::vector<cv::Rect> boxes;
    std::vector<float> confs;
    std::vector<int> class_ids;

    // TensorRT 输出通常是平铺的 Planar 格式
    // 布局假设: [1, 5, 8400] -> [cx, cy, w, h, conf]
    // 指针偏移量计算
    float* p_data = output_buffer_host;
    
    float* p_cx   = p_data;
    float* p_cy   = p_data + OUTPUT_CANDIDATES;
    float* p_w    = p_data + 2 * OUTPUT_CANDIDATES;
    float* p_h    = p_data + 3 * OUTPUT_CANDIDATES;
    float* p_conf = p_data + 4 * OUTPUT_CANDIDATES;

    for (int i = 0; i < OUTPUT_CANDIDATES; i++) {
        float confidence = p_conf[i];
        
        if (confidence > CONF_THRESHOLD) {
            float cx = p_cx[i];
            float cy = p_cy[i];
            float w  = p_w[i];
            float h  = p_h[i];

            // === 坐标还原 (Letterbox Reverse) ===
            
            // 1. 去除 Padding 偏移
            float cx_unpad = cx - cur_dw;
            float cy_unpad = cy - cur_dh;

            // 2. 去除缩放 (还原回原图尺度)
            float cx_src = cx_unpad / cur_scale;
            float cy_src = cy_unpad / cur_scale;
            float w_src  = w / cur_scale;
            float h_src  = h / cur_scale;

            // 3. 转为左上角坐标 (x, y)
            int x = int(cx_src - w_src * 0.5f);
            int y = int(cy_src - h_src * 0.5f);

            boxes.push_back(cv::Rect(x, y, int(w_src), int(h_src)));
            confs.push_back(confidence);
            class_ids.push_back(0); // 目前只有排球一类
        }
    }

    // NMS (非极大值抑制)
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confs, CONF_THRESHOLD, NMS_THRESHOLD, indices);

    for (int idx : indices) {
        std::vector<float> det;
        det.push_back((float)class_ids[idx]);   // Class ID
        det.push_back(confs[idx]);              // Confidence
        
        // =========================================================
        // [修正] 计算中心点坐标，而不是返回左上角
        // main.cpp 中使用 det[2] 和 det[3] 作为球心进行射线解算
        // =========================================================
        float center_x = boxes[idx].x + boxes[idx].width * 0.5f;
        float center_y = boxes[idx].y + boxes[idx].height * 0.5f;
        
        det.push_back(center_x);              // Center X
        det.push_back(center_y);              // Center Y
        det.push_back((float)boxes[idx].width); // w
        det.push_back((float)boxes[idx].height);// h
        results.push_back(det);
    }
}

std::vector<std::vector<float>> YOLOInference::infer(cv::Mat& img) {
    // 1. GPU 预处理 (上传 + Letterbox Kernel)
    preprocess(img);

    // 2. TensorRT 推理 (GPU)
    context->executeV2(buffers);

    // 3. 结果回传 (GPU -> CPU Pinned Memory)
    cudaMemcpyAsync(output_buffer_host, buffers[1], OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream);
    
    // 4. 等待所有 GPU 操作完成
    cudaStreamSynchronize(stream);

    // 5. CPU 后处理 (NMS + 坐标还原)
    std::vector<std::vector<float>> results;
    postprocess(results);
    return results;
}