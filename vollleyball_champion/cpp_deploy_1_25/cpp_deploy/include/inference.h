#pragma once
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "NvInfer.h"

// 命名空间引用
using namespace nvinfer1;

class YOLOInference {
public:
    // 构造函数：传入 engine 路径
    YOLOInference(const std::string& engine_path);
    ~YOLOInference();

    // 核心接口：推理单帧，返回检测框
    // 返回格式: [class_id, confidence, x, y, w, h]
    std::vector<std::vector<float>> infer(cv::Mat& img);

private:
    void loadEngine(const std::string& engine_path);
    void preprocess(cv::Mat& img);
    void postprocess(std::vector<std::vector<float>>& results);
    // 在 private: 下面添加
    float cur_scale = 1.0f;
    int cur_dw = 0;
    int cur_dh = 0;
    // TensorRT 核心指针
    IRuntime* runtime = nullptr;
    ICudaEngine* engine = nullptr;
    IExecutionContext* context = nullptr;
    uint8_t* input_src_device = nullptr; 
    size_t input_src_size = 0;
    // 内存管理
    void* buffers[2]; // 0: Input(GPU), 1: Output(GPU)
    float* output_buffer_host = nullptr; // CPU端接收结果
    float* input_buffer_host = nullptr;  // CPU端预处理数据
    cudaStream_t stream;

    // 模型参数 (必须与 export_trt.py 里的设置一致)
    const int INPUT_W = 640;
    const int INPUT_H = 640;
    const int NUM_CLASSES = 1;  // 只有排球一类
    // YOLOv8/11 输出尺寸: 1 x (4+classes) x 8400
    const int OUTPUT_CHANNELS = 4 + NUM_CLASSES; 
    const int OUTPUT_CANDIDATES = 8400; 
    const int OUTPUT_SIZE = OUTPUT_CHANNELS * OUTPUT_CANDIDATES;
    
    // 阈值
    const float CONF_THRESHOLD = 0.5f;
    const float NMS_THRESHOLD = 0.45f;
};