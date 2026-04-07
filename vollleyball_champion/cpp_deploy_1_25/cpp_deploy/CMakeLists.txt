cmake_minimum_required(VERSION 3.10)
project(yolo_trt_cpp LANGUAGES CXX CUDA) # 启用 CUDA 语言支持

# =========================================================
# 1. 基础编译选项
# =========================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)

# 注意：如果你的显卡不是 RTX 40/50 系，这里填 89 会导致运行报错
# 建议改为 native 让编译器自动检测，或者根据实际显卡查表填写
set(CMAKE_CUDA_ARCHITECTURES 89) 

# 忽略过时警告（针对 TensorRT 10.x 接口变动）
add_compile_options(-Wno-deprecated-declarations)

# =========================================================
# 2. 依赖库查找 (关键修正区域)
# =========================================================
set(TRT_ROOT "/home/robot/TensorRT-10.14.1.48") 

# [修正 1] 必须显式查找 Eigen3，否则报 fatal error: Eigen/Dense
find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

# =========================================================
# 3. 头文件路径 (关键修正区域)
# =========================================================
include_directories(
    # [修正 2] 必须包含项目自己的 include 目录，否则报 parabola_fitter_clib.hpp 找不到
    ${PROJECT_SOURCE_DIR}/include
    
    # [修正 3] 必须包含 Eigen3 的头文件路径
    ${EIGEN3_INCLUDE_DIR}
    
    "${TRT_ROOT}/include"
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
)

# =========================================================
# 4. 库文件路径
# =========================================================
link_directories(
    "${TRT_ROOT}/lib"
    "/usr/local/cuda/lib64"
)

# =========================================================
# 5. 构建目标
# =========================================================
# 搜集源文件 (.cpp, .cu, .c)
file(GLOB SOURCES "src/*.cpp" "src/*.cu" "src/*.c")

add_executable(yolo_app ${SOURCES})

# 链接库
target_link_libraries(yolo_app 
    ${OpenCV_LIBS} 
    ${CUDA_LIBRARIES} 
    nvinfer 
    cudart
    pthread
    # 注意：Eigen 是 header-only 库，通常不需要 link，但如果需要可以使用 Eigen3::Eigen
)