from ultralytics import YOLO
import torch
import struct
import os

# ================= 配置 =================
MODEL_PATH = "best.pt"
ENGINE_PATH = "best.engine"
# ========================================

def strip_ultralytics_header(engine_path):
    """
    移除 Ultralytics 强加在 TensorRT engine 前面的 metadata 头。
    C++ TensorRT 接口只认以 'trtf' 开头的纯二进制数据。
    """
    print(f"-> Inspecting {engine_path} for Ultralytics header...")
    
    with open(engine_path, "rb") as f:
        # 1. 读取前4个字节（Little Endian Unsigned Int），这是元数据长度
        magic_len_bytes = f.read(4)
        if not magic_len_bytes:
            print("Error: File is empty.")
            return

        # 尝试解析长度
        meta_len = struct.unpack('<I', magic_len_bytes)[0]
        
        # 2. 检查这是否真的是 Ultralytics 的头
        # 真正的 TRT engine 开头是 'trtf' (0x74727466)
        # 如果前4个字节直接就是 trtf，说明没有头，不需要处理
        f.seek(0)
        first_4_bytes = f.read(4)
        if first_4_bytes == b'trtf':
            print("-> Good news: This file is already a raw TRT engine. No change needed.")
            return

        print(f"-> Detected Metadata Header Length: {meta_len} bytes")
        
        # 3. 验证去除头之后，是不是 trtf
        f.seek(4 + meta_len)
        check_magic = f.read(4)
        if check_magic != b'trtf':
            print(f"CRITICAL WARNING: Offset {4+meta_len} is not 'trtf'. Found {check_magic}. Extraction might fail!")
        else:
            print(f"-> Verification passed: Found 'trtf' magic tag at offset {4+meta_len}.")

        # 4. 读取剩余的所有数据（Raw Engine）
        f.seek(4 + meta_len)
        raw_data = f.read()

    # 5. 覆盖写回原文件
    # 这一步把纯净的 engine 写回去，替换掉带头的文件
    with open(engine_path, "wb") as f_out:
        f_out.write(raw_data)
        
    print(f"-> Success! Header stripped. {engine_path} is now C++ ready.")
    print(f"-> Final size: {len(raw_data) / 1024 / 1024:.2f} MB")


if __name__ == "__main__":
    # 1. 检查 GPU
    if not torch.cuda.is_available():
        raise SystemError("CRITICAL: No GPU found. TensorRT requires NVIDIA GPU.")

    # 2. 加载模型
    print(f"Loading {MODEL_PATH}...")
    model = YOLO(MODEL_PATH)

    # 3. 导出 (Ultralytics 标准导出)
    # dynamic=False 对高频控制至关重要，固定尺寸推理最快
    print("Starting TensorRT export...")
    model.export(format="engine", half=False, device=0, dynamic=False)

    # 4. 执行手术 (关键步骤)
    if os.path.exists(ENGINE_PATH):
        strip_ultralytics_header(ENGINE_PATH)
    else:
        print(f"Error: {ENGINE_PATH} not found after export.")