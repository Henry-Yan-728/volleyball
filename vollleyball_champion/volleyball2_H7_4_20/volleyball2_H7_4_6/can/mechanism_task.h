#ifndef MECHANISM_TASK_H
#define MECHANISM_TASK_H

#include "main.h"
#include "unitree_motor.h"

// =============================================================
//  核心机械参数配置
// =============================================================
// 宇树电机减速比 (原代码中的 6.33)
#define UNITREE_REDUCTION_RATIO  6.33f  

// 外部机械臂/滑轨传动比 
// 含义：电机(M3508 19.2:1) 
#define EXTERNAL_MECHANISM_RATIO (19.2032f*3)

// =============================================================
//  电机 ID 定义
//  垫球机构的 3 个电机共用同一个 RS485 ID
//  发球机构使用独立 ID
// =============================================================
#define UNITREE_ID_CUSHION_1  15
#define UNITREE_ID_SERVE      0

#define DJI_INDEX_AXIS_MASTER  4
#define DJI_INDEX_AXIS_SLAVE   5

// 全局控制目标值:
// yaw: 速度指令 (RPM)
// pitch: 目标角度 (deg)
extern volatile float current_yaw_speed;
extern volatile float current_pitch_target_deg;

typedef struct {
    float current_angle;    // 虚拟轴当前的实时角度 (度)
    float target_angle;     // 最终想要到达的角度 (度)
    float velocity_deg_ms;  // 速度：每毫秒转多少度 
    uint32_t last_tick;     // 上次更新的时间戳
} Virtual_Axis_t;

typedef enum {
    START = 0,
    OVER,
    START_BUSY,
    OVER_BUSY,
    SYS_ERROR_SENSOR_JAM,
    SYS_ERROR_MOTOR_COMMS
} PC_state;

void Set_System_Doit_State(PC_state new_state);
PC_state Get_System_Doit_State(void);

typedef struct {
    uint8_t init_state;
    uint8_t request_pending;
    uint8_t action_state;
    uint8_t retry_count;
    uint32_t init_tick;
    uint32_t action_tick;
} MechanismServeController_t;

// =============================================================
//  函数声明
// =============================================================

void Mechanism_Init(void);

/**
 * @brief 控制垫球机构 (3个宇树电机同步运动)
 * @param angle_deg: 目标角度 (度)
 * @param kp: 刚度系数 
 */
UnitreeMotorLinkResult_t Mechanism_Cushion_SetAngle(float angle_deg, float kp);

/**
 * @brief 控制发球机构 (1个宇树电机)
 * @param angle_deg: 目标值 (度制，根据模式不同可能为位置或速度)
 */
UnitreeMotorLinkResult_t Mechanism_Serve_SetAngle(float angle_deg);
UnitreeMotorLinkResult_t Mechanism_Serve_SetAngle_back(float angle_deg);

/**
 * @brief 控制俯仰机构 
 */
void Mechanism_Dian_Pitch_SetAngle(float angle_deg);

void Mechanism_Zero(void);

void Update_Virtual_Axis(void);

void Mechanism_Loop_1ms(void); 

void Mechanism_ServeController_Init(MechanismServeController_t *controller, uint32_t now_tick);
void Mechanism_ServeController_Request(MechanismServeController_t *controller);
void Mechanism_ServeController_Cancel(MechanismServeController_t *controller);
uint8_t Mechanism_ServeController_IsReady(const MechanismServeController_t *controller);
uint8_t Mechanism_ServeController_IsIdle(const MechanismServeController_t *controller);
void Mechanism_ServeController_Process(MechanismServeController_t *controller, uint32_t now_tick,
                                       float cushion_kp);
void Mechanism_BallDetect_Process(float cushion_kp);

#endif // MECHANISM_TASK_H
