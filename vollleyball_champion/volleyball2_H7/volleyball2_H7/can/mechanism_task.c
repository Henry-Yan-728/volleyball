#include "mechanism_task.h"
#include "dji_motor.h"
#include "cmsis_os.h"
#include <math.h>
#include "Pan_Tilt_control.h"
#include "unitree_motor.h"
#include "cybergear_motor.h"

// 辅助宏
#ifndef M_PI
#define M_PI 3.1415926535f
#endif

#define DEG_TO_RAD_FACTOR (M_PI / 180.0f)

// 初始化：起始0度，目标0度，速度 0.1度/ms 
Virtual_Axis_t v_axis = {0.0f, 0.0f, 0.1f, 0}; 

float current_yaw_speed = 0;
float current_pitch_speed = 0;
static CyberGear_Motor_Instance* g_pitch_motor = NULL;
static uint8_t g_pitch_motor_ready = 0;
// -------------------------------------------------------------
//  私有辅助函数: 角度(度) -> 宇树电机弧度
//  Note: 宇树电机通常需要输入 轴前弧度 * 减速比
// -------------------------------------------------------------
static float convert_deg_to_unitree_rad(float deg) {
    return deg * DEG_TO_RAD_FACTOR * UNITREE_REDUCTION_RATIO;
}

void Mechanism_Init(void)
{
    dji_motors_init();
    cybergear_motors_init();
    gimbal_control_init();
    HAL_Delay(100);
}
// =============================================================
//  垫球机构控制 (Position Control)
// =============================================================
void Mechanism_Cushion_SetAngle(float angle_deg, float kp)
{
    float target_rad = convert_deg_to_unitree_rad(angle_deg);
    float kd_default = 0.2f; 
    
    // 使用纯净的驱动层接口下发指令
    UnitreeMotor_SetPosition(UNITREE_ID_CUSHION_1, target_rad, kp, kd_default);
    UnitreeMotor_SetPosition(UNITREE_ID_CUSHION_2, target_rad, kp, kd_default);
    UnitreeMotor_SetPosition(UNITREE_ID_CUSHION_3, target_rad, kp, kd_default);
}

void Mechanism_Zero()
{
    MotorData_t rx_data;
    // 下发 0 刚度获取反馈或清空当前状态（根据实际需求调整）
    UnitreeMotor_SetPosition(UNITREE_ID_CUSHION_1, 0.0f, 0.0f, 0.1f);
    UnitreeMotor_ReceiveData(&rx_data, 2);
    UnitreeMotor_SetPosition(UNITREE_ID_CUSHION_2, 0.0f, 0.0f, 0.1f);
    UnitreeMotor_ReceiveData(&rx_data, 2);
    UnitreeMotor_SetPosition(UNITREE_ID_CUSHION_3, 0.0f, 0.0f, 0.1f);
    UnitreeMotor_ReceiveData(&rx_data, 2);
    UnitreeMotor_SetPosition(UNITREE_ID_SERVE, 0.0f, 0.0f, 0.1f);
    UnitreeMotor_ReceiveData(&rx_data, 2);
}

// =============================================================
//  发球机构控制 (Velocity/Position Hybrid)
// =============================================================
void Mechanism_Serve_SetAngle(float angle_deg)
{
    float target_val = convert_deg_to_unitree_rad(angle_deg);
    float kp = 1.0f;
    float kd = 0.1f; 
    
    // 使用纯净的驱动层接口下发指令
    UnitreeMotor_SetPosition(UNITREE_ID_SERVE, target_val, kp, kd);
}
// =============================================================
//  俯仰机构控制 (DJI Position Control)
// =============================================================
void Mechanism_Dian_Pitch_SetAngle(float angle_deg)
{
	v_axis.target_angle = angle_deg;
}

/* ----------------- 2. 轨迹更新函数 ----------------- */
void Update_Virtual_Axis(void)
{
    uint32_t now = HAL_GetTick();
    
    if (now - v_axis.last_tick < 1) return; 
    v_axis.last_tick = now;

    // --- 轨迹插补算法 (Ramp) ---
    float error = v_axis.target_angle - v_axis.current_angle;
    
    if (fabs(error) <= v_axis.velocity_deg_ms) {
        v_axis.current_angle = v_axis.target_angle;
    }
    else {
        if (error > 0) v_axis.current_angle += v_axis.velocity_deg_ms;
        else           v_axis.current_angle -= v_axis.velocity_deg_ms;
    }

    // --- 同步分发给物理电机 ---
    DJI_Motor_Instance* m_right = dji_motor_get_instance(0); // ID 0x201
    DJI_Motor_Instance* m_left  = dji_motor_get_instance(1); // ID 0x202

    if (m_right && m_left)
    {
        // 规范化转化: 角度 -> 编码器值
        // 使用宏 EXTERNAL_MECHANISM_RATIO (19.203 ) 
        int32_t encoder_pos = dji_angle_to_encoder(v_axis.current_angle, EXTERNAL_MECHANISM_RATIO);

        // 左电机：正向旋转
        dji_motor_set_location(m_left, encoder_pos);
        
        // 右电机：反向旋转 (同轴异向)
        dji_motor_set_location(m_right, -encoder_pos);
    }
}

void Mechanism_Loop_1ms(void)
{
    gimbal_set_speed((int16_t)current_yaw_speed, 0);

    if (g_pitch_motor == NULL)
    {
        g_pitch_motor = cybergear_motor_get_instance(0);
    }

    if (g_pitch_motor != NULL && !g_pitch_motor_ready)
    {
        cybergear_motor_set_mode(g_pitch_motor, MOTOR_CONTROL_MODE_SPEED);
        cybergear_motor_enable(g_pitch_motor);
        cybergear_motor_set_speed(g_pitch_motor, 0.0f);
        g_pitch_motor_ready = 1;
    }

    if (g_pitch_motor != NULL && g_pitch_motor_ready)
    {
        float pitch_rad_s = cybergear_motor_rpm2rad((int16_t)current_pitch_speed);
        cybergear_motor_set_speed(g_pitch_motor, pitch_rad_s);
    }
}
