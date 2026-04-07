/**
  ******************************************************************************
  * @file           : cybergear_motor.h
  * @brief          : 灏忕背 CyberGear 鐢垫満鎺у埗椹卞姩鎺ュ彛鏂囦欢
  ******************************************************************************
  * @attention
  *
  * 1. 鏈ā鍧楀皝瑁呬簡瀵瑰皬绫矯yberGear绯诲垪寰數鏈虹殑CAN鎬荤嚎鎺у埗銆?
  * 2. 鏀寔澶氱鎺у埗妯″紡锛屽寘鎷綅缃€侀€熷害銆佸姏鐭╀互鍙婇珮鏁堢殑杩愭帶妯″紡銆?
  * 3. 鎻愪緵浜嗕富鍔ㄨ疆璇㈢數鏈虹壒瀹氬弬鏁帮紙濡傝搴︺€佺數鍘嬶級鐨勫姛鑳斤紝浠ユ敮鎸侀珮棰戠姸鎬佺洃鎺с€?
  * 4. 浣跨敤鍓嶅繀椤荤‘淇?`fdcan_bsp` 妯″潡宸茬粡姝ｇ‘绉绘骞跺彲鐢ㄣ€?
  * 5. 鎵€鏈夌數鏈哄疄渚嬪瓨鍌ㄥ湪涓€涓潤鎬佸叏灞€鏁扮粍涓紝閫氳繃绱㈠紩杩涜璁块棶銆?
  *
  ******************************************************************************
  */

#ifndef CYBERGEAR_MOTOR_H
#define CYBERGEAR_MOTOR_H

#include "main.h"
#include <math.h>

// 瀹氫箟椤圭洰涓?CyberGear 鐢垫満鐨勬€绘暟閲?
#define CYBERGEAR_MOTOR_COUNT 1 

// 瀹氫箟涓绘満ID锛屼笌fdcan_bsp娉ㄥ唽鏃朵繚鎸佷竴鑷?
#define MASTER_CAN_ID 0


// 鏍规嵁璇存槑涔?4.1.3 鐢垫満鍙嶉鏁版嵁 瀹氫箟
#define P_MIN (-4.0f * M_PI) // 瑙掑害涓嬮檺 -4蟺 rad
#define P_MAX (4.0f * M_PI)  // 瑙掑害涓婇檺 4蟺 rad
#define V_MIN (-30.0f)       // 瑙掗€熷害涓嬮檺 -30 rad/s
#define V_MAX (30.0f)        // 瑙掗€熷害涓婇檺 30 rad/s
#define T_MIN (-12.0f)       // 鍔涚煩涓嬮檺 -12 N.m
#define T_MAX (12.0f)        // 鍔涚煩涓婇檺 12 N.m

// 鏍规嵁璇存槑涔﹀拰鍘熶緥绋?CanCommand.h 娣诲姞
#define KP_MIN (0.0f)
#define KP_MAX (500.0f)
#define KD_MIN (0.0f)
#define KD_MAX (5.0f)

/**
 * @brief 鐢垫満杩愯妯″紡鍙嶉鏋氫妇 (鏉ヨ嚜CAN鎶ユ枃)
 */
typedef enum {
    MOTOR_RESET_MODE = 0, // Reset 妯″紡 [澶嶄綅] 
    MOTOR_CALI_MODE  = 1, // Cali 妯″紡 [鏍囧畾] 
    MOTOR_RUN_MODE   = 2, // Motor 妯″紡 [杩愯] 
} CyberGear_Motor_Mode_e;

/**
 * @brief 鐢垫満鎺у埗妯″紡鏋氫妇
 */
typedef enum {
	  MOTOR_CONTROL_MODE_MOTION = 0,    // 杩愭帶妯″紡
    MOTOR_CONTROL_MODE_POSITION,  		// 浣嶇疆妯″紡
    MOTOR_CONTROL_MODE_SPEED,     		// 閫熷害妯″紡
    MOTOR_CONTROL_MODE_TORQUE,    		// 鍔涚煩(鐢垫祦)妯″紡
		MOTOR_CONTROL_MODE_UNSET, 				// 鏈瀹氭垨鏈煡妯″紡
} CyberGear_Control_Mode_e;

/**
 * @brief 鍙緵涓诲姩璇锋眰璇诲彇鐨勭數鏈哄弬鏁板湴鍧€鏋氫妇
 * @details 鐢ㄤ簬 cybergear_motor_request_parameter() 鍑芥暟
 */
typedef enum {
    PARAM_MECH_POS   = 0x7019, // 璐熻浇绔鍦堟満姊拌搴?(rad), float
    PARAM_IQ_FILTER  = 0x701A, // iq 婊ゆ尝鍊?(A), float
    PARAM_MECH_VEL   = 0x701B, // 璐熻浇绔浆閫?(rad/s), float
    PARAM_VBUS       = 0x701C, // 姣嶇嚎鐢靛帇 (V), float
    PARAM_ROTATION   = 0x701D, // 鍦堟暟, int16_t
} CyberGear_Param_Index_e;

/**
 * @brief 鐢垫満娴嬮噺鍊间笌鐘舵€佺粨鏋勪綋 (鏉ヨ嚜鐢佃皟鍙嶉)
 */
typedef struct {
    float angle;         // 璐熻浇绔鍦堟満姊拌搴? 鍗曚綅 rad
    float speed;         // 璐熻浇绔浆閫? 鍗曚綅 rad/s
    float torque;        // 鍙嶉鍔涚煩 (N.m) - 娉ㄦ剰: 杩欎釜鍙兘鐢辩被鍨?鎶ユ枃鏇存柊
    float temperature;   // 鐢垫満娓╁害 (掳C) - 娉ㄦ剰: 杩欎釜涔熷彧鑳界敱绫诲瀷2鎶ユ枃鏇存柊

    // 杩欎簺鍊煎彲浠ョ敱绫诲瀷17鐨勬姤鏂囨洿鏂?
    float vbus;          // 姣嶇嚎鐢靛帇 (V)
    int16_t rotation;    // 绱鍦堟暟

    CyberGear_Motor_Mode_e mode; // 褰撳墠鐢垫満鎵€澶勭殑妯″紡 
    uint8_t fault;               // 鏁呴殰淇℃伅 
} motor_measure_cybergear_t;

/**
 * @brief CyberGear 鐢垫満瀹炰緥鐨勬€荤粨鏋勪綋
 */
typedef struct CyberGear_Motor_Instance_t {
    motor_measure_cybergear_t measure;      // 鐢垫満鐨勬祴閲忓€煎拰鐘舵€?
    // 褰撳墠鎺у埗妯″紡
    // 娉ㄦ剰璇ュ弬鏁颁粎鍦ㄤ富鎺т晶缁存姢锛岀敤浜庤窡韪綋鍓嶇殑鎺у埗妯″紡锛屼笉浼氱敱鐢垫満鍙嶉鏇存柊
    CyberGear_Control_Mode_e control_mode;  

    // --- 浠ヤ笅鏄笌 bsp 妗嗘灦闆嗘垚鎵€闇€鐨勪俊鎭?---
    uint8_t  can_id;                // 璇ョ數鏈虹殑鍙嶉ID (e.g., 1)
    uint8_t  is_online;             // 鍦ㄧ嚎鐘舵€佹爣蹇?
    uint32_t last_msg_time;         // 涓婃鏀跺埌鎶ユ枃鐨凥AL_GetTick()鏃堕棿鎴?
    FDCAN_HandleTypeDef* hfdcan;    // 璇ョ數鏈虹粦瀹氱殑FDCAN鍙ユ焺

} CyberGear_Motor_Instance;


/*************************** Public Functions ***************************/

float cybergear_motor_rpm2rad(int16_t rpm);
float cybergear_motor_degree2rad(float degree);
void cybergear_motors_init(void);
CyberGear_Motor_Instance* cybergear_motor_get_instance(uint8_t motor_index);
void cybergear_motor_enable(CyberGear_Motor_Instance* motor);
void cybergear_motor_stop(CyberGear_Motor_Instance* motor);
void cybergear_motor_set_mode(CyberGear_Motor_Instance* motor, CyberGear_Control_Mode_e mode);
void cybergear_motor_set_zero_position(CyberGear_Motor_Instance* motor);
void cybergear_motor_set_position(CyberGear_Motor_Instance* motor, float position, float speed_limit);
void cybergear_motor_set_speed(CyberGear_Motor_Instance* motor, float speed);
void cybergear_motor_set_torque(CyberGear_Motor_Instance* motor, float torque);
void cybergear_motor_set_motion_control(CyberGear_Motor_Instance* motor, float position, float speed, float kp, float kd, float torque_ff);
void cybergear_motor_set_pid_gains(CyberGear_Motor_Instance* motor, float loc_kp, float spd_kp, float spd_ki);
void cybergear_motor_set_current_limit(CyberGear_Motor_Instance* motor, float current_limit);
void cybergear_motor_request_parameter(CyberGear_Motor_Instance* motor, CyberGear_Param_Index_e param_index);

/*************************** Internal Functions (for fdcan_bsp) ***************************/
void cybergear_message_handler(void* instance, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[8]);

#endif // CYBERGEAR_MOTOR_H

