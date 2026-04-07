/**
  ******************************************************************************
  * @file           : cybergear_motor.c
  * @brief          : 灏忕背 CyberGear 鐢垫満鎺у埗椹卞姩瀹炵幇鏂囦欢銆?
  * @author         : 绉︽辰瀹?& Gemini
  * @date           : 2025-10-18
  * @version        : v1.2
  * @note           :
  * - v1.2 (2025-10-20): 鏂板瀵笷reeRTOS鐨勯€傞厤鍗囩骇锛屼娇鐢ㄦ潯浠剁紪璇戜笌瑁告満鍖哄垎銆?
  * - v1.1 (2025-10-19): 鏂板鐢垫満妯″紡璁剧疆鍑芥暟锛屾柊澧為€熷害/浣嶇疆妯″紡鐢垫祦闄愬埗璁剧疆鍑芥暟锛屾柊澧?
  *                      璁剧疆鏈烘闆朵綅鍑芥暟銆?
  ******************************************************************************
  * @attention
  *
  * [Module Design]
  * 1. 鏈ā鍧楁棬鍦ㄥ皝瑁呭皬绫矯yberGear鐢垫満鐨凜AN閫氫俊鍗忚锛屽皢澶嶆潅鐨勬姤鏂囨瀯閫犱笌瑙ｆ瀽
  *    缁嗚妭瀵逛笂灞傚簲鐢ㄩ€忔槑鍖栥€?
  * 2. 妯″潡瀹屽叏渚濊禆浜?`fdcan_bsp` 妯″潡鎻愪緵鐨勫簳灞侰AN鎶ユ枃鏀跺彂鍜屼腑鏂垎鍙戞湇鍔°€?
  *    閫氳繃鍚?`fdcan_bsp` 娉ㄥ唽涓€涓甫鎺╃爜鐨勯€氱敤鍥炶皟瑙勫垯锛岃兘澶熷紓姝ユ帴鏀跺拰澶勭悊鏉ヨ嚜
  *    鍚屼竴鐢垫満鐨勫绉嶅弽棣堟姤鏂囷紙濡傜被鍨?鐨勬寚浠ゅ簲绛斿拰绫诲瀷17鐨勫弬鏁版煡璇㈠洖澶嶏級銆?
  * 3. 椹卞姩鍐呴儴瀹炵幇浜嗗鐢垫満澶氱鎺у埗妯″紡鐨凙PI灏佽锛岄€氳繃鍙戦€侀€氫俊绫诲瀷涓?8鐨?
  *    鍙傛暟鍐欏叆鎸囦护鎴栭€氫俊绫诲瀷涓?鐨勮繍鎺ф寚浠ゆ潵鎺у埗鐢垫満銆?
  * 4. 鎻愪緵浜嗕富鍔ㄥ弬鏁拌姹傛満鍒讹紝鍏佽涓婂眰搴旂敤鎸夐渶杞鐢垫満鐘舵€併€?
  *
  * [Usage Flow]
  * 1. 鍦?`cybergear_motors_init()` 鍑芥暟涓紝鏍规嵁纭欢杩炴帴锛岄厤缃瘡涓數鏈?
  *    鐨凜AN ID鍜屾墍灞炵殑FDCAN閫氶亾銆?
  * 2. 鍦ㄧ郴缁熷垵濮嬪寲浠ｇ爜涓紝缁?`fdcan_bsp_init()` 涔嬪悗锛岃皟鐢?
  *    `cybergear_motors_init()` 鏉ュ畬鎴愮數鏈虹殑娉ㄥ唽銆?
  * 3. 鍦ㄩ渶瑕佹帶鍒剁數鏈虹殑鍦版柟锛岄鍏堥€氳繃 `cybergear_motor_get_instance(index)`
  *    鏉ヨ幏鍙栨寚瀹氱數鏈虹殑瀹炰緥鎸囬拡銆?
  * 4. 鍦ㄤ笅杈惧叿浣撴帶鍒舵寚浠ゅ墠锛屻€愬繀椤汇€戝厛璋冪敤 `cybergear_motor_set_mode()` 灏嗙數鏈?
  *    璁剧疆涓虹洰鏍囨ā寮忥紙杩愭帶妯″紡闄ゅ锛夈€?
  * 5. 鍦ㄥ垵濮嬪寲鎴栨ā寮忓垏鎹㈠悗锛岃皟鐢?`cybergear_motor_enable()` 鏉ヤ娇鑳界數鏈恒€傛牴鎹渶瑕侊紝
  *    鍙皟鐢?`set_pid_gains()` 鎴?`set_current_limit()` 杩涜鍙傛暟閰嶇疆銆?
  * 6. 鍦ㄤ富寰幆鎴栧畾鏃朵换鍔′腑锛岃皟鐢?`set_position()`, `set_speed()` 绛夊嚱鏁颁笅杈?
  *    鎺у埗鎸囦护锛屾垨璋冪敤 `request_parameter()` 璇锋眰鏁版嵁鏇存柊銆?
  * 7. 鐢垫満鐨勭姸鎬佷細鍦?`cybergear_message_handler()` 鍥炶皟鍑芥暟涓鑷姩寮傛鏇存柊锛?
  *    涓婂眰搴旂敤鍙殢鏃惰闂數鏈哄疄渚嬩腑鐨?`measure` 缁撴瀯浣撴潵鑾峰彇鏈€鏂版暟鎹€?
  * 
  * [FreeRTOS 绾跨▼瀹夊叏鏀寔]
  * 1. 鍦?FreeRTOS 绛夊浠诲姟鐜涓嬶紝涓嶅悓鐨勪换鍔″彲鑳藉悓鏃惰皟鐢ㄦ湰椹卞姩鐨凙PI鍑芥暟鏉ュ彂閫丆AN鎸囦护锛?
  *    杩欎細閫犳垚瀵笷DCAN纭欢璧勬簮鐨勫苟鍙戣闂紝鍙兘瀵艰嚧鍙戦€侀槦鍒楁崯鍧忔垨鎬荤嚎閿欒銆?
  * 2. 鏈┍鍔ㄩ€氳繃鍐呴儴瀹炵幇鐨勪竴涓簰鏂ラ攣 (Mutex) 鏈哄埗锛屽鎵€鏈塁AN鎶ユ枃鍙戦€佹搷浣滆繘琛屼繚鎶わ紝纭繚
  *    浠讳綍鏃跺埢鍙湁涓€涓换鍔″彲浠ヨ闂瓹AN鍙戦€佺‖浠讹紝浠庤€屽疄鐜扮嚎绋嬪畨鍏ㄣ€?
  * 3. 鎵€鏈変笌FreeRTOS鐩稿叧鐨勪唬鐮佸潎閫氳繃鏉′欢缂栬瘧瀹?`USE_FREERTOS` 杩涜闅旂銆?
  *    - 褰撳畾涔変簡 `USE_FREERTOS` 鏃讹紝绾跨▼瀹夊叏浠ｇ爜灏嗚缂栬瘧锛岄┍鍔ㄩ€傜敤浜嶧reeRTOS鐜銆?
  *    - 褰撴湭瀹氫箟 `USE_FREERTOS` 鏃讹紝鐩稿叧浠ｇ爜灏嗚蹇界暐锛岄┍鍔ㄥ彲鏃犵紳鐢ㄤ簬瑁告満鐜銆?
  * 4. 瀹炵幇缁嗚妭锛?
  *    a. `cybergear_lock_init()`: 鍦?`cybergear_motors_init()` 涓嚜鍔ㄨ皟鐢紝鐢ㄤ簬鍒涘缓浜掓枼閿併€?
  *    b. `cybergear_lock_tx()`: 鍦ㄥ彂閫丆AN鎶ユ枃鍓嶈幏鍙栭攣銆?
  *    c. `cybergear_unlock_tx()`: 鍦ㄥ彂閫丆AN鎶ユ枃鍚庨噴鏀鹃攣銆?
  *
  ******************************************************************************
  */

#include "cybergear_motor.h"
#include "fdcan_bsp.h" 
#include <string.h>

// 闃叉閲嶅畾涔?M_PI
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef USE_FREERTOS 
    #include "FreeRTOS.h"
    #include "semphr.h"
    
    // RTOS鐜涓嬶細瀹氫箟涓€涓湡瀹炵殑浜掓枼閿?
    static SemaphoreHandle_t g_can_tx_mutex = NULL;
    
    // 瀹氫箟閿佹垚鍔熺殑杩斿洖鍊?(鍩轰簬FreeRTOS)
    #define CYBERGEAR_LOCK_SUCCESS  (pdTRUE)

#else // 瑁告満 (Bare-metal) 鐜涓?
    
    // 瀹氫箟閿佹垚鍔熺殑杩斿洖鍊?(瑁告満)
    #define CYBERGEAR_LOCK_SUCCESS  (1) 
    // 瑁告満鐜涓嬩笉闇€瑕佸叏灞€鍙橀噺

#endif



/* ------------------------- Static Variables ------------------------- */



// 鍏ㄥ眬鐢垫満瀹炰緥鏁扮粍
static CyberGear_Motor_Instance cybergear_motors[CYBERGEAR_MOTOR_COUNT];



/* ------------------------- Private Function Prototypes ------------------------- */



// 娴偣鏁板埌鏃犵鍙锋暣鏁扮殑杞崲杈呭姪鍑芥暟
static uint16_t float_to_uint(float x, float x_min, float x_max, int bits);

// 鏃犵鍙锋暣鏁板埌娴偣鏁扮殑杞崲杈呭姪鍑芥暟
static float uint16_to_float(uint16_t x, float x_min, float x_max, int bits);

// 鍗曚釜鐢垫満閰嶇疆涓庢敞鍐岀殑绉佹湁杈呭姪鍑芥暟
static void cybergear_motor_configure(uint8_t index, const uint8_t id, FDCAN_HandleTypeDef* hfdcan);

// 鍙戦€丆AN鎶ユ枃鐨勫簳灞傝緟鍔╁嚱鏁?
static void cybergear_can_send_raw(CyberGear_Motor_Instance* motor, uint8_t comm_type, uint16_t data2, uint8_t target_id, uint8_t tx_data[8]);

// 浣胯兘鐢垫満
static void motor_enable(CyberGear_Motor_Instance* motor);

// 鍚戠數鏈哄啓鍏ヤ竴涓诞鐐瑰瀷(float)鍙傛暟
static void cybergear_write_float_parameter(CyberGear_Motor_Instance* motor, uint16_t index, float value);

// 鍚戠數鏈哄啓鍏ヤ竴涓崟瀛楄妭(uint8_t)鍙傛暟
static void cybergear_write_u8_parameter(CyberGear_Motor_Instance* motor, uint16_t index, uint8_t value);

// 閫傞厤FreeRTOS鐨勭敤浜庡苟鍙戞帶鍒剁殑绉佹湁鍑芥暟
static void cybergear_lock_init(void);
static uint8_t cybergear_lock_tx(void); // 杩斿洖鍊肩敤浜庡垽鏂槸鍚︽垚鍔?
static void cybergear_unlock_tx(void);

/* ------------------------- Public Function Implementations ------------------------- */



/**
 * @brief   灏嗕竴涓粰瀹氱殑瑙掑害鍊?搴?杞崲涓哄姬搴﹀€?寮у害)
 * @note    鐢变簬鐢垫満浣嶇疆妯″紡浠呮敮鎸?720~720搴﹁寖鍥村唴鐨勪綅缃帶鍒讹紝瓒呭嚭鑼冨洿鐨勫€煎皢琚鍓?
 * @param   degree: 瑙掑害鍊?-720~720)
 * @retval  涓€涓姬搴﹀€?float)
 */
float cybergear_motor_degree2rad(float degree)
{
    if (degree < -720.0f) degree = -720.0f;
    if (degree > 720.0f) degree = 720.0f;
    return degree * (M_PI / 180.0f);
}

/**
 * @brief   灏嗕竴涓粰瀹氱殑杞€熷€?rpm)杞崲涓鸿閫熷害(rad/s)
 * @note    鐢变簬鐢垫満閫熷害妯″紡浠呮敮鎸?30~30 rad/s鑼冨洿鍐呯殑閫熷害鎺у埗锛岃秴鍑鸿寖鍥寸殑鍊煎皢琚鍓?
 * @param   rpm: 杞€?-296~296卤10%)
 * @retval  涓€涓閫熷害鍊?float)
 */
float cybergear_motor_rpm2rad(int16_t rpm)
{
    float rad_per_sec = (float)rpm * (2.0f * M_PI / 60.0f);
    if (rad_per_sec < -30.0f) rad_per_sec = -30.0f;
    if (rad_per_sec > 30.0f) rad_per_sec = 30.0f;
    return rad_per_sec;
}

/**
 * @brief  鍒濆鍖栨墍鏈塁yberGear鐢垫満瀹炰緥
 */
void cybergear_motors_init(void)
{
    // 璋冪敤瀹為檯鐨勯攣鍒濆鍖栧嚱鏁?
    cybergear_lock_init();

    // --- 鍦ㄨ繖閲岄泦涓厤缃墍鏈夌數鏈?---
    // 鍙傛暟: 鏁扮粍绱㈠紩, 鐢垫満CAN_ID, 鎵€灞濬DCAN鍙ユ焺
    cybergear_motor_configure(0, 1, &hfdcan2); 
    // 濡傛灉鏈夋洿澶氱數鏈猴紝璇风户缁坊鍔?..
    // cybergear_motor_configure(1, 2, &hfdcan1);
}

/**
 * @brief  鑾峰彇鎸囧畾绱㈠紩鐨勭數鏈哄疄渚嬫寚閽?
 * @param  motor_index: 鐢垫満绱㈠紩 (0 to CYBERGEAR_MOTOR_COUNT-1)
 * @retval 鎸囧悜鐢垫満瀹炰緥鐨勬寚閽堬紝濡傛灉绱㈠紩鏃犳晥鍒欒繑鍥濶ULL
 */
CyberGear_Motor_Instance* cybergear_motor_get_instance(uint8_t motor_index)
{
    if (motor_index >= CYBERGEAR_MOTOR_COUNT)
    {
        return NULL;
    }
    return &cybergear_motors[motor_index];
}

/**
 * @brief  鍏紑鐨勭數鏈轰娇鑳藉嚱鏁?
 * @param  motor: 鎸囧悜瑕佷娇鑳界殑鐢垫満瀹炰緥
 */
void cybergear_motor_enable(CyberGear_Motor_Instance* motor)
{
    motor_enable(motor); // 璋冪敤 static motor_enable 鍑芥暟
}

/**
 * @brief  鍋滄鐢垫満杩愯 (鍙戦€侀€氫俊绫诲瀷4)
 * @param  motor: 鎸囧悜瑕佸仠姝㈢殑鐢垫満瀹炰緥
 */
void cybergear_motor_stop(CyberGear_Motor_Instance* motor)
{
    uint8_t tx_data[8] = {0};
    // 閫氫俊绫诲瀷4锛屼富鏈篒D鍦╠ata2瀛楁锛岀洰鏍囨槸鐢垫満ID
    cybergear_can_send_raw(motor, 4, MASTER_CAN_ID, motor->can_id, tx_data);
}

/**
 * @brief   璁剧疆鐢垫満鐨勬帶鍒舵ā寮?
 * @details 閬靛惊璇存槑涔﹀畨鍏ㄨ鑼冿細鍏堝彂閫佸仠姝㈡寚浠わ紝鍐嶅垏鎹㈡ā寮忋€?
 *          娉ㄦ剰锛氭鍑芥暟鎵ц鍚庯紝鐢垫満灏嗗浜庡仠姝㈢姸鎬侊紝闇€瑕侀噸鏂拌皟鐢?enable 鎵嶈兘杩愯銆?
 * @param  motor: 鎸囧悜鐢垫満瀹炰緥
 * @param  mode:  瑕佽缃殑鐩爣鎺у埗妯″紡
 */
void cybergear_motor_set_mode(CyberGear_Motor_Instance* motor, CyberGear_Control_Mode_e mode)
{
    // 濡傛灉鐩爣妯″紡涓庡綋鍓嶆ā寮忕浉鍚岋紝鍒欐棤闇€鎿嶄綔
    if (motor->control_mode == mode)
    {
        return;
    }

    // 1. 鍏堝彂閫佸仠姝㈡寚浠わ紝纭繚瀹夊叏
    cybergear_motor_stop(motor);
    // (鍙互鏍规嵁闇€瑕佸姞鍏ヤ竴涓皬鐨勫欢鏃? HAL_Delay(1), 浠ョ‘淇濈數鏈烘湁瓒冲鏃堕棿鍝嶅簲)
    // HAL_Delay(1);

    // 2. 鏍规嵁鐩爣妯″紡锛屽彂閫佸弬鏁板啓鍏ユ寚浠?
    cybergear_write_u8_parameter(motor, 0x7005, (uint8_t)mode);

    // 3. 鏇存柊涓绘帶渚х殑妯″紡缂撳瓨
    motor->control_mode = mode;
}

/**
 * @brief   璁剧疆褰撳墠鐢垫満浣嶇疆涓烘満姊伴浂浣?(鎺夌數涓㈠け)
 * @details 鍙戦€侀€氫俊绫诲瀷涓?鐨勬寚浠?
 * @param  motor: 鎸囧悜鐢垫満瀹炰緥
 */
void cybergear_motor_set_zero_position(CyberGear_Motor_Instance* motor)
{
    uint8_t tx_data[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    // 閫氫俊绫诲瀷6锛屼富鏈篒D鍦╠ata2瀛楁锛岀洰鏍囨槸鐢垫満ID
    cybergear_can_send_raw(motor, 6, MASTER_CAN_ID, motor->can_id, tx_data);
}

/**
 * @brief   璁剧疆鐢垫満鍒版寚瀹氫綅缃?(浣嶇疆妯″紡)
 * @details 鍐呴儴浼氬彂閫佹寚浠わ細璁剧疆鐩爣浣嶇疆
 * @note    璋冪敤姝ゅ嚱鏁板墠锛屽繀椤诲厛閫氳繃 cybergear_motor_set_mode() 灏嗙數鏈鸿涓轰綅缃ā寮忋€?
 * @param   motor:       鎸囧悜鐢垫満瀹炰緥
 * @param   position:    鐩爣浣嶇疆 (rad)
 * @param   speed_limit: 鍒拌揪鐩爣浣嶇疆杩囩▼涓殑鏈€澶ч€熷害闄愬埗 (rad/s)
 */
void cybergear_motor_set_position(CyberGear_Motor_Instance* motor, float position, float speed_limit)
{
    // 1. 瀹夊叏妫€鏌ワ細纭繚褰撳墠澶勪簬姝ｇ‘鐨勬ā寮?
    if (motor->control_mode != MOTOR_CONTROL_MODE_POSITION)
    {
        return;
    }
    
    // 2. 璁剧疆浣嶇疆妯″紡涓嬬殑閫熷害闄愬埗
    cybergear_write_float_parameter(motor, 0x7017, speed_limit);
    
    // 3. 璁剧疆鐩爣浣嶇疆
    cybergear_write_float_parameter(motor, 0x7016, position);
}

/**
 * @brief  璁剧疆鐢垫満浠ユ寚瀹氶€熷害杩愯 (閫熷害妯″紡)
 * @details 鍐呴儴浼氬彂閫佹寚浠わ細璁剧疆鐩爣閫熷害
 * @note   璋冪敤姝ゅ嚱鏁板墠锛屽繀椤诲厛閫氳繃 cybergear_motor_set_mode() 灏嗙數鏈鸿涓洪€熷害妯″紡銆?
 * @param  motor: 鎸囧悜鐢垫満瀹炰緥
 * @param  speed: 鐩爣閫熷害 (rad/s), 鑼冨洿 -30 ~ 30
 */
void cybergear_motor_set_speed(CyberGear_Motor_Instance* motor, float speed)
{
    // 1. 瀹夊叏妫€鏌ワ細纭繚褰撳墠澶勪簬姝ｇ‘鐨勬ā寮?
    if (motor->control_mode != MOTOR_CONTROL_MODE_SPEED)
    {
        return;
    }
    
    // 2. 璁剧疆鐩爣閫熷害
    cybergear_write_float_parameter(motor, 0x700A, speed);
}

/**
 * @brief  璁剧疆鐢垫満杈撳嚭鎸囧畾鐨勫姏鐭?(鐢垫祦妯″紡)
 * @details 鍐呴儴浼氬彂閫佹寚浠わ細璁剧疆鐩爣 Iq 鐢垫祦
 * @note   璋冪敤姝ゅ嚱鏁板墠锛屽繀椤诲厛閫氳繃 cybergear_motor_set_mode() 灏嗙數鏈鸿涓哄姏鐭╂ā寮忋€?
 * @param  motor:  鎸囧悜鐢垫満瀹炰緥
 * @param  torque: 鐩爣鍔涚煩 (N.m)锛岃鍊间細閫氳繃杞煩甯告暟杩戜技杞崲涓篒q鐢垫祦
 */
void cybergear_motor_set_torque(CyberGear_Motor_Instance* motor, float torque)
{
    // 鐢垫満鐨勮浆鐭╁父鏁扮害涓?0.87 N.m/Arms
    // 杩欐槸涓€涓繎浼煎€硷紝瀹為檯Iq鍒板姏鐭╃殑鍏崇郴鍙兘鏇村鏉傦紝浣嗕綔涓烘帶鍒惰冻澶熶簡
    const float TORQUE_CONSTANT = 0.87f;
    float iq_ref = torque / TORQUE_CONSTANT;

    // 1. 瀹夊叏妫€鏌ワ細纭繚褰撳墠澶勪簬姝ｇ‘鐨勬ā寮?
    if (motor->control_mode != MOTOR_CONTROL_MODE_TORQUE)
    {
        return;
    }

    // 2. 璁剧疆鐩爣Iq鐢垫祦
    cybergear_write_float_parameter(motor, 0x7006, iq_ref);
}

/**
 * @brief  浣跨敤杩愭帶妯″紡鎺у埗鐢垫満
 * @details 鍙戦€侀€氫俊绫诲瀷涓?鐨勫崟甯ф姤鏂囷紝鍚屾椂璁惧畾鎵€鏈?涓帶鍒跺弬鏁般€?
 * @note   杩愭帶妯″紡鏄數鏈虹殑榛樿妯″紡锛屼笖涓虹洿鎺ュ姩浣滄寚浠わ紝鏃犻渶棰勮妯″紡銆?
 * @param  motor:     鎸囧悜鐢垫満瀹炰緥
 * @param  position:  鐩爣浣嶇疆 (rad)
 * @param  speed:     鐩爣閫熷害 (rad/s)
 * @param  kp:        浣嶇疆鐜瘮渚嬪鐩?Kp (0~500)
 * @param  kd:        閫熷害鐜樆灏肩郴鏁?Kd (0~5)
 * @param  torque_ff: 鍓嶉鍔涚煩 (N.m), 鑼冨洿 -12 ~ 12
 */
void cybergear_motor_set_motion_control(CyberGear_Motor_Instance* motor, float position, float speed, float kp, float kd, float torque_ff)
{
    // 鏇存柊涓绘帶渚х殑妯″紡缂撳瓨锛屼互渚垮叾浠栧嚱鏁拌兘姝ｇ‘鍒ゆ柇鐘舵€?
    motor->control_mode = MOTOR_CONTROL_MODE_MOTION;

    // 1. 灏嗗墠棣堝姏鐭╂墦鍖呭埌鎵╁睍ID鐨?data2 瀛楁涓?
    uint16_t torque_uint = float_to_uint(torque_ff, T_MIN, T_MAX, 16);

    // 2. 灏嗗叾浠栧洓涓弬鏁版墦鍖呭埌8瀛楄妭鐨勬暟鎹尯涓?
    uint8_t tx_data[8];
    
    uint16_t pos_uint = float_to_uint(position, P_MIN, P_MAX, 16);
    uint16_t spd_uint = float_to_uint(speed, V_MIN, V_MAX, 16);
    uint16_t kp_uint = float_to_uint(kp, KP_MIN, KP_MAX, 16);
    uint16_t kd_uint = float_to_uint(kd, KD_MIN, KD_MAX, 16);
    
    tx_data[0] = (uint8_t)(pos_uint >> 8);
    tx_data[1] = (uint8_t)(pos_uint);
    tx_data[2] = (uint8_t)(spd_uint >> 8);
    tx_data[3] = (uint8_t)(spd_uint);
    tx_data[4] = (uint8_t)(kp_uint >> 8);
    tx_data[5] = (uint8_t)(kp_uint);
    tx_data[6] = (uint8_t)(kd_uint >> 8);
    tx_data[7] = (uint8_t)(kd_uint);


    // 3. 璋冪敤搴曞眰鍙戦€佸嚱鏁?
    // 閫氫俊绫诲瀷1锛屾墦鍖呭悗鐨勫姏鐭╁湪data2瀛楁锛岀洰鏍囨槸鐢垫満ID
    cybergear_can_send_raw(motor, 1, torque_uint, motor->can_id, tx_data);
}

/**
 * @brief  璁剧疆鐢垫満鍐呴儴鐨勪綅缃幆鍜岄€熷害鐜疨ID澧炵泭
 * @details 灏嗚皟璇曞ソ鐨凱ID鍙傛暟鍐欏叆鐢垫満RAM (鎺夌數涓㈠け)
 * @param  motor:  鎸囧悜鐢垫満瀹炰緥
 * @param  loc_kp: 浣嶇疆鐜?Kp (鏉ヨ嚜鍙傛暟琛?0x2016)
 * @param  spd_kp: 閫熷害鐜?Kp (鏉ヨ嚜鍙傛暟琛?0x2014)
 * @param  spd_ki: 閫熷害鐜?Ki (鏉ヨ嚜鍙傛暟琛?0x2015)
 */
void cybergear_motor_set_pid_gains(CyberGear_Motor_Instance* motor, float loc_kp, float spd_kp, float spd_ki)
{
    // 鍐欏叆浣嶇疆鐜?Kp (鍦板潃 0x2016)
    cybergear_write_float_parameter(motor, 0x2016, loc_kp);

    // 鍐欏叆閫熷害鐜?Kp (鍦板潃 0x2014)
    cybergear_write_float_parameter(motor, 0x2014, spd_kp);

    // 鍐欏叆閫熷害鐜?Ki (鍦板潃 0x2015)
    cybergear_write_float_parameter(motor, 0x2015, spd_ki);
}

/**
 * @brief   璁剧疆鐢垫満鍦ㄩ€熷害/浣嶇疆妯″紡涓嬬殑鏈€澶х數娴侀檺鍒?
 * @details 灏嗗€煎啓鍏ュ弬鏁板湴鍧€ 0x7018 (limit_cur)銆傝鍊兼帀鐢典涪澶便€?
 * @param   motor:         鎸囧悜鐢垫満瀹炰緥
 * @param   current_limit: 鏈€澶х數娴佸€?(A)锛岃寖鍥?0~23A
 */
void cybergear_motor_set_current_limit(CyberGear_Motor_Instance* motor, float current_limit)
{
    // 璁剧疆涓€涓畨鍏ㄩ挸浣?
    if (current_limit < 0) current_limit = 0;
    if (current_limit > 23.0f) current_limit = 23.0f;

    cybergear_write_float_parameter(motor, 0x7018, current_limit);
}

/**
 * @brief  涓诲姩璇锋眰璇诲彇鐢垫満鐨勫崟涓弬鏁?
 * @details 鍙戦€侀€氫俊绫诲瀷涓?7鐨勬寚浠ゃ€傜數鏈哄皢浼氬洖澶嶄竴甯у悓鏍风被鍨嬩负17鐨勬姤鏂囷紝
 *          鍏朵腑鍖呭惈浜嗘墍璇锋眰鍙傛暟鐨勫€笺€傞渶瑕佸湪FDCAN鐨勬帴鏀剁澶勭悊璇ュ洖澶嶃€?
 * @param  motor:           鎸囧悜鐢垫満瀹炰緥
 * @param  parameter_index: 鎯宠鍙栫殑鍙傛暟鐨勫湴鍧€锛屾潵鑷鏄庝功4.1.11鑺傘€?
 *          宸插皝瑁呬簡閮ㄥ垎甯哥敤鍙傛暟涓烘灇涓?CyberGear_Param_Index_e
 *          - PARAM_MECH_POS = 0x7019: 璐熻浇绔鍦堟満姊拌搴?(rad)
 *          - PARAM_IQ_FILTER = 0x701A: iq 婊ゆ尝鍊?(A)
 *          - PARAM_MECH_VEL = 0x701B: 璐熻浇绔浆閫?(rad/s)
 *          - PARAM_VBUS = 0x701C: 姣嶇嚎鐢靛帇 (V)
 *          - PARAM_ROTATION = 0x701D: 鍦堟暟, int16_t
 */
void cybergear_motor_request_parameter(CyberGear_Motor_Instance* motor, CyberGear_Param_Index_e param_index)
{
    uint8_t tx_data[8] = {0};
    uint16_t index_val = param_index; // 鏋氫妇鍊煎彲浠ョ洿鎺ヨ祴缁檜int16_t

    memcpy(&tx_data[0], &index_val, sizeof(uint16_t));
    
    cybergear_can_send_raw(motor, 17, MASTER_CAN_ID, motor->can_id, tx_data);
}



/* ------------------------- Private Function Implementations ------------------------- */



/**
 * @brief  CyberGear鐢垫満鎶ユ枃鐨勭粺涓€澶勭悊鍑芥暟 (鍥炶皟鍑芥暟)
 * @details 璇ュ嚱鏁颁細琚?`fdcan_bsp` 妯″潡鍦ㄦ敹鍒板尮閰嶇殑CAN ID鏃惰皟鐢ㄣ€?
 * @param  instance:  鎸囧悜瑙﹀彂鍥炶皟鐨勭數鏈哄疄渚嬬殑void鎸囬拡
 * @param  rx_header: 鍖呭惈鎵╁睍ID绛変俊鎭殑FDCAN鎶ユ枃澶?
 * @param  rx_data:   鎺ユ敹鍒扮殑8瀛楄妭CAN鏁版嵁
 */
void cybergear_message_handler(void* instance, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[8])
{
    if (instance == NULL || rx_header == NULL) return;
    
    // 灏嗘帴鏀跺埌鐨勫疄渚嬭浆鎹㈠洖鐢垫満瀹炰緥
    CyberGear_Motor_Instance* motor = (CyberGear_Motor_Instance*)instance;

    // 浠庢墿灞旾D涓В鏋愬嚭閫氫俊绫诲瀷
    uint8_t comm_type = (rx_header->Identifier >> 24) & 0x1F;

    // 鏍规嵁閫氫俊绫诲瀷锛屽垎鍙戝埌涓嶅悓鐨勫鐞嗛€昏緫
    switch(comm_type) {
        case 2: // 绫诲瀷2: 鐢垫満閫氱敤鍙嶉鎶ユ枃
        {
            // 瑙ｆ瀽妯″紡浣嶅拰鏁呴殰浣?
            motor->measure.mode = (CyberGear_Motor_Mode_e)((rx_header->Identifier >> 22) & 0x03);
            motor->measure.fault = (uint8_t)((rx_header->Identifier >> 16) & 0x3F);

            // 瑙ｆ瀽鏁版嵁浣?
            uint16_t raw_angle = (rx_data[0] << 8) | rx_data[1];
            uint16_t raw_speed = (rx_data[2] << 8) | rx_data[3];
            uint16_t raw_torque = (rx_data[4] << 8) | rx_data[5];
            uint16_t raw_temp = (rx_data[6] << 8) | rx_data[7];

            // 澶勭悊鏁版嵁锛岃浆鍖栦负鐩磋鐨勯噺绾?
            motor->measure.angle = uint16_to_float(raw_angle, P_MIN, P_MAX, 16);
            motor->measure.speed = uint16_to_float(raw_speed, V_MIN, V_MAX, 16);
            motor->measure.torque = uint16_to_float(raw_torque, T_MIN, T_MAX, 16);
            motor->measure.temperature = (float)raw_temp * 0.1f;
            break;
        }
        case 17: // 绫诲瀷17: 鍗曚釜鍙傛暟璇诲彇鐨勫洖澶嶆姤鏂?
        {
            // 瑙ｆ瀽鍥炲鐨勫弬鏁板湴鍧€鍜屽€?
            uint16_t param_index;
            // 浣跨敤灏忕瑙ｆ瀽鍣?
            memcpy(&param_index, &rx_data[0], sizeof(uint16_t));

            // 鏍规嵁鍦板潃锛屽皢鍊煎瓨鍏ュ搴旂殑缁撴瀯浣撴垚鍛?
            // 娉ㄦ剰杩欓噷浠呮牴鎹灇涓綜yberGear_Param_Index_e涓皝瑁呯殑鍑犱釜鍙傛暟鍋氳В鏋?
            // 濡傛灉闇€瑕佸叾浠栧弬鏁拌В鏋愶紝鍙嚜琛屾嫇灞?
            switch(param_index) {
                case PARAM_MECH_POS: // 璐熻浇绔鍦堟満姊拌搴?(rad), float
                {
                    float val;
                    memcpy(&val, &rx_data[4], sizeof(float));
                    motor->measure.angle = val; // 鏇存柊涓庣被鍨?鎶ユ枃鐩稿悓鐨勫瓧娈?
                    break;
                }
                case PARAM_MECH_VEL: // 璐熻浇绔浆閫?(rad/s), float
                {
                    float val;
                    memcpy(&val, &rx_data[4], sizeof(float));
                    motor->measure.speed = val; // 鏇存柊涓庣被鍨?鎶ユ枃鐩稿悓鐨勫瓧娈?
                    break;
                }
                case PARAM_VBUS: // 姣嶇嚎鐢靛帇 (V), float
                {
                    float val;
                    memcpy(&val, &rx_data[4], sizeof(float));
                    motor->measure.vbus = val; // 鏇存柊绫诲瀷17鎶ユ枃浼犲洖鐨勫瓧娈?
                    break;
                }
                case PARAM_ROTATION: // 鍦堟暟, int16_t
                {
                    int16_t val;
                    // 娉ㄦ剰鍦堟暟鏄痠nt16锛屽崰2瀛楄妭
                    memcpy(&val, &rx_data[4], sizeof(int16_t));
                    motor->measure.rotation = val; // 鏇存柊绫诲瀷17鎶ユ枃浼犲洖鐨勫瓧娈?
                    break;
                }
                // 瀵逛簬鍏朵粬鍙傛暟锛屽彲浠ユ殏鏃朵笉澶勭悊鎴栨寜闇€娣诲姞
                default: break; 
            }

            break;
        }
        // 瀵逛簬鍏朵粬閫氫俊绫诲瀷锛屾殏鏃朵笉鍏冲績
        // 鍚庣画鏈夐渶瑕佹椂鍙坊鍔犵浉鍏冲鐞嗛€昏緫
        default: break;
    }
}

/**
 * @brief  灏嗕竴涓诞鐐规暟杞崲涓烘寚瀹氫綅鏁扮殑鏃犵鍙锋暣鏁?
 * @param  x:      杈撳叆鐨勬诞鐐规暟
 * @param  x_min:  娴偣鏁拌寖鍥寸殑涓嬮檺
 * @param  x_max:  娴偣鏁拌寖鍥寸殑涓婇檺
 * @param  bits:   鐩爣鏁存暟鐨勪綅鏁?(閫氬父鏄?6)
 * @retval 杞崲鍚庣殑鏃犵鍙锋暣鏁?
 */
static uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    // 闄愬埗杈撳叆鍊煎湪鑼冨洿鍐?
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    // 鎵ц杞崲
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief  灏嗕竴涓棤绗﹀彿鏁存暟鏍规嵁鑼冨洿杞崲涓烘诞鐐规暟
 * @param  x:      杈撳叆鐨?6浣嶆棤绗﹀彿鏁存暟
 * @param  x_min:  娴偣鏁拌寖鍥寸殑涓嬮檺
 * @param  x_max:  娴偣鏁拌寖鍥寸殑涓婇檺
 * @param  bits:   杈撳叆鏁存暟鐨勪綅鏁?(閫氬父鏄?6)
 * @retval 杞崲鍚庣殑娴偣鏁?
 */
static float uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
{
    uint32_t span = (1 << bits) - 1;
    float offset = x_max - x_min;
    return ((float)x * offset) / ((float)span) + x_min;
}

/**
 * @brief  閰嶇疆鍗曚釜鐢垫満瀹炰緥骞舵敞鍐屽埌FDCAN鍒嗗彂鍣?
 * @param  index:  鐢垫満鍦?cybergear_motors 鏁扮粍涓殑绱㈠紩
 * @param  id:     鐢垫満鐨凜AN ID (0-127)
 * @param  hfdcan: 璇ョ數鏈烘墍灞炵殑FDCAN鍙ユ焺 (e.g. &hfdcan1)
 */
static void cybergear_motor_configure(uint8_t index, const uint8_t id, FDCAN_HandleTypeDef* hfdcan)
{
    if (index >= CYBERGEAR_MOTOR_COUNT) return;

    CyberGear_Motor_Instance* motor = &cybergear_motors[index];

    // 1. 閰嶇疆鍩烘湰淇℃伅
    motor->can_id = id;
    motor->hfdcan = hfdcan;
    motor->control_mode = MOTOR_CONTROL_MODE_UNSET; // 鍒濆鍖栨帶鍒舵ā寮忎负鏈瀹?
    
    // 2. 鍑嗗娉ㄥ唽淇℃伅
    FDCAN_Dispatch_t dispatch_item;
    
    // 灏忕背鐢垫満浣跨敤29浣嶆墿灞曞抚
    dispatch_item.id_type = FDCAN_EXTENDED_ID; 
    
    // 銆愪娇鐢ㄦ帺鐮佹満鍒惰繘琛屾敞鍐屻€?
    // 1. 鎴戜滑瑕佸尮閰嶇殑ID妯℃澘锛氬彧璁剧疆鎴戜滑鍏冲績鐨勩€佸浐瀹氱殑閮ㄥ垎
    // 鏍煎紡: comm_type (5bit) | source_id (16bit) | target_id (8bit)
    // 涓轰簡鍚屾椂瀹炵幇搴旂瓟寮忓弽棣堝抚鍜岃疆璇㈠紡鍙嶉甯э紝鍙叧蹇冩姤鏂囩殑鍙戦€佹柟鍜屾帴鏀舵柟
    dispatch_item.id = (uint32_t)(id << 8) |      // 婧愬湴鍧€蹇呴』鏄數鏈鸿嚜宸辩殑ID
                       (uint32_t)(MASTER_CAN_ID); // 鐩爣鍦板潃蹇呴』鏄富鏈篒D
    
    dispatch_item.instance_ptr = motor;                 // 浼犻€掔數鏈哄疄渚嬫寚閽?
    dispatch_item.handler = cybergear_message_handler;  // 娉ㄥ唽鍥炶皟鍑芥暟

    // 3. 璋冪敤bsp灞傜殑娉ㄥ唽鍑芥暟
    fdcan_bsp_register(&dispatch_item, hfdcan);
}

/**
 * @brief  CyberGear鐢垫満CAN鎸囦护鐨勫簳灞傚彂閫佸嚱鏁?
 * @param  motor:       鎸囧悜鐢垫満瀹炰緥锛岀敤浜庤幏鍙朏DCAN鍙ユ焺
 * @param  comm_type:   閫氫俊绫诲瀷 (缂栫爜浜嶦xtID bit28-24)
 * @param  data2:       鏁版嵁鍖? (缂栫爜浜嶦xtID bit23-8, 閫氬父鏄富鏈篒D鎴栧弬鏁?
 * @param  target_id:   鐩爣鐢垫満ID (缂栫爜浜嶦xtID bit7-0)
 * @param  tx_data:     鎸囧悜8瀛楄妭鏁版嵁璐熻浇鐨勬寚閽?
 */
static void cybergear_can_send_raw(CyberGear_Motor_Instance* motor, 
                                   uint8_t comm_type, 
                                   uint16_t data2, 
                                   uint8_t target_id, 
                                   uint8_t tx_data[8])
{
    if (motor == NULL || motor->hfdcan == NULL) return;

    // ---==== 淇濇姢鍖哄紑濮?====---
    // 璋冪敤瀹為檯鐨勯攣鍑芥暟
    if (cybergear_lock_tx() != CYBERGEAR_LOCK_SUCCESS)
    {
        // 鑾峰彇閿佽秴鏃?(浠呭湪RTOS涓嬪彲鑳藉彂鐢?
        // 娉ㄦ剰姝ゅ瓒呮椂鏃堕棿涓?10 ms锛屽叿浣撳€煎彲鍦╜cybergear_lock_tx`鍑芥暟涓慨鏀?
        // TODO: 澶勭悊鍙戦€佽秴鏃堕敊璇?
        return; 
    }

    FDCAN_TxHeaderTypeDef tx_header;
    
    // 1. 鏋勯€?9浣嶆墿灞旾D
    // 鏍煎紡:  comm_type (5bit) | data2 (16bit) | target_id (8bit) 
    tx_header.Identifier = (uint32_t)(comm_type << 24) | 
                           (uint32_t)(data2 << 8)    | 
                           (uint32_t)(target_id);
    
    // 2. 閰嶇疆鎶ユ枃澶?
    tx_header.IdType = FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN; // 浣跨敤浼犵粺CAN 2.0鏍煎紡
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    // 3. 鍙戦€佹姤鏂?
    if (HAL_FDCAN_AddMessageToTxFifoQ(motor->hfdcan, &tx_header, tx_data) != HAL_OK)
    {
        // TODO: 澶勭悊鍙戦€侀敊璇?(渚嬪璁板綍鏃ュ織)
        Error_Handler();
    }

    // ---==== 淇濇姢鍖虹粨鏉?====---
    // 璋冪敤瀹為檯鐨勯噴鏀鹃攣鍑芥暟
    cybergear_unlock_tx();
}

/**
 * @brief  浣胯兘鐢垫満
 * @param  motor: 鎸囧悜瑕佷娇鑳界殑鐢垫満瀹炰緥
 */
static void motor_enable(CyberGear_Motor_Instance* motor)
{
    uint8_t tx_data[8] = {0}; // 鏁版嵁鍖轰负绌?
    // 閫氫俊绫诲瀷3锛屼富鏈篒D鍦╠ata2瀛楁锛岀洰鏍囨槸鐢垫満ID
    cybergear_can_send_raw(motor, 3, MASTER_CAN_ID, motor->can_id, tx_data);
}

/**
 * @brief  鍚戠數鏈哄啓鍏ヤ竴涓诞鐐瑰瀷鍙傛暟
 * @param  motor: 鎸囧悜鐢垫満瀹炰緥
 * @param  index: 瑕佸啓鍏ョ殑鍙傛暟鍦板潃 (e.g., 0x7006 for iq_ref)
 * @param  value: 瑕佸啓鍏ョ殑娴偣鏁板€?
 */
static void cybergear_write_float_parameter(CyberGear_Motor_Instance* motor, uint16_t index, float value)
{
    uint8_t tx_data[8] = {0};
    
    // 鏍规嵁鍗忚锛屽弬鏁板湴鍧€鏀惧湪 Byte0-1
    memcpy(&tx_data[0], &index, sizeof(uint16_t));
    
    // 娴偣鏁板€兼斁鍦?Byte4-7
    memcpy(&tx_data[4], &value, sizeof(float));

    // 閫氫俊绫诲瀷18锛屼富鏈篒D鍦╠ata2瀛楁锛岀洰鏍囨槸鐢垫満ID
    cybergear_can_send_raw(motor, 18, MASTER_CAN_ID, motor->can_id, tx_data);
}

/**
 * @brief  鍚戠數鏈哄啓鍏ヤ竴涓崟瀛楄妭(uint8_t)鍙傛暟
 * @param  motor: 鎸囧悜鐢垫満瀹炰緥
 * @param  index: 瑕佸啓鍏ョ殑鍙傛暟鍦板潃 (e.g., 0x7005 for run_mode)
 * @param  value: 瑕佸啓鍏ョ殑鍗曞瓧鑺傛暟鍊?
 */
static void cybergear_write_u8_parameter(CyberGear_Motor_Instance* motor, uint16_t index, uint8_t value)
{
    uint8_t tx_data[8] = {0};
    
    // 鍙傛暟鍦板潃鏀惧湪 Byte0-1
    memcpy(&tx_data[0], &index, sizeof(uint16_t));
    
    // 鍗曞瓧鑺傚€兼斁鍦?Byte4
    tx_data[4] = value;

    // 閫氫俊绫诲瀷18锛屼富鏈篒D鍦╠ata2瀛楁锛岀洰鏍囨槸鐢垫満ID
    cybergear_can_send_raw(motor, 18, MASTER_CAN_ID, motor->can_id, tx_data);
}

/**
 * @brief   鍒濆鍖朇AN鍙戦€侀攣
 * @details RTOS鐜涓嬪垱寤篗utex锛岃８鏈虹幆澧冧笅涓虹┖鎿嶄綔
 */
static void cybergear_lock_init(void)
{
#ifdef USE_FREERTOS
    if (g_can_tx_mutex == NULL) // 闃叉閲嶅鍒涘缓
    {
        g_can_tx_mutex = xSemaphoreCreateMutex();
    }
#else
    // 瑁告満鐜涓嬶紝姝ゅ嚱鏁颁綋涓虹┖
    (void)0;    // 闃叉鏌愪簺缂栬瘧鍣ㄦ姤 "empty function body" 璀﹀憡
#endif
}

/**
 * @brief   鑾峰彇CAN鍙戦€侀攣
 * @details RTOS鐜涓嬭皟鐢?xSemaphoreTake锛岃８鏈虹幆澧冧笅鐩存帴杩斿洖鎴愬姛
 * @retval  CYBERGEAR_LOCK_SUCCESS (1) 琛ㄧず鎴愬姛, 0 琛ㄧず澶辫触/瓒呮椂
 */
static uint8_t cybergear_lock_tx(void)
{
#ifdef USE_FREERTOS
    if (g_can_tx_mutex == NULL) return 0; // 閿佹湭鍒濆鍖栵紝杩斿洖澶辫触
    
    // xSemaphoreTake 鎴愬姛杩斿洖 pdTRUE (1), 澶辫触杩斿洖 pdFALSE (0)
    // 鐩存帴杩斿洖缁撴灉 (杞崲涓?uint8_t)
    return (uint8_t)xSemaphoreTake(g_can_tx_mutex, pdMS_TO_TICKS(10));
#else
    // 瑁告満鐜涓嬶紝鎬绘槸杩斿洖鎴愬姛
    return CYBERGEAR_LOCK_SUCCESS;
#endif
}

/**
 * @brief   閲婃斁CAN鍙戦€侀攣
 * @details RTOS鐜涓嬭皟鐢?xSemaphoreGive锛岃８鏈虹幆澧冧笅涓虹┖鎿嶄綔
 */
static void cybergear_unlock_tx(void)
{
#ifdef USE_FREERTOS
    if (g_can_tx_mutex != NULL)
    {
        xSemaphoreGive(g_can_tx_mutex);
    }
#else
    // 瑁告満鐜涓嬶紝姝ゅ嚱鏁颁綋涓虹┖
    (void)0; // 闃叉璀﹀憡
#endif
}

