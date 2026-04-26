#include "hmi_panel.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "cmsis_os2.h"
#include "mechanism_task.h"
#include "remote_driver.h"
#include "robot_data.h"

HmiPanelStatus_t g_hmi_panel_status = {0};

static UART_HandleTypeDef *s_hmi_uart = NULL;
static uint8_t s_rx_byte = 0U;
static uint8_t s_rx_buffer[HMI_PANEL_RX_BUFFER_SIZE];
static uint8_t s_rx_index = 0U;
static volatile uint8_t s_message_ready = 0U;

static const uint8_t s_hmi_end[3] = {0xFFU, 0xFFU, 0xFFU};

static void hmi_trim_line(char *line)
{
    size_t len;

    if (line == NULL) {
        return;
    }

    len = strlen(line);
    while ((len > 0U) && ((line[len - 1U] == '\r') || (line[len - 1U] == '\n') || (line[len - 1U] == ' '))) {
        line[len - 1U] = '\0';
        len--;
    }
}

static const char *hmi_mode_text(chassis_mode_e mode)
{
    switch (mode) {
        case CHASSIS_MODE_AUTO: return "AUTO";
        case CHASSIS_MODE_MANUAL: return "MANUAL";
        case CHASSIS_MODE_SERVE: return "SERVE";
        case CHASSIS_MODE_STANDBY:
        default: return "STANDBY";
    }
}

static const char *hmi_state_text(PC_state state)
{
    switch (state) {
        case START: return "START";
        case OVER: return "OVER";
        case START_BUSY: return "START_BUSY";
        case OVER_BUSY: return "OVER_BUSY";
        case SYS_ERROR_SENSOR_JAM: return "ERR_SENSOR";
        case SYS_ERROR_MOTOR_COMMS: return "ERR_MOTOR";
        default: return "UNKNOWN";
    }
}

static uint16_t hmi_state_color(PC_state state)
{
    switch (state) {
        case START:
        case OVER:
            return 2016U;
        case START_BUSY:
        case OVER_BUSY:
            return 65504U;
        case SYS_ERROR_SENSOR_JAM:
        case SYS_ERROR_MOTOR_COMMS:
            return 63488U;
        default:
            return 65535U;
    }
}

static void hmi_send_raw_command(const char *cmd)
{
    if ((s_hmi_uart == NULL) || (cmd == NULL)) {
        return;
    }

    (void)HAL_UART_Transmit(s_hmi_uart, (uint8_t *)cmd, (uint16_t)strlen(cmd), 20U);
    (void)HAL_UART_Transmit(s_hmi_uart, (uint8_t *)s_hmi_end, sizeof(s_hmi_end), 20U);
}

static void hmi_set_text(const char *object_name, const char *text)
{
    char cmd[96];

    if ((object_name == NULL) || (text == NULL)) {
        return;
    }

    (void)snprintf(cmd, sizeof(cmd), "%s.txt=\"%s\"", object_name, text);
    hmi_send_raw_command(cmd);
}

static void hmi_set_text_if_changed(const char *object_name,
                                    const char *text,
                                    char *last_text,
                                    size_t last_text_size)
{
    if ((text == NULL) || (last_text == NULL) || (last_text_size == 0U)) {
        return;
    }

    if (strncmp(last_text, text, last_text_size) == 0) {
        return;
    }

    hmi_set_text(object_name, text);
    (void)snprintf(last_text, last_text_size, "%s", text);
}

static void hmi_set_textf_if_changed(const char *object_name,
                                     char *last_text,
                                     size_t last_text_size,
                                     const char *format,
                                     ...)
{
    char text[72];
    va_list args;

    va_start(args, format);
    (void)vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    hmi_set_text_if_changed(object_name, text, last_text, last_text_size);
}

static void hmi_set_color(const char *object_name, uint16_t color)
{
    char cmd[48];

    if (object_name == NULL) {
        return;
    }

    (void)snprintf(cmd, sizeof(cmd), "%s.pco=%u", object_name, (unsigned int)color);
    hmi_send_raw_command(cmd);
}

static void hmi_set_color_if_changed(const char *object_name, uint16_t color, uint16_t *last_color)
{
    if ((last_color == NULL) || (*last_color == color)) {
        return;
    }

    hmi_set_color(object_name, color);
    *last_color = color;
}

static void hmi_apply_received_message(void)
{
    char local[HMI_PANEL_RX_BUFFER_SIZE];
    float target_x = 0.0f;
    float target_y = 0.0f;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(local, s_rx_buffer, sizeof(local));
    s_message_ready = 0U;
    s_rx_index = 0U;
    memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
    if (primask == 0U) {
        __enable_irq();
    }

    hmi_trim_line(local);

    if (strcmp(local, "start") == 0) {
        g_hmi_panel_status.cmd_start = 1U;
        Set_System_Doit_State(START);
        hmi_set_text("t_msg", "START command");
        hmi_set_color("t_msg", 2016U);
    } else if (strcmp(local, "end") == 0) {
        g_hmi_panel_status.cmd_end = 1U;
        Set_System_Doit_State(OVER);
        hmi_set_text("t_msg", "END command");
        hmi_set_color("t_msg", 65504U);
    } else if (sscanf(local, "X:%f,Y:%f", &target_x, &target_y) == 2) {
        g_hmi_panel_status.target_x = target_x;
        g_hmi_panel_status.target_y = target_y;
        g_hmi_panel_status.coords_updated = 1U;
        Robot_Data_SetTarget(target_x, target_y);
        hmi_set_text("t_msg", "Target updated");
        hmi_set_color("t_msg", 2016U);
    } else {
        hmi_set_text("t_msg", "Unknown command");
        hmi_set_color("t_msg", 63488U);
    }
}

void HmiPanel_Init(UART_HandleTypeDef *huart)
{
    s_hmi_uart = huart;
    s_rx_index = 0U;
    s_message_ready = 0U;
    memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
    HmiPanel_RestartRx();
    hmi_send_raw_command("bkcmd=0");
    hmi_send_raw_command("page main");
}

uint8_t HmiPanel_IsUart(const UART_HandleTypeDef *huart)
{
    return (uint8_t)((huart != NULL) &&
                     (s_hmi_uart != NULL) &&
                     (huart->Instance == s_hmi_uart->Instance));
}

void HmiPanel_RestartRx(void)
{
    if (s_hmi_uart != NULL) {
        (void)HAL_UART_Receive_IT(s_hmi_uart, &s_rx_byte, 1U);
    }
}

void HmiPanel_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (HmiPanel_IsUart(huart) == 0U) {
        return;
    }

    if (s_rx_index < (HMI_PANEL_RX_BUFFER_SIZE - 1U)) {
        s_rx_buffer[s_rx_index++] = s_rx_byte;
        if (s_rx_byte == 0x0AU) {
            s_rx_buffer[s_rx_index] = '\0';
            s_message_ready = 1U;
        }
    } else {
        s_rx_index = 0U;
        memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
    }

    HmiPanel_RestartRx();
}

void HmiPanel_Process(void)
{
    if (s_message_ready != 0U) {
        hmi_apply_received_message();
    }
}

void HmiPanel_UpdateScreen(void)
{
    Robot_Pose_t pose;
    Robot_Target_t target;
    remote_engineer_t rc_data = {0};
    PC_state state;
    static char s_last_mode[16] = "";
    static char s_last_state[20] = "";
    static char s_last_pose[72] = "";
    static char s_last_vel[72] = "";
    static char s_last_target[72] = "";
    static char s_last_rc[72] = "";
    static char s_last_tick[32] = "";
    static uint16_t s_last_state_color = 0U;

    Robot_Data_GetPoseSnapshot(&pose);
    Robot_Data_GetTargetSnapshot(&target);
    (void)Remote_GetEngineerData(&rc_data);
    state = Get_System_Doit_State();

    hmi_set_text_if_changed("t_mode", hmi_mode_text(rc_data.mode), s_last_mode, sizeof(s_last_mode));
    hmi_set_text_if_changed("t_state", hmi_state_text(state), s_last_state, sizeof(s_last_state));
    hmi_set_color_if_changed("t_state", hmi_state_color(state), &s_last_state_color);
    hmi_set_textf_if_changed("t_pose", s_last_pose, sizeof(s_last_pose), "X:%ld Y:%ld A:%ld",
                             (long)pose.x, (long)pose.y, (long)pose.angle);
    hmi_set_textf_if_changed("t_vel", s_last_vel, sizeof(s_last_vel), "Vx:%ld Vy:%ld Vr:%ld",
                             (long)pose.vx, (long)pose.vy, (long)(pose.vr * 1000.0f));
    hmi_set_textf_if_changed("t_target", s_last_target, sizeof(s_last_target), "X:%ld Y:%ld",
                             (long)target.target_x, (long)target.target_y);
    hmi_set_textf_if_changed("t_rc", s_last_rc, sizeof(s_last_rc), "vx:%ld vy:%ld vw:%ld",
                             (long)(rc_data.vx * 100.0f),
                             (long)(rc_data.vy * 100.0f),
                             (long)(rc_data.vw * 100.0f));
    hmi_set_textf_if_changed("t_tick", s_last_tick, sizeof(s_last_tick), "tick:%lu",
                             (unsigned long)(HAL_GetTick() / 1000U));
}
