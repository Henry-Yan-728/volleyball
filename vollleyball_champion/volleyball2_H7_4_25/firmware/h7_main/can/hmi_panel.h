#ifndef HMI_PANEL_H
#define HMI_PANEL_H

#include "main.h"

#define HMI_PANEL_RX_BUFFER_SIZE 64U

typedef struct {
    float target_x;
    float target_y;
    uint8_t cmd_start;
    uint8_t cmd_end;
    uint8_t coords_updated;
} HmiPanelStatus_t;

extern HmiPanelStatus_t g_hmi_panel_status;

void HmiPanel_Init(UART_HandleTypeDef *huart);
void HmiPanel_RxCpltCallback(UART_HandleTypeDef *huart);
void HmiPanel_RestartRx(void);
uint8_t HmiPanel_IsUart(const UART_HandleTypeDef *huart);
void HmiPanel_Process(void);
void HmiPanel_UpdateScreen(void);

#endif
