//
// Created by ??? on 2025/11/20.
//
#ifndef R1_CHASSIS_TASK_COMMAND_H
#define R1_CHASSIS_TASK_COMMAND_H
#include "main.h"
#include <string.h>
#include "cmsis_os2.h"
#define COMMAND_LENGTH 10U
#define UART_RX_MESSAGE_MAX_SIZE 64U
extern uint8_t remote_Buffer[];
extern osMessageQueueId_t remote_queueHandle;
typedef struct {
    uint8_t data[UART_RX_MESSAGE_MAX_SIZE];
    uint16_t size;
} UartRxMessage_t;
void Command_Reset(void);
uint8_t Command_Write(const uint8_t *data, uint8_t length);
uint8_t Command_GetCommand(uint8_t *command);
#endif // R1_CHASSIS_TASK_COMMAND_H
