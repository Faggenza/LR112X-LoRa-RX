#ifndef RUNTIME_CONTROL_H
#define RUNTIME_CONTROL_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  LR1121_HandleTypeDef *radio;
  UART_HandleTypeDef *uart;
  LR1121_LoRaProfile profile;
  uint8_t rx_byte;
  uint8_t rx_fifo[256];
  volatile uint16_t rx_head;
  volatile uint16_t rx_tail;
  char cmd_line[96];
  uint16_t cmd_idx;
  bool rx_enabled;
} RuntimeControlCtx;

void RuntimeControl_Init(RuntimeControlCtx *ctx,
                         LR1121_HandleTypeDef *radio,
                         UART_HandleTypeDef *uart,
                         const LR1121_LoRaProfile *initial_profile);
HAL_StatusTypeDef RuntimeControl_ApplyInitial(RuntimeControlCtx *ctx);
void RuntimeControl_PrintWelcome(const RuntimeControlCtx *ctx);
void RuntimeControl_Poll(RuntimeControlCtx *ctx);
void RuntimeControl_OnUartRxCplt(RuntimeControlCtx *ctx, UART_HandleTypeDef *huart);
void RuntimeControl_OnUartError(RuntimeControlCtx *ctx, UART_HandleTypeDef *huart);
bool RuntimeControl_IsRxEnabled(const RuntimeControlCtx *ctx);

#endif
