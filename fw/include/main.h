#ifndef MAIN_H
#define MAIN_H

#include "stm32l4xx_hal.h"
#include "lr1121.h"

#define APP_LED_PIN GPIO_PIN_1
#define APP_LED_PORT GPIOC

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;
extern LR1121_HandleTypeDef hlr1121;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_SPI1_Init(void);
void MX_USART2_UART_Init(void);
void Error_Handler(void);
void SysTick_Handler(void);
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);

void uart_log(const char *fmt, ...);

#endif
