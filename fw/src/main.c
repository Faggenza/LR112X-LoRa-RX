#include "main.h"
#include <ctype.h>
#include "runtime_control.h"
#include "utils.h"


SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;
LR1121_HandleTypeDef hlr1121;
static RuntimeControlCtx g_runtime;

int main(void)
{
	HAL_StatusTypeDef st;
	uint8_t rx_payload[255];
	uint8_t rx_len = 0U;
	int16_t rssi_dbm_x2 = 0;
	const uint32_t rx_timeout_rtc = 32768UL;
	const LR1121_LoRaProfile default_profile = {
		.frequency_hz = 868030000UL,
		.modulation = {
			.sf = LR1121_LORA_SF12,
			.bw = LR1121_LORA_BW_125,
			.cr = LR1121_LORA_CR_4_5,
			.ldro = LR1121_LORA_LDRO_OFF,
		},
		.packet = {
			.preamble_len = 8,
			.header_type = LR1121_LORA_HEADER_EXPLICIT,
			.payload_len = 255,
			.crc = LR1121_LORA_CRC_OFF,
			.iq = LR1121_LORA_IQ_STANDARD,
		},
	};

	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_USART2_UART_Init();

	hlr1121.hspi = &hspi1;
	hlr1121.reset_port = LR1121_RESET_PORT;
	hlr1121.reset_pin = LR1121_RESET_PIN;
	hlr1121.cs_port = LR1121_CS_PORT;
	hlr1121.cs_pin = LR1121_CS_PIN;
	hlr1121.busy_port = LR1121_BUSY_PORT;
	hlr1121.busy_pin = LR1121_BUSY_PIN;
	hlr1121.dio1_port = LR1121_DIO1_PORT;
	hlr1121.dio1_pin = LR1121_DIO1_PIN;

	st = LR1121_Init(&hlr1121);
	if (st != HAL_OK)
	{
		uart_log("LR1121 init failed: %d busy=%u dio1=%u\r\n",
				 (int)st,
				 (unsigned int)HAL_GPIO_ReadPin(LR1121_BUSY_PORT, LR1121_BUSY_PIN),
				 (unsigned int)HAL_GPIO_ReadPin(LR1121_DIO1_PORT, LR1121_DIO1_PIN));
		Error_Handler();
	}

	RuntimeControl_Init(&g_runtime, &hlr1121, &huart2, &default_profile);
	st = RuntimeControl_ApplyInitial(&g_runtime);
	if (st != HAL_OK)
	{
		const LR1121_DebugInfo *dbg = LR1121_GetLastDebugInfo();
		uint16_t sys_err = 0U;
		(void)LR1121_GetErrors(&hlr1121, &sys_err);
		uart_log("LR1121 LoRa cfg failed: %d stage=%u op=0x%04X irq=0x%08lX sys_err=0x%04X\r\n",
				 (int)st,
				 (unsigned int)dbg->stage,
				 (unsigned int)dbg->opcode,
				 dbg->irq,
				 (unsigned int)sys_err);
		Error_Handler();
	}

	RuntimeControl_PrintWelcome(&g_runtime);

	while (1)
	{
		RuntimeControl_Poll(&g_runtime);

		if (!RuntimeControl_IsRxEnabled(&g_runtime))
		{
			HAL_Delay(10);
			continue;
		}

		st = LR1121_ReceiveLoRaPacket(&hlr1121,
												 rx_payload,
												 (uint8_t)sizeof(rx_payload),
												 &rx_len,
												 &rssi_dbm_x2,
												 rx_timeout_rtc);
		if (st == HAL_OK)
		{
			HAL_GPIO_TogglePin(APP_LED_PORT, APP_LED_PIN);
			int16_t rssi_abs_x2 = (rssi_dbm_x2 < 0) ? (int16_t)(-rssi_dbm_x2) : rssi_dbm_x2;
			uart_log("RX done: RSSI=%d.%u dBm\r\n",
						 (int)(rssi_dbm_x2 / 2),
						 (unsigned int)(rssi_abs_x2 % 2U ? 5U : 0U));
			log_payload(rx_payload, rx_len);
			HAL_GPIO_TogglePin(APP_LED_PORT, APP_LED_PIN);
		}
		else if (st == HAL_TIMEOUT)
		{
			/* Timeout is expected in periodic RX windows. */
		}
		else
		{
			const LR1121_DebugInfo *dbg = LR1121_GetLastDebugInfo();
			uint32_t irq_mask = 0U;
			uint16_t sys_err = 0U;
			(void)LR1121_GetIrqStatus(&hlr1121, &irq_mask);
			(void)LR1121_GetErrors(&hlr1121, &sys_err);
			uart_log("RX failed: %d irq=0x%08lX busy=%u dio1=%u stage=%u op=0x%04X dbg_irq=0x%08lX sys_err=0x%04X\r\n",
					 (int)st,
					 irq_mask,
					 (unsigned int)HAL_GPIO_ReadPin(LR1121_BUSY_PORT, LR1121_BUSY_PIN),
					 (unsigned int)HAL_GPIO_ReadPin(LR1121_DIO1_PORT, LR1121_DIO1_PIN),
					 (unsigned int)dbg->stage,
					 (unsigned int)dbg->opcode,
					 dbg->irq,
					 (unsigned int)sys_err);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	RuntimeControl_OnUartRxCplt(&g_runtime, huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	RuntimeControl_OnUartError(&g_runtime, huart);
}

void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);
}