#include "main.h"
#include <ctype.h>
#include "utils.h"


SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;
LR1121_HandleTypeDef hlr1121;

int main(void)
{
	HAL_StatusTypeDef st;
	uint8_t rx_payload[255];
	uint8_t rx_len = 0U;
	int16_t rssi_dbm_x2 = 0;
	const uint32_t rx_timeout_rtc = 0x00000000UL; // No timeout, 0x000FFFFUL for max timeout
	LR1121_LoRaProfile rx_profile = {
		.frequency_hz = 2403000000UL,
		.modulation = {
			.sf = LR1121_LORA_SF7,
			.bw = LR1121_LORA_BW_400,
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

	st = LR1121_ConfigureLoRa(&hlr1121, &rx_profile);
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

	uart_log("LR1121 RX ready at %lu Hz, waiting for LoRa packets\r\n", rx_profile.frequency_hz);

	while (1)
	{
		st = LR1121_ReceiveLoRaPacket(&hlr1121,
												 rx_payload,
												 (uint8_t)sizeof(rx_payload),
												 &rx_len,
												 &rssi_dbm_x2,
												 rx_timeout_rtc);
		if (st == HAL_OK)
		{
			int16_t rssi_abs_x2 = (rssi_dbm_x2 < 0) ? (int16_t)(-rssi_dbm_x2) : rssi_dbm_x2;
			uart_log("RX done: RSSI=%d.%u dBm\r\n",
						 (int)(rssi_dbm_x2 / 2),
						 (unsigned int)(rssi_abs_x2 % 2U ? 5U : 0U));
			log_payload(rx_payload, rx_len);
			HAL_GPIO_TogglePin(APP_LED_PORT, APP_LED_PIN);
		}
		else if (st == HAL_TIMEOUT)
		{
			uart_log("RX timeout\r\n");
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