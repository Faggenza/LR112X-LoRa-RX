#include "utils.h"

void log_payload(const uint8_t *payload, uint8_t len)
{
	uart_log("Payload (%u bytes) HEX:", (unsigned int)len);
	for (uint8_t i = 0; i < len; i++)
	{
		uart_log(" %02X", (unsigned int)payload[i]);
	}
	uart_log("\r\n");

	uart_log("Payload ASCII: ");
	for (uint8_t i = 0; i < len; i++)
	{
		char c = isprint((int)payload[i]) ? (char)payload[i] : '.';
		uart_log("%c", c);
	}
	uart_log("\r\n");
}

void uart_log(const char *fmt, ...)
{
  char buffer[160];
  int len;
  va_list args;

  va_start(args, fmt);
  len = vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  if (len <= 0)
  {
    return;
  }

  if ((size_t)len > sizeof(buffer))
  {
    len = (int)(sizeof(buffer) - 1U);
  }

  (void)HAL_UART_Transmit(&huart2, (uint8_t *)buffer, (uint16_t)len, 100);
}
