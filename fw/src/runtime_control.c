#include "runtime_control.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void start_uart_rx(RuntimeControlCtx *ctx)
{
  if ((ctx == NULL) || (ctx->uart == NULL))
  {
    return;
  }

  (void)HAL_UART_Receive_IT(ctx->uart, &ctx->rx_byte, 1U);
}

static void rx_fifo_push(RuntimeControlCtx *ctx, uint8_t ch)
{
  uint16_t next;

  if (ctx == NULL)
  {
    return;
  }

  next = (uint16_t)((ctx->rx_head + 1U) % (uint16_t)sizeof(ctx->rx_fifo));
  if (next == ctx->rx_tail)
  {
    return;
  }

  ctx->rx_fifo[ctx->rx_head] = ch;
  ctx->rx_head = next;
}

static bool rx_fifo_pop(RuntimeControlCtx *ctx, uint8_t *ch)
{
  if ((ctx == NULL) || (ch == NULL) || (ctx->rx_tail == ctx->rx_head))
  {
    return false;
  }

  *ch = ctx->rx_fifo[ctx->rx_tail];
  ctx->rx_tail = (uint16_t)((ctx->rx_tail + 1U) % (uint16_t)sizeof(ctx->rx_fifo));
  return true;
}

static LR1121_LoRaSf parse_sf(uint32_t value, bool *ok)
{
  *ok = true;
  switch (value)
  {
  case 5U:
    return LR1121_LORA_SF5;
  case 6U:
    return LR1121_LORA_SF6;
  case 7U:
    return LR1121_LORA_SF7;
  case 8U:
    return LR1121_LORA_SF8;
  case 9U:
    return LR1121_LORA_SF9;
  case 10U:
    return LR1121_LORA_SF10;
  case 11U:
    return LR1121_LORA_SF11;
  case 12U:
    return LR1121_LORA_SF12;
  default:
    *ok = false;
    return LR1121_LORA_SF7;
  }
}

static LR1121_LoRaCr parse_cr(const char *value, bool *ok)
{
  if ((strcmp(value, "45") == 0) || (strcmp(value, "4/5") == 0) || (strcmp(value, "5") == 0))
  {
    *ok = true;
    return LR1121_LORA_CR_4_5;
  }
  if ((strcmp(value, "46") == 0) || (strcmp(value, "4/6") == 0) || (strcmp(value, "6") == 0))
  {
    *ok = true;
    return LR1121_LORA_CR_4_6;
  }
  if ((strcmp(value, "47") == 0) || (strcmp(value, "4/7") == 0) || (strcmp(value, "7") == 0))
  {
    *ok = true;
    return LR1121_LORA_CR_4_7;
  }
  if ((strcmp(value, "48") == 0) || (strcmp(value, "4/8") == 0) || (strcmp(value, "8") == 0))
  {
    *ok = true;
    return LR1121_LORA_CR_4_8;
  }

  *ok = false;
  return LR1121_LORA_CR_4_5;
}

static LR1121_LoRaBw parse_bw(uint32_t value, bool *ok)
{
  *ok = true;
  switch (value)
  {
  case 125U:
  case 125000U:
    return LR1121_LORA_BW_125;
  case 250U:
  case 250000U:
    return LR1121_LORA_BW_250;
  case 500U:
  case 500000U:
    return LR1121_LORA_BW_500;
  case 200U:
  case 200000U:
    return LR1121_LORA_BW_200;
  case 400U:
  case 400000U:
    return LR1121_LORA_BW_400;
  case 800U:
  case 800000U:
    return LR1121_LORA_BW_800;
  default:
    *ok = false;
    return LR1121_LORA_BW_125;
  }
}

static const char *bw_to_string(LR1121_LoRaBw bw)
{
  switch (bw)
  {
  case LR1121_LORA_BW_125:
    return "125kHz";
  case LR1121_LORA_BW_250:
    return "250kHz";
  case LR1121_LORA_BW_500:
    return "500kHz";
  case LR1121_LORA_BW_200:
    return "200kHz";
  case LR1121_LORA_BW_400:
    return "400kHz";
  case LR1121_LORA_BW_800:
    return "800kHz";
  default:
    return "unknown";
  }
}

static uint8_t sf_to_u8(LR1121_LoRaSf sf)
{
  return (uint8_t)sf;
}

static uint8_t cr_to_u8(LR1121_LoRaCr cr)
{
  return (uint8_t)cr + 4U;
}

static void print_profile(const LR1121_LoRaProfile *profile)
{
  uart_log("CFG freq=%lu sf=%u cr=4/%u bw=%s preamble=%u payload=%u\\r\\n",
           (unsigned long)profile->frequency_hz,
           (unsigned int)sf_to_u8(profile->modulation.sf),
           (unsigned int)cr_to_u8(profile->modulation.cr),
           bw_to_string(profile->modulation.bw),
           (unsigned int)profile->packet.preamble_len,
           (unsigned int)profile->packet.payload_len);
}

static void print_help(void)
{
  uart_log("Commands:\\r\\n");
  uart_log("  SHOW\\r\\n");
  uart_log("  PING\\r\\n");
  uart_log("  RX START\\r\\n");
  uart_log("  RX STOP\\r\\n");
  uart_log("  SET FREQ <hz>\\r\\n");
  uart_log("  SET SF <5..12>\\r\\n");
  uart_log("  SET CR <45|46|47|48 or 4/5..4/8>\\r\\n");
  uart_log("  SET BW <125|250|500|200|400|800 kHz>\\r\\n");
  uart_log("  SET PREAMBLE <1..65535>\\r\\n");
  uart_log("  SET PAYLOAD <1..255>\\r\\n");
  uart_log("  HELP\\r\\n");
}

static HAL_StatusTypeDef apply_profile(RuntimeControlCtx *ctx,
                                       const LR1121_LoRaProfile *candidate)
{
  HAL_StatusTypeDef st = LR1121_ConfigureLoRa(ctx->radio, candidate);

  if (st == HAL_OK)
  {
    ctx->profile = *candidate;
  }

  return st;
}

static void process_command(RuntimeControlCtx *ctx, char *line)
{
  char key[16];
  char value[32];
  LR1121_LoRaProfile candidate = ctx->profile;
  HAL_StatusTypeDef st;
  char *p = line;

  while ((*p != '\0') && isspace((unsigned char)*p))
  {
    p++;
  }

  if (*p == '\0')
  {
    return;
  }

  if ((strcmp(p, "HELP") == 0) || (strcmp(p, "?") == 0))
  {
    print_help();
    return;
  }

  if (strcmp(p, "PING") == 0)
  {
    uart_log("PONG\\r\\n");
    return;
  }

  if (strcmp(p, "RX START") == 0)
  {
    ctx->rx_enabled = true;
    uart_log("OK RX START\\r\\n");
    return;
  }

  if (strcmp(p, "RX STOP") == 0)
  {
    ctx->rx_enabled = false;
    uart_log("OK RX STOP\\r\\n");
    return;
  }

  if (strcmp(p, "SHOW") == 0)
  {
    print_profile(&ctx->profile);
    return;
  }

  if (sscanf(p, "SET %15s %31s", key, value) != 2)
  {
    uart_log("ERR bad syntax, use HELP\\r\\n");
    return;
  }

  if (strcmp(key, "FREQ") == 0)
  {
    unsigned long f = strtoul(value, NULL, 10);
    if ((f < 150000000UL) || (f > 2500000000UL))
    {
      uart_log("ERR FREQ out of range\\r\\n");
      return;
    }
    candidate.frequency_hz = (uint32_t)f;
  }
  else if (strcmp(key, "SF") == 0)
  {
    bool ok;
    unsigned long sf = strtoul(value, NULL, 10);
    candidate.modulation.sf = parse_sf((uint32_t)sf, &ok);
    if (!ok)
    {
      uart_log("ERR SF must be 5..12\\r\\n");
      return;
    }
  }
  else if (strcmp(key, "CR") == 0)
  {
    bool ok;
    candidate.modulation.cr = parse_cr(value, &ok);
    if (!ok)
    {
      uart_log("ERR CR use 45/46/47/48 or 4/5..4/8\\r\\n");
      return;
    }
  }
  else if (strcmp(key, "BW") == 0)
  {
    bool ok;
    unsigned long bw = strtoul(value, NULL, 10);
    candidate.modulation.bw = parse_bw((uint32_t)bw, &ok);
    if (!ok)
    {
      uart_log("ERR BW use 125/250/500/200/400/800\\r\\n");
      return;
    }
  }
  else if (strcmp(key, "PREAMBLE") == 0)
  {
    unsigned long preamble = strtoul(value, NULL, 10);
    if ((preamble == 0UL) || (preamble > 65535UL))
    {
      uart_log("ERR PREAMBLE must be 1..65535\\r\\n");
      return;
    }
    candidate.packet.preamble_len = (uint16_t)preamble;
  }
  else if (strcmp(key, "PAYLOAD") == 0)
  {
    unsigned long payload_len = strtoul(value, NULL, 10);
    if ((payload_len == 0UL) || (payload_len > 255UL))
    {
      uart_log("ERR PAYLOAD must be 1..255\\r\\n");
      return;
    }
    candidate.packet.payload_len = (uint8_t)payload_len;
  }
  else
  {
    uart_log("ERR unknown key '%s'\\r\\n", key);
    return;
  }

  st = apply_profile(ctx, &candidate);
  if (st != HAL_OK)
  {
    const LR1121_DebugInfo *dbg = LR1121_GetLastDebugInfo();
    uart_log("ERR apply failed st=%d stage=%u op=0x%04X irq=0x%08lX\\r\\n",
             (int)st,
             (unsigned int)dbg->stage,
             (unsigned int)dbg->opcode,
             dbg->irq);
    return;
  }

  uart_log("OK updated %s=%s\\r\\n", key, value);
  print_profile(&ctx->profile);
}

void RuntimeControl_Init(RuntimeControlCtx *ctx,
                         LR1121_HandleTypeDef *radio,
                         UART_HandleTypeDef *uart,
                         const LR1121_LoRaProfile *initial_profile)
{
  if ((ctx == NULL) || (radio == NULL) || (uart == NULL) || (initial_profile == NULL))
  {
    return;
  }

  memset(ctx, 0, sizeof(*ctx));
  ctx->radio = radio;
  ctx->uart = uart;
  ctx->profile = *initial_profile;
  ctx->rx_enabled = true;
  start_uart_rx(ctx);
}

HAL_StatusTypeDef RuntimeControl_ApplyInitial(RuntimeControlCtx *ctx)
{
  if (ctx == NULL)
  {
    return HAL_ERROR;
  }

  return LR1121_ConfigureLoRa(ctx->radio, &ctx->profile);
}

void RuntimeControl_PrintWelcome(const RuntimeControlCtx *ctx)
{
  if (ctx == NULL)
  {
    return;
  }

  uart_log("LR1121 RX ready. Type HELP + Enter for commands.\\r\\n");
  print_profile(&ctx->profile);
}

void RuntimeControl_Poll(RuntimeControlCtx *ctx)
{
  uint8_t ch;

  if (ctx == NULL)
  {
    return;
  }

  while (rx_fifo_pop(ctx, &ch))
  {
    if ((ch == '\r') || (ch == '\n'))
    {
      if (ctx->cmd_idx > 0U)
      {
        ctx->cmd_line[ctx->cmd_idx] = '\0';
        process_command(ctx, ctx->cmd_line);
        ctx->cmd_idx = 0U;
      }
      continue;
    }

    if (ctx->cmd_idx < (uint16_t)(sizeof(ctx->cmd_line) - 1U))
    {
      ctx->cmd_line[ctx->cmd_idx++] = (char)toupper((unsigned char)ch);
    }
    else
    {
      ctx->cmd_idx = 0U;
      uart_log("ERR command too long\\r\\n");
    }
  }
}

void RuntimeControl_OnUartRxCplt(RuntimeControlCtx *ctx, UART_HandleTypeDef *huart)
{
  if ((ctx == NULL) || (huart == NULL) || (ctx->uart != huart))
  {
    return;
  }

  rx_fifo_push(ctx, ctx->rx_byte);
  start_uart_rx(ctx);
}

void RuntimeControl_OnUartError(RuntimeControlCtx *ctx, UART_HandleTypeDef *huart)
{
  if ((ctx == NULL) || (huart == NULL) || (ctx->uart != huart))
  {
    return;
  }

  (void)HAL_UART_AbortReceive(huart);
  start_uart_rx(ctx);
}

bool RuntimeControl_IsRxEnabled(const RuntimeControlCtx *ctx)
{
  if (ctx == NULL)
  {
    return false;
  }

  return ctx->rx_enabled;
}
