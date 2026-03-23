#ifndef LR1121_H
#define LR1121_H

#include "stm32l4xx_hal.h"
#include "lr1121_cmds.h"

#include <stdbool.h>
#include <stdint.h>

#define LR1121_RESET_PIN GPIO_PIN_0
#define LR1121_RESET_PORT GPIOA
#define LR1121_CS_PIN GPIO_PIN_8
#define LR1121_CS_PORT GPIOA
#define LR1121_BUSY_PIN GPIO_PIN_3
#define LR1121_BUSY_PORT GPIOB
#define LR1121_DIO1_PIN GPIO_PIN_4
#define LR1121_DIO1_PORT GPIOB

#define LR1121_STDBY_RC 0x00
#define LR1121_STDBY_XOSC 0x01
#define LR1121_PKT_TYPE_LORA 0x02
#define LR1121_LORA_NETWORK_PRIVATE 0x00
#define LR1121_LORA_NETWORK_PUBLIC 0x01
#define LR1121_SYSTEM_REG_MODE_LDO 0x00
#define LR1121_SYSTEM_REG_MODE_DCDC 0x01

#define LR1121_RFSW0_HIGH (1U << 0)
#define LR1121_RFSW1_HIGH (1U << 1)
#define LR1121_RFSW2_HIGH (1U << 2)
#define LR1121_RFSW3_HIGH (1U << 3)
#define LR1121_RFSW4_HIGH (1U << 4)

#define LR1121_IRQ_TX_DONE (1UL << 2)
#define LR1121_IRQ_RX_DONE (1UL << 3)
#define LR1121_IRQ_PREAMBLE_DETECTED (1UL << 4)
#define LR1121_IRQ_SYNC_WORD_HEADER_VALID (1UL << 5)
#define LR1121_IRQ_HEADER_ERROR (1UL << 6)
#define LR1121_IRQ_CRC_ERROR (1UL << 7)
#define LR1121_IRQ_TIMEOUT (1UL << 10)
#define LR1121_IRQ_CMD_ERROR (1UL << 22)
#define LR1121_IRQ_ERROR (1UL << 23)
#define LR1121_IRQ_LORA_RX_TIMESTAMP (1UL << 27)
#define LR1121_IRQ_ALL_MASK 0x0FFFFFFCUL

#define LR1121_SYS_ERR_HF_XOSC_START (1U << 5)

#define LR1121_TCXO_CTRL_3_0V 0x06

typedef enum
{
  LR1121_LORA_SF5 = 0x05,
  LR1121_LORA_SF6 = 0x06,
  LR1121_LORA_SF7 = 0x07,
  LR1121_LORA_SF8 = 0x08,
  LR1121_LORA_SF9 = 0x09,
  LR1121_LORA_SF10 = 0x0A,
  LR1121_LORA_SF11 = 0x0B,
  LR1121_LORA_SF12 = 0x0C
} LR1121_LoRaSf;

typedef enum
{
  LR1121_LORA_BW_125 = 0x04,
  LR1121_LORA_BW_250 = 0x05,
  LR1121_LORA_BW_500 = 0x06,
  LR1121_LORA_BW_200 = 0x0D,
  LR1121_LORA_BW_400 = 0x0E,
  LR1121_LORA_BW_800 = 0x0F
} LR1121_LoRaBw;

typedef enum
{
  LR1121_LORA_CR_4_5 = 0x01,
  LR1121_LORA_CR_4_6 = 0x02,
  LR1121_LORA_CR_4_7 = 0x03,
  LR1121_LORA_CR_4_8 = 0x04
} LR1121_LoRaCr;

typedef enum
{
  LR1121_LORA_LDRO_OFF = 0x00,
  LR1121_LORA_LDRO_ON = 0x01
} LR1121_LoRaLdro;

typedef enum
{
  LR1121_LORA_HEADER_EXPLICIT = 0x00,
  LR1121_LORA_HEADER_IMPLICIT = 0x01
} LR1121_LoRaHeaderType;

typedef enum
{
  LR1121_LORA_CRC_OFF = 0x00,
  LR1121_LORA_CRC_ON = 0x01
} LR1121_LoRaCrc;

typedef enum
{
  LR1121_LORA_IQ_STANDARD = 0x00,
  LR1121_LORA_IQ_INVERTED = 0x01
} LR1121_LoRaIq;

typedef enum
{
  LR1121_RAMP_48_US = 0x02,
  LR1121_RAMP_64_US = 0x03,
  LR1121_RAMP_80_US = 0x04,
  LR1121_RAMP_96_US = 0x05
} LR1121_RampTime;

typedef enum
{
  LR1121_PA_SEL_LP = 0x00,
  LR1121_PA_SEL_HP = 0x01,
  LR1121_PA_SEL_HF = 0x02
} LR1121_PaSel;

typedef enum
{
  LR1121_PA_REG_SUPPLY_VREG = 0x00,
  LR1121_PA_REG_SUPPLY_VBAT = 0x01
} LR1121_PaRegSupply;

typedef struct
{
  LR1121_LoRaSf sf;
  LR1121_LoRaBw bw;
  LR1121_LoRaCr cr;
  LR1121_LoRaLdro ldro;
} LR1121_LoRaModulationParams;

typedef struct
{
  uint16_t preamble_len;
  LR1121_LoRaHeaderType header_type;
  uint8_t payload_len;
  LR1121_LoRaCrc crc;
  LR1121_LoRaIq iq;
} LR1121_LoRaPacketParams;

typedef struct
{
  int8_t power_dbm;
  LR1121_RampTime ramp;
} LR1121_LoRaTxParams;

typedef struct
{
  uint32_t frequency_hz;
  LR1121_LoRaModulationParams modulation;
  LR1121_LoRaPacketParams packet;
} LR1121_LoRaProfile;

typedef struct
{
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *reset_port;
  uint16_t reset_pin;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
  GPIO_TypeDef *busy_port;
  uint16_t busy_pin;
  GPIO_TypeDef *dio1_port;
  uint16_t dio1_pin;
} LR1121_HandleTypeDef;

typedef struct
{
  uint8_t enable;
  uint8_t standby;
  uint8_t rx;
  uint8_t tx;
  uint8_t tx_hp;
  uint8_t tx_hf;
  uint8_t gnss;
  uint8_t wifi;
} LR1121_RfSwitchCfg;

typedef enum
{
  LR1121_DBG_NONE = 0,
  LR1121_DBG_CFG_STANDBY_RC,
  LR1121_DBG_CFG_CLEAR_IRQ,
  LR1121_DBG_CFG_SET_DIO_IRQ,
  LR1121_DBG_CFG_SET_TCXO,
  LR1121_DBG_CFG_CALIBRATE,
  LR1121_DBG_CFG_CALIBRATE_IMAGE,
  LR1121_DBG_CFG_SET_PKT_TYPE,
  LR1121_DBG_CFG_SET_RF_FREQ,
  LR1121_DBG_CFG_SET_MOD_PARAMS,
  LR1121_DBG_CFG_SET_PKT_PARAMS,
  LR1121_DBG_RX_SET_STOP_TIMEOUT_ON_PREAMBLE,
  LR1121_DBG_RX_SET_RX,
  LR1121_DBG_RX_WAIT_DIO1,
  LR1121_DBG_RX_GET_IRQ,
  LR1121_DBG_RX_GET_BUFFER_STATUS,
  LR1121_DBG_RX_READ_BUFFER,
  LR1121_DBG_RX_GET_RSSI,
  LR1121_DBG_RX_CLEAR_DONE_IRQ,
  LR1121_DBG_RX_CHECK_IRQ,
} LR1121_DebugStage;

typedef struct
{
  LR1121_DebugStage stage;
  HAL_StatusTypeDef status;
  uint16_t opcode;
  uint32_t irq;
} LR1121_DebugInfo;

HAL_StatusTypeDef LR1121_Init(LR1121_HandleTypeDef *dev);
void LR1121_Reset(LR1121_HandleTypeDef *dev);
bool LR1121_WaitWhileBusy(LR1121_HandleTypeDef *dev, uint32_t timeout_ms);

HAL_StatusTypeDef LR1121_WriteCommand(LR1121_HandleTypeDef *dev,
                                      uint16_t opcode,
                                      const uint8_t *payload,
                                      uint16_t length);
HAL_StatusTypeDef LR1121_ReadCommand(LR1121_HandleTypeDef *dev,
                                     uint16_t opcode,
                                     uint8_t *response,
                                     uint16_t length);

HAL_StatusTypeDef LR1121_WriteBuffer(LR1121_HandleTypeDef *dev, const uint8_t *data, uint16_t length);
HAL_StatusTypeDef LR1121_ReadBuffer(LR1121_HandleTypeDef *dev, uint8_t offset, uint8_t *data, uint16_t length);
HAL_StatusTypeDef LR1121_WriteRegMem32Mask(LR1121_HandleTypeDef *dev,
                                                uint32_t address,
                                                uint32_t mask,
                                                uint32_t data);

HAL_StatusTypeDef LR1121_SetStandby(LR1121_HandleTypeDef *dev, uint8_t standby_cfg);
HAL_StatusTypeDef LR1121_SetSleep(LR1121_HandleTypeDef *dev, uint8_t sleep_cfg, uint32_t sleep_time);
HAL_StatusTypeDef LR1121_SetFs(LR1121_HandleTypeDef *dev);
HAL_StatusTypeDef LR1121_SetTcxoMode(LR1121_HandleTypeDef *dev, uint8_t tune, uint32_t timeout);
HAL_StatusTypeDef LR1121_Calibrate(LR1121_HandleTypeDef *dev, uint8_t calib_mask);
HAL_StatusTypeDef LR1121_CalibrateImage(LR1121_HandleTypeDef *dev, uint8_t freq1, uint8_t freq2);
HAL_StatusTypeDef LR1121_GetStatus(LR1121_HandleTypeDef *dev, uint8_t *status);
HAL_StatusTypeDef LR1121_GetErrors(LR1121_HandleTypeDef *dev, uint16_t *errors);
HAL_StatusTypeDef LR1121_ClearErrors(LR1121_HandleTypeDef *dev);
HAL_StatusTypeDef LR1121_SetRegMode(LR1121_HandleTypeDef *dev, uint8_t reg_mode);
HAL_StatusTypeDef LR1121_SetDioAsRfSwitch(LR1121_HandleTypeDef *dev, const LR1121_RfSwitchCfg *cfg);
HAL_StatusTypeDef LR1121_SetDioIrqParams(LR1121_HandleTypeDef *dev, uint32_t dio1_mask, uint32_t dio2_mask);
HAL_StatusTypeDef LR1121_ClearIrqStatus(LR1121_HandleTypeDef *dev, uint32_t mask);
HAL_StatusTypeDef LR1121_GetIrqStatus(LR1121_HandleTypeDef *dev, uint32_t *mask);
HAL_StatusTypeDef LR1121_WaitForDio1Irq(LR1121_HandleTypeDef *dev, uint32_t timeout_ms);

HAL_StatusTypeDef LR1121_SetRfFrequency(LR1121_HandleTypeDef *dev, uint32_t freq_hz);
HAL_StatusTypeDef LR1121_SetPacketType(LR1121_HandleTypeDef *dev, uint8_t packet_type);
HAL_StatusTypeDef LR1121_SetLoRaPublicNetwork(LR1121_HandleTypeDef *dev, uint8_t network_type);
HAL_StatusTypeDef LR1121_SetModulationParamsLoRa(LR1121_HandleTypeDef *dev,
                                                     uint8_t sf,
                                                     uint8_t bw,
                                                     uint8_t cr,
                                                     uint8_t ldro);
HAL_StatusTypeDef LR1121_SetPacketParamsLoRa(LR1121_HandleTypeDef *dev,
                                                 uint16_t preamble_len,
                                                 uint8_t header_type,
                                                 uint8_t payload_len,
                                                 uint8_t crc_mode,
                                                 uint8_t invert_iq);
HAL_StatusTypeDef LR1121_SetRx(LR1121_HandleTypeDef *dev, uint32_t timeout_rtc_step);
HAL_StatusTypeDef LR1121_StopTimeoutOnPreamble(LR1121_HandleTypeDef *dev, uint8_t stop_on_preamble);
HAL_StatusTypeDef LR1121_GetRxBufferStatus(LR1121_HandleTypeDef *dev,
                                               uint8_t *payload_len,
                                               uint8_t *start_buffer_pointer);
HAL_StatusTypeDef LR1121_GetRssiInst(LR1121_HandleTypeDef *dev, uint8_t *rssi_raw);
HAL_StatusTypeDef LR1121_GetLoRaPacketStatus(LR1121_HandleTypeDef *dev,
                                                 int16_t *rssi_pkt_dbm_x2,
                                                 int8_t *snr_db,
                                                 int16_t *signal_rssi_dbm_x2);
HAL_StatusTypeDef LR1121_ConfigureLoRa(LR1121_HandleTypeDef *dev, const LR1121_LoRaProfile *profile);

HAL_StatusTypeDef LR1121_ReceiveLoRaPacket(LR1121_HandleTypeDef *dev,
                                               uint8_t *payload,
                                               uint8_t payload_max_len,
                                               uint8_t *payload_len,
                                               int16_t *rssi_dbm_x2,
                                               uint32_t timeout_rtc_step);
const LR1121_DebugInfo *LR1121_GetLastDebugInfo(void);

#endif