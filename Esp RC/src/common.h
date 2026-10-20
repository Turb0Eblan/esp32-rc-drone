#pragma once
#include <stdint.h>

// ============================================================
//  common.h — общие типы для Master и Slave
// ============================================================

// Структура пакета ESP-NOW: 16 каналов SBUS
struct SbusChannels {
  uint16_t ch[16];   // значения 1000–2000 мкс
};

// Размер должен быть ≤ 250 байт (лимит ESP-NOW)
static_assert(sizeof(SbusChannels) <= 250, "SbusChannels too large for ESP-NOW");

// SBUS константы
constexpr uint8_t  SBUS_HEADER      = 0x0F;
constexpr uint8_t  SBUS_FOOTER      = 0x00;
constexpr uint8_t  SBUS_PACKET_LEN  = 25;
constexpr uint32_t SBUS_BAUD        = 100000;
constexpr uint8_t  SBUS_TX_PIN      = 13;   // GPIO13 → FC RX pad

// Канальные константы
constexpr uint8_t  ARM_CHANNEL      = 4;
constexpr uint16_t ARM_ON_VALUE     = 1800;
constexpr uint16_t ARM_OFF_VALUE    = 1000;
constexpr uint16_t CH_CENTER        = 1500;
constexpr uint16_t CH_THR_MIN       = 900;
constexpr uint16_t CH_THR_MAX       = 2000;
