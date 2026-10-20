// ============================================================
//  slave/main.cpp
//
//  Принимает значения 16 каналов от Master по ESP-NOW,
//  пакует SBUS (25 байт) и выдаёт на FC через Serial2
//  (GPIO13, инвертированный, 100000 бод 8E2).
//
//  ПРИ ПЕРВОМ ЗАПУСКЕ: смотри Serial Monitor — он напечатает
//  "Slave MAC: XX:XX:XX:XX:XX:XX" — вставь этот адрес в
//  SLAVE_MAC в master/main.cpp
// ============================================================

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "common.h"

// SBUS выход на FC
// GPIO13 → FC RX pad, инвертированный сигнал
// Serial2.begin(100000, SERIAL_8E2, -1, SBUS_TX_PIN, true)

// ─────────────────────────────────────────────
//  Состояние
// ─────────────────────────────────────────────
static SbusChannels rxChannels;          // последние принятые каналы
static bool         hasData    = false;  // получен хоть один пакет
static unsigned long lastRx    = 0;      // время последнего приёма

static uint8_t  sbusBuf[SBUS_PACKET_LEN]; // рабочий буфер пакета

// ─────────────────────────────────────────────
//  Упаковка каналов в SBUS пакет
// ─────────────────────────────────────────────
void packSBUS(const SbusChannels& src, uint8_t* out) {
  out[0] = SBUS_HEADER;

  // Маппинг 1000–2000 → 172–1811 (11-битный диапазон SBUS)
  uint16_t ch[16];
  for (int i = 0; i < 16; i++) {
    ch[i] = static_cast<uint16_t>(
      map(src.ch[i], 1000, 2000, 172, 1811));
    ch[i] = constrain(ch[i], 172, 1811);
  }

  out[1]  =  ch[0] & 0xFF;
  out[2]  = (ch[0] >> 8) | (ch[1]  << 3);
  out[3]  = (ch[1] >> 5) | (ch[2]  << 6);
  out[4]  = (ch[2] >> 2);
  out[5]  = (ch[2] >> 10)| (ch[3]  << 1);
  out[6]  = (ch[3] >> 7) | (ch[4]  << 4);
  out[7]  = (ch[4] >> 4) | (ch[5]  << 7);
  out[8]  =  ch[5] >> 1;
  out[9]  = (ch[5] >> 9) | (ch[6]  << 2);
  out[10] = (ch[6] >> 6) | (ch[7]  << 5);
  out[11] =  ch[7] >> 3;
  out[12] =  ch[8] & 0xFF;
  out[13] = (ch[8] >> 8) | (ch[9]  << 3);
  out[14] = (ch[9] >> 5) | (ch[10] << 6);
  out[15] =  ch[10] >> 2;
  out[16] = (ch[10] >> 10)|(ch[11] << 1);
  out[17] = (ch[11] >> 7) | (ch[12] << 4);
  out[18] = (ch[12] >> 4) | (ch[13] << 7);
  out[19] =  ch[13] >> 1;
  out[20] = (ch[13] >> 9) | (ch[14] << 2);
  out[21] = (ch[14] >> 6) | (ch[15] << 5);
  out[22] =  ch[15] >> 3;
  out[23] = 0x00;  // флаги: failsafe выключен
  out[24] = SBUS_FOOTER;
}

// ─────────────────────────────────────────────
//  Нейтральный пакет (для старта)
// ─────────────────────────────────────────────
void setNeutralChannels(SbusChannels& c) {
  c.ch[0] = CH_CENTER;   // Roll
  c.ch[1] = CH_CENTER;   // Pitch
  c.ch[2] = CH_THR_MIN;  // Throttle
  c.ch[3] = CH_CENTER;   // Yaw
  c.ch[ARM_CHANNEL] = ARM_OFF_VALUE;
  for (int i = 5; i < 16; i++) c.ch[i] = CH_CENTER;
}

// ─────────────────────────────────────────────
//  ESP-NOW callback: вызывается при получении пакета
//  ВАЖНО: работает в контексте Wi-Fi task (IRAM),
//  поэтому только копируем данные, не делаем Serial
// ─────────────────────────────────────────────
void onDataRecv(const uint8_t* mac,
                const uint8_t* data,
                int len)
{
  if (len != static_cast<int>(sizeof(SbusChannels))) return;
  memcpy(&rxChannels, data, sizeof(SbusChannels));
  hasData = true;
  lastRx  = millis();
}

// ─────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== SLAVE starting ===");

  // Сразу печатаем MAC — нужен для master/main.cpp
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.print("Slave MAC: ");
  Serial.println(WiFi.macAddress());

  // ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] Init FAILED — halt");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onDataRecv);
  Serial.println("[ESPNOW] Ready, waiting for Master...");

  // SBUS Serial2: инвертированный, 100000 бод, 8E2
  // -1 = RX не используется, SBUS_TX_PIN = GPIO13
  Serial2.begin(SBUS_BAUD, SERIAL_8E2, -1, SBUS_TX_PIN, true);

  // Стартовые нейтральные пакеты
  setNeutralChannels(rxChannels);
  packSBUS(rxChannels, sbusBuf);
  for (int i = 0; i < 10; i++) {
    Serial2.write(sbusBuf, SBUS_PACKET_LEN);
    delay(7);
  }

  Serial.println("[SBUS] Output started on GPIO13");
}

// ─────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────
void loop() {
  static unsigned long lastSbus = 0;
  static unsigned long lastLog  = 0;
  static unsigned long lastWarn = 0;

  // ── Отправка SBUS на FC каждые 7 мс ──────
  if (millis() - lastSbus >= 7) {
    packSBUS(rxChannels, sbusBuf);
    Serial2.write(sbusBuf, SBUS_PACKET_LEN);
    lastSbus = millis();
  }

  // ── Лог при наличии данных (каждые 2 сек) ─
  if (hasData && millis() - lastLog >= 2000) {
    Serial.printf("[SLV] ch1=%u ch2=%u thr=%u yaw=%u arm=%u\n",
      rxChannels.ch[0], rxChannels.ch[1],
      rxChannels.ch[2], rxChannels.ch[3],
      rxChannels.ch[ARM_CHANNEL]);
    lastLog = millis();
  }

  // ── Предупреждение при молчании мастера ──
  if (hasData && millis() - lastRx > 2000
              && millis() - lastWarn > 5000)
  {
    Serial.printf("[SLV] WARNING: no data from Master >%lu ms\n",
      millis() - lastRx);
    lastWarn = millis();
  }
}