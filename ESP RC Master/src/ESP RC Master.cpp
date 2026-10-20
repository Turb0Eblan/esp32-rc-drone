// ============================================================
//  master/main.cpp
//
//  Физическая модель управления (velocity + damping):
//    Команда добавляет импульс скорости к оси.
//    Скорость интегрируется в отклонение канала.
//    Damping постепенно гасит скорость → канал сам
//    возвращается к нейтрали, как Angle mode на FC.
//
//  Параметры физики (подбирай под себя):
//    IMPULSE_KICK    — сила толчка за команду
//    DAMPING         — коэффициент затухания (0.0–1.0)
//                      чем выше → быстрее гасится
//    SPRING          — сила возврата к нейтрали
//                      чем выше → жёстче центровка
//    PHYSICS_DT_MS   — шаг физики (мс)
//
//  Режимы:
//    AUTO   — одна команда = один толчок (импульс)
//    MANUAL — команда удерживается = непрерывные толчки
//             throttle управляется пошагово up/down
//
//  Переключение: "manual" / "auto"
//
//  Команды движения:
//    up / down          — throttle ±шаг (в обоих режимах)
//    left / right       — roll
//    forward / back     — pitch
//    rotl / rotr        — yaw
//
//  MANUAL hold (удержание стика):
//    start_left / stop_left
//    start_right / stop_right
//    start_forward / stop_forward
//    start_back / stop_back
//    start_rotl / stop_rotl
//    start_rotr / stop_rotr
//
//  Управление:
//    arm / disarm  — вооружить/разоружить
//    manual / auto — сменить режим
//    stop          — сброс стиков и скоростей
//    status        — текущее состояние
//
//  НАСТРОЙКА:
//    1. Прошей slave_main.cpp на второй ESP32
//    2. Запусти slave, скопируй его MAC из Serial Monitor
//    3. Вставь MAC в SLAVE_MAC ниже
//    4. Прошей этот файл на первый ESP32
// ============================================================

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <esp_now.h>
#include <WiFi.h>
#include "common.h"

// ── MAC-адрес Slave ──────────────────────────────────────────
static uint8_t SLAVE_MAC[6] = { 0x30, 0x76, 0xF5, 0xF7, 0x4E, 0xA8 };
// ─────────────────────────────────────────────────────────────

BluetoothSerial SerialBT;

// ============================================================
//  Параметры физической модели
//  Все три параметра применяются каждые PHYSICS_DT_MS мс.
// ============================================================
constexpr float    IMPULSE_KICK   = 18.0f;  // сила единичного толчка (мкс/тик²)
constexpr float    DAMPING        = 0.88f;  // затухание скорости (0–1, выше = быстрее)
constexpr float    SPRING         = 0.045f; // сила возврата к центру (Spring coefficient)
constexpr uint32_t PHYSICS_DT_MS  = 15;     // шаг физики (мс)

// Максимальное отклонение стиков от центра (мкс)
constexpr float    MAX_STICK_DEV  = 400.0f;

// Throttle — пошаговое управление (физику не применяем)
constexpr uint16_t THR_STEP       = 50;     // шаг газа за команду up/down

// MANUAL: непрерывные толчки пока удерживается команда
constexpr float    HOLD_KICK      = 10.0f;  // толчок за тик при удержании

// ============================================================
//  Режим работы
// ============================================================
enum class Mode { AUTO, MANUAL };
static Mode currentMode = Mode::AUTO;

// ============================================================
//  Физическое состояние осей (roll / pitch / yaw)
//  position — текущее отклонение от центра (float, мкс)
//  velocity — скорость изменения (мкс/тик)
// ============================================================
struct Axis {
  float position = 0.0f;
  float velocity = 0.0f;

  // Добавить импульс
  void kick(float force) {
    velocity += force;
  }

  // Шаг физики: damping + spring → интеграция
  void step() {
    velocity *= DAMPING;                    // затухание
    velocity -= position * SPRING;         // пружина к центру
    position += velocity;
    // Ограничение диапазона
    if (position >  MAX_STICK_DEV) { position =  MAX_STICK_DEV; velocity = 0.0f; }
    if (position < -MAX_STICK_DEV) { position = -MAX_STICK_DEV; velocity = 0.0f; }
  }

  // Текущее значение канала
  uint16_t channelValue() const {
    int32_t val = CH_CENTER + static_cast<int32_t>(position);
    if (val < 1000) val = 1000;
    if (val > 2000) val = 2000;
    return static_cast<uint16_t>(val);
  }

  void reset() { position = 0.0f; velocity = 0.0f; }
};

static Axis axisRoll;   // CH0
static Axis axisPitch;  // CH1
static Axis axisYaw;    // CH3

// Throttle — отдельно, без физики
static uint16_t throttleValue = CH_THR_MIN;

// Армирование
static bool     isArmed  = false;
static uint16_t armValue = ARM_OFF_VALUE;

// Итоговые каналы для ESP-NOW
static SbusChannels channels;

// ============================================================
//  MANUAL режим — удерживаемые команды
// ============================================================
struct ManualState {
  bool rollLeft   = false;
  bool rollRight  = false;
  bool pitchFwd   = false;
  bool pitchBack  = false;
  bool yawLeft    = false;
  bool yawRight   = false;
};
static ManualState manualCmds;

// ============================================================
//  Физический цикл
// ============================================================
static unsigned long lastPhysicsTime = 0;

static void physicsStep() {
  if (millis() - lastPhysicsTime < PHYSICS_DT_MS) return;
  lastPhysicsTime = millis();

  // В MANUAL: применяем непрерывные толчки по удерживаемым командам
  if (currentMode == Mode::MANUAL) {
    if (manualCmds.rollLeft)   axisRoll.kick(-HOLD_KICK);
    if (manualCmds.rollRight)  axisRoll.kick( HOLD_KICK);
    if (manualCmds.pitchFwd)   axisPitch.kick( HOLD_KICK);
    if (manualCmds.pitchBack)  axisPitch.kick(-HOLD_KICK);
    if (manualCmds.yawLeft)    axisYaw.kick(-HOLD_KICK);
    if (manualCmds.yawRight)   axisYaw.kick( HOLD_KICK);
  }

  // Шаг физики для всех осей (в обоих режимах)
  axisRoll.step();
  axisPitch.step();
  axisYaw.step();

  // Обновляем каналы
  channels.ch[0] = axisRoll.channelValue();
  channels.ch[1] = axisPitch.channelValue();
  channels.ch[2] = throttleValue;
  channels.ch[3] = axisYaw.channelValue();
  channels.ch[ARM_CHANNEL] = armValue;
  for (int i = 5; i < 16; i++) channels.ch[i] = CH_CENTER;
}

// ============================================================
//  Инициализация
// ============================================================
static void setNeutral() {
  axisRoll.reset();
  axisPitch.reset();
  axisYaw.reset();
  throttleValue = CH_THR_MIN;
  channels.ch[0] = CH_CENTER;
  channels.ch[1] = CH_CENTER;
  channels.ch[2] = CH_THR_MIN;
  channels.ch[3] = CH_CENTER;
  channels.ch[ARM_CHANNEL] = ARM_OFF_VALUE;
  for (int i = 5; i < 16; i++) channels.ch[i] = CH_CENTER;
}

static uint16_t clampU16(int32_t v, uint16_t lo, uint16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return static_cast<uint16_t>(v);
}

// ============================================================
//  ESP-NOW callback
// ============================================================
static void onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS)
    Serial.println("[ESPNOW] Send FAILED");
}

// ============================================================
//  Ответ пользователю
// ============================================================
static void reply(const char* msg, bool fromBT) {
  Serial.println(msg);
  if (fromBT) SerialBT.println(msg);
}
static void replyStr(const String& msg, bool fromBT) {
  Serial.println(msg);
  if (fromBT) SerialBT.println(msg);
}

// ============================================================
//  Обработчик команд
// ============================================================
static void handleCommand(const String& rawCmd, bool fromBT) {
  String cmd = rawCmd;
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.isEmpty()) return;

  Serial.printf("[CMD][%s][%s] %s\n",
    currentMode == Mode::AUTO ? "AUTO" : "MANUAL",
    fromBT ? "BT" : "USB", cmd.c_str());

  // ── Режим ───────────────────────────────────────────────
  if (cmd == "manual") {
    currentMode = Mode::MANUAL;
    manualCmds  = ManualState{};
    reply("Mode: MANUAL (hold: start_forward/stop_forward etc, thr: up/down)", fromBT);
    return;
  }
  if (cmd == "auto") {
    currentMode = Mode::AUTO;
    manualCmds  = ManualState{};
    reply("Mode: AUTO (impulse kick per command)", fromBT);
    return;
  }

  // ── Армирование ─────────────────────────────────────────
  if (cmd == "arm")    { isArmed = true;  armValue = ARM_ON_VALUE;  reply("Armed!",    fromBT); return; }
  if (cmd == "disarm") { isArmed = false; armValue = ARM_OFF_VALUE; reply("Disarmed!", fromBT); return; }

  // ── Стоп ────────────────────────────────────────────────
  if (cmd == "stop") {
    manualCmds = ManualState{};
    axisRoll.reset();
    axisPitch.reset();
    axisYaw.reset();
    reply("Axes reset (throttle unchanged)", fromBT);
    return;
  }

  // ── Статус ──────────────────────────────────────────────
  if (cmd == "status") {
    char buf[160];
    snprintf(buf, sizeof(buf),
      "mode=%s armed=%d thr=%u\n"
      "roll pos=%.1f vel=%.2f ch=%u\n"
      "pitch pos=%.1f vel=%.2f ch=%u\n"
      "yaw  pos=%.1f vel=%.2f ch=%u",
      currentMode == Mode::AUTO ? "AUTO" : "MANUAL", isArmed, throttleValue,
      axisRoll.position,  axisRoll.velocity,  channels.ch[0],
      axisPitch.position, axisPitch.velocity, channels.ch[1],
      axisYaw.position,   axisYaw.velocity,   channels.ch[3]);
    replyStr(buf, fromBT);
    return;
  }

  // ── Throttle (оба режима) ────────────────────────────────
  if (cmd == "up") {
    throttleValue = clampU16(throttleValue + THR_STEP, CH_THR_MIN, CH_THR_MAX);
    char buf[24]; snprintf(buf, sizeof(buf), "thr -> %u", throttleValue);
    replyStr(buf, fromBT); return;
  }
  if (cmd == "down") {
    throttleValue = clampU16(static_cast<int32_t>(throttleValue) - THR_STEP,
                             CH_THR_MIN, CH_THR_MAX);
    char buf[24]; snprintf(buf, sizeof(buf), "thr -> %u", throttleValue);
    replyStr(buf, fromBT); return;
  }

  // ── Движение ─────────────────────────────────────────────
  if (currentMode == Mode::AUTO) {
    // Одиночный толчок — физика сама дотянет и вернёт
    if      (cmd == "left")    axisRoll.kick(-IMPULSE_KICK);
    else if (cmd == "right")   axisRoll.kick( IMPULSE_KICK);
    else if (cmd == "forward") axisPitch.kick( IMPULSE_KICK);
    else if (cmd == "back")    axisPitch.kick(-IMPULSE_KICK);
    else if (cmd == "rotl")    axisYaw.kick(-IMPULSE_KICK);
    else if (cmd == "rotr")    axisYaw.kick( IMPULSE_KICK);
    else replyStr("Unknown: " + cmd, fromBT);

  } else { // MANUAL — удержание через флаги
    if      (cmd == "start_left")    manualCmds.rollLeft   = true;
    else if (cmd == "stop_left")     manualCmds.rollLeft   = false;
    else if (cmd == "start_right")   manualCmds.rollRight  = true;
    else if (cmd == "stop_right")    manualCmds.rollRight  = false;
    else if (cmd == "start_forward") manualCmds.pitchFwd   = true;
    else if (cmd == "stop_forward")  manualCmds.pitchFwd   = false;
    else if (cmd == "start_back")    manualCmds.pitchBack  = true;
    else if (cmd == "stop_back")     manualCmds.pitchBack  = false;
    else if (cmd == "start_rotl")    manualCmds.yawLeft    = true;
    else if (cmd == "stop_rotl")     manualCmds.yawLeft    = false;
    else if (cmd == "start_rotr")    manualCmds.yawRight   = true;
    else if (cmd == "stop_rotr")     manualCmds.yawRight   = false;
    // Алиасы без префикса — один толчок (как в AUTO)
    else if (cmd == "left")    axisRoll.kick(-IMPULSE_KICK);
    else if (cmd == "right")   axisRoll.kick( IMPULSE_KICK);
    else if (cmd == "forward") axisPitch.kick( IMPULSE_KICK);
    else if (cmd == "back")    axisPitch.kick(-IMPULSE_KICK);
    else if (cmd == "rotl")    axisYaw.kick(-IMPULSE_KICK);
    else if (cmd == "rotr")    axisYaw.kick( IMPULSE_KICK);
    else replyStr("Unknown: " + cmd, fromBT);
  }
}

// ============================================================
//  Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== MASTER starting ===");

  setNeutral();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.print("Master MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] Init FAILED — halt");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, SLAVE_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESPNOW] Add peer FAILED — check SLAVE_MAC");
  } else {
    Serial.println("[ESPNOW] Peer added OK");
  }

  SerialBT.begin("Pavo360_Master");
  Serial.println("[BT] 'Pavo360_Master' ready");
  Serial.println("Physics: KICK=" + String(IMPULSE_KICK) +
                 " DAMPING=" + String(DAMPING) +
                 " SPRING=" + String(SPRING, 3));
  Serial.println("Commands: arm/disarm  auto/manual  stop  status");
  Serial.println("Motion:   up/down  left/right  forward/back  rotl/rotr");
  Serial.println("Hold:     start_forward/stop_forward  start_left/stop_left ...");
}

// ============================================================
//  Loop
// ============================================================
static String btBuf;
static String usbBuf;

void loop() {
  // ── BT ──────────────────────────────────────────────────
  while (SerialBT.available()) {
    char c = static_cast<char>(SerialBT.read());
    if (c == '\n' || c == '\r') {
      if (!btBuf.isEmpty()) { handleCommand(btBuf, true); btBuf.clear(); }
    } else { btBuf += c; }
  }

  // ── USB Serial Monitor ──────────────────────────────────
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (!usbBuf.isEmpty()) { handleCommand(usbBuf, false); usbBuf.clear(); }
    } else { usbBuf += c; }
  }

  // ── Физика ──────────────────────────────────────────────
  physicsStep();

  // ── DEBUG при арме (каждые 500 мс) ──────────────────────
  static unsigned long lastDbg = 0;
  if (isArmed && millis() - lastDbg >= 500) {
    Serial.printf("[DBG][%s] arm=%u thr=%u roll=%u(v=%.1f) pitch=%u(v=%.1f) yaw=%u(v=%.1f)\n",
      currentMode == Mode::AUTO ? "AUTO" : "MAN",
      channels.ch[ARM_CHANNEL], channels.ch[2],
      channels.ch[0], axisRoll.velocity,
      channels.ch[1], axisPitch.velocity,
      channels.ch[3], axisYaw.velocity);
    lastDbg = millis();
  }

  // ── ESP-NOW ~143 Hz ─────────────────────────────────────
  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 7) {
    esp_now_send(SLAVE_MAC,
                 reinterpret_cast<const uint8_t*>(&channels),
                 sizeof(channels));
    lastSend = millis();
  }
}