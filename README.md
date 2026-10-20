# ESP32 RC: Bluetooth → ESP-NOW → SBUS

Беспроводное управление дроном с двух плат ESP32. Команды приходят текстом по Bluetooth (или USB Serial) на **Master**, тот передаёт 16 каналов по ESP-NOW на **Slave**, а Slave выдаёт SBUS на полётный контроллер (FC).

```
Телефон / ПК ──BT Serial──▶ ESP32 Master ──ESP-NOW (~143 Гц)──▶ ESP32 Slave ──SBUS (GPIO13)──▶ FC
```

## Структура

| Папка | Назначение |
|---|---|
| [`ESP RC Master/`](ESP%20RC%20Master) | Master: BT-сервер `Pavo360_Master`, разбор команд, физическая модель стиков, отправка по ESP-NOW |
| [`Esp RC/`](Esp%20RC) | Slave: приём по ESP-NOW, упаковка в SBUS (25 байт, 100000 бод, 8E2, инвертированный) |

Оба проекта — PlatformIO (`espressif32`, `esp32dev`, Arduino). Файл `src/common.h` одинаковый в обоих: структура пакета `SbusChannels` и константы каналов. Если меняешь его — меняй в обоих проектах.

## Быстрый старт

1. Прошей **Slave** (`Esp RC`) и открой Serial Monitor (115200). Он напечатает `Slave MAC: XX:XX:XX:XX:XX:XX`.
2. Вставь этот MAC в `SLAVE_MAC` в `ESP RC Master/src/ESP RC Master.cpp`.
3. Прошей **Master** (`ESP RC Master`).
4. Подключи GPIO13 Slave к RX-пину UART на FC, в Betaflight выбери Serial RX → SBUS.
5. Подключись к `Pavo360_Master` по Bluetooth (любой BT-терминал) и отправляй команды.

```bash
cd "Esp RC"        && pio run -t upload && pio device monitor
cd "ESP RC Master" && pio run -t upload && pio device monitor
```

## Каналы

| Канал | Функция | Диапазон |
|---|---|---|
| CH0 | Roll | 1000–2000, центр 1500 |
| CH1 | Pitch | 1000–2000, центр 1500 |
| CH2 | Throttle | 900–2000 |
| CH3 | Yaw | 1000–2000, центр 1500 |
| CH4 | ARM | 1000 выкл / 1800 вкл |
| CH5–15 | — | 1500 |

## Команды Master

| Команда | Действие |
|---|---|
| `arm` / `disarm` | Включить / выключить моторы (CH4) |
| `auto` / `manual` | Сменить режим |
| `up` / `down` | Газ ±50 |
| `left` / `right` | Roll — толчок |
| `forward` / `back` | Pitch — толчок |
| `rotl` / `rotr` | Yaw — толчок |
| `start_<dir>` / `stop_<dir>` | В режиме MANUAL: удерживать направление (`left`, `right`, `forward`, `back`, `rotl`, `rotr`) |
| `stop` | Сбросить стики в центр (газ не меняется) |
| `status` | Вывести состояние осей и каналов |

### Режимы

- **AUTO** — каждая команда даёт один импульс, стик отклоняется и плавно возвращается в центр.
- **MANUAL** — пока направление удерживается (`start_*`), толчки идут каждый тик физики; `stop_*` отпускает.

### Физическая модель

Каждая ось (roll/pitch/yaw) — масса на пружине с затуханием, шаг каждые 15 мс. Параметры в начале `ESP RC Master.cpp`:

| Параметр | Значение | Смысл |
|---|---|---|
| `IMPULSE_KICK` | 18.0 | Сила одного толчка (AUTO) |
| `HOLD_KICK` | 10.0 | Толчок за тик при удержании (MANUAL) |
| `DAMPING` | 0.88 | Множитель скорости за тик |
| `SPRING` | 0.045 | Сила возврата к центру |
| `MAX_STICK_DEV` | 400 | Максимальное отклонение от центра, мкс |

## Безопасность

- Снимай пропеллеры при первых тестах.
- Slave не включает failsafe-флаг SBUS: если Master пропал, Slave продолжает слать последние принятые каналы и только пишет предупреждение в Serial. Настрой failsafe на FC или учитывай это.
