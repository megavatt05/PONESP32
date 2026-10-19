# ESP32-C3 Modbus RTU Master (ESP-IDF 6.1.0)

Управление двигателем по **RS485 / Modbus RTU** на **ESP32-C3**.

Используются **3 пина**: TX, RX и DE (направление).

## Логика работы

После запуска прошивка выполняет **4 цикла**:

1. Команда **ПУСК** (Holding Register `8192` = `18`)
2. Работа **5 секунд**
3. Команда **СТОП** (Holding Register `8192` = `1`)
4. Пауза 1 секунда → следующий цикл

После 4-го цикла программа завершает работу и уходит в сон.

## Аппаратная часть (3 пина)

| Сигнал | GPIO ESP32-C3 | Модуль RS485 (MAX485 / XY-017) |
|--------|---------------|--------------------------------|
| TX     | **GPIO7**     | DI                             |
| RX     | **GPIO6**     | RO                             |
| DE     | **GPIO5**     | DE + /RE (соединены вместе)    |
| GND    | GND           | GND                            |
| 3.3V/5V| 3V3 / 5V      | VCC                            |

> DE и /RE на модуле обычно соединяют вместе и подключают к одному GPIO.
> ESP-IDF в режиме `UART_MODE_RS485_HALF_DUPLEX` сам управляет этим пином.

## Параметры Modbus

| Параметр           | Значение      |
|--------------------|---------------|
| Скорость           | 9600 бод      |
| Формат             | 8N1           |
| Адрес slave        | 1             |
| Регистр команды    | 8192 (0x2000) |
| Значение ПУСК      | 18            |
| Значение СТОП      | 1             |
| Регистр разгона    | 8193 (0x2001) |
| Скорость разгона   | 1000          |

## Сборка и прошивка (PlatformIO)

```bash
# Клонировать ветку
git clone -b esp32c3-modbus-rtu https://github.com/megavatt05/PONESP32.git
cd PONESP32

# Сборка
pio run

# Прошивка
pio run -t upload

# Монитор
pio device monitor
```

Или через VS Code + PlatformIO: выбрать окружение `esp32-c3` → Build / Upload.

## Изменение пинов / параметров

В файле `src/main.c` в начале:

```c
#define MB_UART_TXD        7
#define MB_UART_RXD        6
#define MB_UART_RTS        5          // DE

#define CYCLE_COUNT        4
#define WORK_TIME_MS       5000
#define PAUSE_BETWEEN_MS   1000
```

## Пример лога

```
I (xxx) MB_MASTER: ========================================
I (xxx) MB_MASTER:  ESP32-C3 Modbus RTU Master (IDF 6.1)
I (xxx) MB_MASTER:  4 цикла × 5 сек работы
I (xxx) MB_MASTER: ========================================
I (xxx) MB_MASTER: UART1 RS485 init: TX=7 RX=6 DE=5 @ 9600 baud
I (xxx) MB_MASTER: ──────── Цикл 1 / 4 ────────
I (xxx) MB_MASTER: >>> ПУСК двигателя
I (xxx) MB_MASTER: OK: slave=1 reg=8192 value=18
I (xxx) MB_MASTER: Работа 5000 мс...
I (xxx) MB_MASTER: >>> СТОП двигателя
I (xxx) MB_MASTER: OK: slave=1 reg=8192 value=1
...
I (xxx) MB_MASTER: Все 4 циклов завершены. Конец.
```

## Лицензия

MIT — см. [LICENSE](LICENSE).

---

**Автор:** [@megavatt05](https://github.com/megavatt05)
