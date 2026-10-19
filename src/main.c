/**
 * ESP32-C3 + ESP-IDF 6.1.0
 * Modbus RTU Master → EasyDrive MINI-S
 * Управление через UART (Serial Monitor)
 *
 * Алгоритм Fail-Safe по ГОСТ Р МЭК 60204-1:
 *   1. Диагностика (3000H / 3001H)
 *   2. Предустановка 10.00 Гц
 *   3. Разрешение кнопки Пуск
 *   4. Старт + контроль разгона
 *   5. Рабочий режим + мониторинг RPM
 *
 * Команды UART:
 *   D / d  — Пуск (если разрешён)
 *   S / s  — Стоп
 *   R / r  — Сброс ошибки ПЧ
 *   H / h  — Справка
 *   + / -  — Изменить рабочую частоту (±1 Гц)
 *
 * Пины RS485 (3 шт):
 *   TX = GPIO7, RX = GPIO6, DE = GPIO5
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "WINDER";

// ==================== Hardware RS485 ====================
#define MB_UART_NUM        UART_NUM_1
#define MB_UART_TXD        7
#define MB_UART_RXD        6
#define MB_UART_RTS        5          // DE+/RE
#define MB_BAUD_RATE       19200      // P13.01 = 4 → 19200
#define MB_BUF_SIZE        256

// ==================== EasyDrive registers ====================
#define MB_SLAVE           1
#define REG_CMD            0x2000     // 8192  команды
#define REG_FREQ           0x2001     // 8193  задание частоты (x100)
#define REG_STATUS         0x3000     // 12288 состояние
#define REG_FAULT          0x3001     // 12289 ошибка
#define REG_OUT_FREQ       0x1001     // 4097  выходная частота
#define REG_RPM            0x1004     // 4100  обороты

#define CMD_START          0x0001     // СТАРТ вперёд
#define CMD_STOP           0x0005     // СТОП с замедлением
#define CMD_RESET          0x0008     // Сброс ошибки

#define SAFE_FREQ_X100     1000       // 10.00 Гц
#define DEFAULT_WORK_FREQ  5000       // 50.00 Гц

// ==================== State ====================
typedef enum {
    ST_DIAG = 0,      // диагностика, Пуск заблокирован
    ST_READY,         // готов, Пуск разрешён
    ST_STARTING,      // разгон
    ST_RUNNING,       // рабочий режим
    ST_FAULT          // авария
} state_t;

static state_t      g_state = ST_DIAG;
static uint16_t     g_work_freq = DEFAULT_WORK_FREQ;
static bool         g_start_allowed = false;
static uint16_t     g_last_status = 0;
static uint16_t     g_last_fault  = 0;
static uint16_t     g_last_rpm    = 0;

// ==================== CRC16 ====================
static uint16_t modbus_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc;
}

// ==================== UART RS485 init ====================
static esp_err_t mb_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = MB_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(MB_UART_NUM, MB_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MB_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(MB_UART_NUM, MB_UART_TXD, MB_UART_RXD, MB_UART_RTS, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(MB_UART_NUM, UART_MODE_RS485_HALF_DUPLEX));
    ESP_LOGI(TAG, "RS485: TX=%d RX=%d DE=%d @ %d 8N1", MB_UART_TXD, MB_UART_RXD, MB_UART_RTS, MB_BAUD_RATE);
    return ESP_OK;
}

// ==================== FC06 Write Single Register ====================
static esp_err_t mb_write(uint16_t reg, uint16_t value)
{
    uint8_t frame[8];
    frame[0] = MB_SLAVE;
    frame[1] = 0x06;
    frame[2] = (reg >> 8) & 0xFF;
    frame[3] = reg & 0xFF;
    frame[4] = (value >> 8) & 0xFF;
    frame[5] = value & 0xFF;
    uint16_t crc = modbus_crc16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;

    uart_flush_input(MB_UART_NUM);
    if (uart_write_bytes(MB_UART_NUM, frame, 8) != 8) return ESP_FAIL;
    uart_wait_tx_done(MB_UART_NUM, pdMS_TO_TICKS(50));

    uint8_t resp[16];
    int len = uart_read_bytes(MB_UART_NUM, resp, sizeof(resp), pdMS_TO_TICKS(200));
    if (len < 8) return ESP_ERR_TIMEOUT;

    if (resp[0] == MB_SLAVE && resp[1] == 0x06 &&
        resp[2] == frame[2] && resp[3] == frame[3] &&
        resp[4] == frame[4] && resp[5] == frame[5]) {
        return ESP_OK;
    }
    return ESP_FAIL;
}

// ==================== FC03 Read Holding Registers ====================
static esp_err_t mb_read(uint16_t reg, uint16_t *out)
{
    uint8_t frame[8];
    frame[0] = MB_SLAVE;
    frame[1] = 0x03;
    frame[2] = (reg >> 8) & 0xFF;
    frame[3] = reg & 0xFF;
    frame[4] = 0x00;
    frame[5] = 0x01;                  // 1 регистр
    uint16_t crc = modbus_crc16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;

    uart_flush_input(MB_UART_NUM);
    if (uart_write_bytes(MB_UART_NUM, frame, 8) != 8) return ESP_FAIL;
    uart_wait_tx_done(MB_UART_NUM, pdMS_TO_TICKS(50));

    uint8_t resp[16];
    int len = uart_read_bytes(MB_UART_NUM, resp, sizeof(resp), pdMS_TO_TICKS(200));
    // ответ: addr FC len dataH dataL CRCL CRCH  → минимум 7 байт
    if (len < 7) return ESP_ERR_TIMEOUT;
    if (resp[0] != MB_SLAVE || resp[1] != 0x03 || resp[2] != 2) return ESP_FAIL;

    *out = ((uint16_t)resp[3] << 8) | resp[4];
    return ESP_OK;
}

// ==================== Высокоуровневые команды ====================
static esp_err_t vfd_start(void)   { return mb_write(REG_CMD,  CMD_START); }
static esp_err_t vfd_stop(void)    { return mb_write(REG_CMD,  CMD_STOP);  }
static esp_err_t vfd_reset(void)   { return mb_write(REG_CMD,  CMD_RESET); }
static esp_err_t vfd_set_freq(uint16_t x100) { return mb_write(REG_FREQ, x100); }

// ==================== Справка ====================
static void print_help(void)
{
    printf("\n========== Команды UART ==========\n");
    printf("  D / d   Пуск (если разрешён)\n");
    printf("  S / s   Стоп\n");
    printf("  R / r   Сброс ошибки ПЧ\n");
    printf("  + / -   Рабочая частота ±1 Гц\n");
    printf("  H / h   Эта справка\n");
    printf("===================================\n");
    printf("Состояние: ");
    switch (g_state) {
        case ST_DIAG:     printf("ДИАГНОСТИКА (Пуск ЗАБЛОКИРОВАН)\n"); break;
        case ST_READY:    printf("ГОТОВ (Пуск разрешён)\n"); break;
        case ST_STARTING: printf("РАЗГОН\n"); break;
        case ST_RUNNING:  printf("РАБОТА  RPM=%u  f=%u.%02u Гц\n",
                                 g_last_rpm, g_work_freq/100, g_work_freq%100); break;
        case ST_FAULT:    printf("АВАРИЯ  fault=%u\n", g_last_fault); break;
    }
    printf("Рабочая частота: %u.%02u Гц\n\n", g_work_freq/100, g_work_freq%100);
}

// ==================== Обработка команд UART ====================
static void handle_uart_cmd(char c)
{
    switch (c) {
    case 'D': case 'd':
        if (!g_start_allowed) {
            printf("[!] Пуск ЗАБЛОКИРОВАН (нет готовности ПЧ)\n");
            return;
        }
        if (g_state == ST_RUNNING || g_state == ST_STARTING) {
            printf("[!] Уже в работе\n");
            return;
        }
        printf(">> ПУСК\n");
        if (vfd_set_freq(SAFE_FREQ_X100) != ESP_OK) {
            printf("[E] Не удалось записать стартовую частоту\n");
            return;
        }
        if (vfd_start() != ESP_OK) {
            printf("[E] Команда СТАРТ не прошла\n");
            g_state = ST_FAULT;
            return;
        }
        g_state = ST_STARTING;
        break;

    case 'S': case 's':
        printf(">> СТОП\n");
        vfd_stop();
        g_state = ST_DIAG;
        g_start_allowed = false;
        break;

    case 'R': case 'r':
        printf(">> Сброс ошибки\n");
        vfd_reset();
        g_state = ST_DIAG;
        g_start_allowed = false;
        break;

    case '+':
        if (g_work_freq < 6000) g_work_freq += 100;
        printf("Рабочая частота: %u.%02u Гц\n", g_work_freq/100, g_work_freq%100);
        if (g_state == ST_RUNNING) vfd_set_freq(g_work_freq);
        break;

    case '-':
        if (g_work_freq > 500) g_work_freq -= 100;
        printf("Рабочая частота: %u.%02u Гц\n", g_work_freq/100, g_work_freq%100);
        if (g_state == ST_RUNNING) vfd_set_freq(g_work_freq);
        break;

    case 'H': case 'h':
        print_help();
        break;

    default:
        break;
    }
}

// ==================== Задача диагностики / мониторинга ====================
static void control_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    while (1) {
        uint16_t status = 0, fault = 0, rpm = 0, outf = 0;
        bool ok_st = (mb_read(REG_STATUS, &status) == ESP_OK);
        bool ok_ft = (mb_read(REG_FAULT,  &fault)  == ESP_OK);

        if (!ok_st || !ok_ft) {
            // нет связи
            if (g_state != ST_FAULT) {
                printf("[!] Нет связи с ПЧ\n");
            }
            g_state = ST_FAULT;
            g_start_allowed = false;
            vTaskDelayUntil(&last, pdMS_TO_TICKS(200));
            continue;
        }

        g_last_status = status;
        g_last_fault  = fault;

        if (fault != 0) {
            if (g_state != ST_FAULT) {
                printf("[!] Авария ПЧ: код %u (Err%u)\n", fault, fault);
            }
            g_state = ST_FAULT;
            g_start_allowed = false;
            vTaskDelayUntil(&last, pdMS_TO_TICKS(200));
            continue;
        }

        // --- Шаг 1: диагностика ---
        if (g_state == ST_DIAG || g_state == ST_FAULT) {
            // ПЧ готов (status==1 остановлен/готов)
            if (status == 1) {
                // Шаг 2: предустановка безопасной частоты
                if (vfd_set_freq(SAFE_FREQ_X100) == ESP_OK) {
                    g_start_allowed = true;
                    g_state = ST_READY;
                    printf("[OK] ПЧ готов. Пуск РАЗРЕШЁН (стартовая 10.00 Гц)\n");
                }
            }
        }

        // --- Шаг 4: контроль разгона ---
        if (g_state == ST_STARTING) {
            mb_read(REG_OUT_FREQ, &outf);
            if (status == 2 && outf >= 900) {   // ~9 Гц достигнуто
                // Шаг 5: рабочая частота
                if (vfd_set_freq(g_work_freq) == ESP_OK) {
                    g_state = ST_RUNNING;
                    printf("[OK] Разгон завершён → рабочая %u.%02u Гц\n",
                           g_work_freq/100, g_work_freq%100);
                }
            }
        }

        // --- Шаг 5: мониторинг RPM ---
        if (g_state == ST_RUNNING) {
            if (mb_read(REG_RPM, &rpm) == ESP_OK) {
                g_last_rpm = rpm;
                // периодический вывод (примерно раз в 1 с)
                static int cnt = 0;
                if (++cnt >= 5) {
                    cnt = 0;
                    printf("  RPM=%u  status=%u\n", rpm, status);
                }
            }
            if (status != 2) {
                // неожиданная остановка
                printf("[!] ПЧ остановился\n");
                g_state = ST_DIAG;
                g_start_allowed = false;
            }
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(200));
    }
}

// ==================== app_main ====================
void app_main(void)
{
    printf("\n========================================\n");
    printf(" ESP32-C3  EasyDrive Modbus RTU Master\n");
    printf(" ESP-IDF 6.1  |  Fail-Safe алгоритм\n");
    printf("========================================\n");

    ESP_ERROR_CHECK(mb_init());
    vTaskDelay(pdMS_TO_TICKS(300));

    print_help();

    // Задача диагностики/мониторинга
    xTaskCreate(control_task, "ctrl", 4096, NULL, 5, NULL);

    // Основной цикл — чтение команд с USB-UART (консоль)
    uint8_t buf[16];
    while (1) {
        int n = uart_read_bytes(UART_NUM_0, buf, sizeof(buf), pdMS_TO_TICKS(50));
        for (int i = 0; i < n; i++) {
            handle_uart_cmd((char)buf[i]);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
