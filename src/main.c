/**
 * ESP32-C3 + ESP-IDF 6.1.0
 * Modbus RTU Master → EasyDrive MINI-S
 * Управление через UART + программный Wobble (Traverse)
 *
 * Команды UART:
 *   D/d  Пуск    S/s  Стоп    R/r  Сброс ошибки
 *   +/-  Базовая частота ±1 Гц
 *   W/w  Вкл/выкл качающейся частоты (Wobble)
 *   [ / ]  Амплитуда Wobble ±0.5 Гц
 *   { / }  Период Wobble ±1 с
 *   H/h  Справка
 *
 * Пины RS485: TX=7 RX=6 DE=5 @ 19200 8N1
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
#include "esp_timer.h"

static const char *TAG = "WINDER";

#define MB_UART_NUM        UART_NUM_1
#define MB_UART_TXD        7
#define MB_UART_RXD        6
#define MB_UART_RTS        5
#define MB_BAUD_RATE       19200
#define MB_BUF_SIZE        256

#define MB_SLAVE           1
#define REG_CMD            0x2000
#define REG_FREQ           0x2001
#define REG_STATUS         0x3000
#define REG_FAULT          0x3001
#define REG_OUT_FREQ       0x1001
#define REG_RPM            0x1004

#define CMD_START          0x0001
#define CMD_STOP           0x0005
#define CMD_RESET          0x0008

#define SAFE_FREQ_X100     1000
#define DEFAULT_WORK_FREQ  4000       // 40.00 Гц — типичная база для намотки

typedef enum {
    ST_DIAG = 0,
    ST_READY,
    ST_STARTING,
    ST_RUNNING,
    ST_FAULT
} state_t;

static state_t  g_state = ST_DIAG;
static uint16_t g_work_freq = DEFAULT_WORK_FREQ;  // центр, x100
static bool     g_start_allowed = false;
static uint16_t g_last_status = 0;
static uint16_t g_last_fault  = 0;
static uint16_t g_last_rpm    = 0;

// ---- Wobble (программный Traverse) ----
static bool     g_wobble_on   = false;
static uint16_t g_wobble_amp  = 200;    // ±2.00 Гц (x100)
static uint32_t g_wobble_period_ms = 10000;  // 10 с полный цикл
static int64_t  g_wobble_t0   = 0;

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
    ESP_LOGI(TAG, "RS485 TX=%d RX=%d DE=%d @ %d", MB_UART_TXD, MB_UART_RXD, MB_UART_RTS, MB_BAUD_RATE);
    return ESP_OK;
}

static esp_err_t mb_write(uint16_t reg, uint16_t value)
{
    uint8_t frame[8];
    frame[0] = MB_SLAVE; frame[1] = 0x06;
    frame[2] = (reg >> 8) & 0xFF; frame[3] = reg & 0xFF;
    frame[4] = (value >> 8) & 0xFF; frame[5] = value & 0xFF;
    uint16_t crc = modbus_crc16(frame, 6);
    frame[6] = crc & 0xFF; frame[7] = (crc >> 8) & 0xFF;

    uart_flush_input(MB_UART_NUM);
    if (uart_write_bytes(MB_UART_NUM, frame, 8) != 8) return ESP_FAIL;
    uart_wait_tx_done(MB_UART_NUM, pdMS_TO_TICKS(50));

    uint8_t resp[16];
    int len = uart_read_bytes(MB_UART_NUM, resp, sizeof(resp), pdMS_TO_TICKS(200));
    if (len < 8) return ESP_ERR_TIMEOUT;
    if (resp[0] == MB_SLAVE && resp[1] == 0x06 &&
        resp[2] == frame[2] && resp[3] == frame[3] &&
        resp[4] == frame[4] && resp[5] == frame[5]) return ESP_OK;
    return ESP_FAIL;
}

static esp_err_t mb_read(uint16_t reg, uint16_t *out)
{
    uint8_t frame[8];
    frame[0] = MB_SLAVE; frame[1] = 0x03;
    frame[2] = (reg >> 8) & 0xFF; frame[3] = reg & 0xFF;
    frame[4] = 0x00; frame[5] = 0x01;
    uint16_t crc = modbus_crc16(frame, 6);
    frame[6] = crc & 0xFF; frame[7] = (crc >> 8) & 0xFF;

    uart_flush_input(MB_UART_NUM);
    if (uart_write_bytes(MB_UART_NUM, frame, 8) != 8) return ESP_FAIL;
    uart_wait_tx_done(MB_UART_NUM, pdMS_TO_TICKS(50));

    uint8_t resp[16];
    int len = uart_read_bytes(MB_UART_NUM, resp, sizeof(resp), pdMS_TO_TICKS(200));
    if (len < 7) return ESP_ERR_TIMEOUT;
    if (resp[0] != MB_SLAVE || resp[1] != 0x03 || resp[2] != 2) return ESP_FAIL;
    *out = ((uint16_t)resp[3] << 8) | resp[4];
    return ESP_OK;
}

static esp_err_t vfd_start(void) { return mb_write(REG_CMD, CMD_START); }
static esp_err_t vfd_stop(void)  { return mb_write(REG_CMD, CMD_STOP); }
static esp_err_t vfd_reset(void) { return mb_write(REG_CMD, CMD_RESET); }
static esp_err_t vfd_set_freq(uint16_t x100) { return mb_write(REG_FREQ, x100); }

/** Треугольный Wobble вокруг g_work_freq */
static uint16_t wobble_freq_now(void)
{
    if (!g_wobble_on || g_wobble_period_ms == 0) return g_work_freq;

    int64_t elapsed = (esp_timer_get_time() / 1000) - g_wobble_t0;
    if (elapsed < 0) elapsed = 0;
    uint32_t phase_ms = (uint32_t)(elapsed % g_wobble_period_ms);
    float phase = (float)phase_ms / (float)g_wobble_period_ms;  // 0…1

    float amp = (float)g_wobble_amp;
    float center = (float)g_work_freq;
    float f;
    if (phase < 0.5f)
        f = center - amp + 4.0f * amp * phase;           // вверх
    else
        f = center + amp - 4.0f * amp * (phase - 0.5f); // вниз

    if (f < 100.0f) f = 100.0f;   // минимум 1 Гц
    if (f > 6000.0f) f = 6000.0f;
    return (uint16_t)(f + 0.5f);
}

static void print_help(void)
{
    printf("\n========== Команды UART ==========\n");
    printf("  D/d     Пуск (если разрешён)\n");
    printf("  S/s     Стоп\n");
    printf("  R/r     Сброс ошибки ПЧ\n");
    printf("  +/-     Базовая частота ±1 Гц\n");
    printf("  W/w     Wobble ВКЛ/ВЫКЛ (качание)\n");
    printf("  [ / ]   Амплитуда Wobble ±0.5 Гц\n");
    printf("  { / }   Период Wobble ±1 с\n");
    printf("  H/h     Справка\n");
    printf("===================================\n");
    printf("Состояние: ");
    switch (g_state) {
        case ST_DIAG:     printf("ДИАГНОСТИКА\n"); break;
        case ST_READY:    printf("ГОТОВ (Пуск разрешён)\n"); break;
        case ST_STARTING: printf("РАЗГОН\n"); break;
        case ST_RUNNING:  printf("РАБОТА  RPM=%u\n", g_last_rpm); break;
        case ST_FAULT:    printf("АВАРИЯ fault=%u\n", g_last_fault); break;
    }
    printf("База: %u.%02u Гц | Wobble: %s  ±%u.%02u Гц  T=%lu с\n\n",
           g_work_freq/100, g_work_freq%100,
           g_wobble_on ? "ВКЛ" : "ВЫКЛ",
           g_wobble_amp/100, g_wobble_amp%100,
           (unsigned long)(g_wobble_period_ms / 1000));
}

static void handle_uart_cmd(char c)
{
    switch (c) {
    case 'D': case 'd':
        if (!g_start_allowed) { printf("[!] Пуск ЗАБЛОКИРОВАН\n"); return; }
        if (g_state == ST_RUNNING || g_state == ST_STARTING) { printf("[!] Уже в работе\n"); return; }
        printf(">> ПУСК\n");
        if (vfd_set_freq(SAFE_FREQ_X100) != ESP_OK) { printf("[E] частота\n"); return; }
        if (vfd_start() != ESP_OK) { printf("[E] СТАРТ\n"); g_state = ST_FAULT; return; }
        g_state = ST_STARTING;
        break;

    case 'S': case 's':
        printf(">> СТОП\n");
        vfd_stop();
        g_wobble_on = false;
        g_state = ST_DIAG;
        g_start_allowed = false;
        break;

    case 'R': case 'r':
        printf(">> Сброс\n");
        vfd_reset();
        g_wobble_on = false;
        g_state = ST_DIAG;
        g_start_allowed = false;
        break;

    case '+':
        if (g_work_freq < 6000) g_work_freq += 100;
        printf("База: %u.%02u Гц\n", g_work_freq/100, g_work_freq%100);
        if (g_state == ST_RUNNING && !g_wobble_on) vfd_set_freq(g_work_freq);
        break;

    case '-':
        if (g_work_freq > 500) g_work_freq -= 100;
        printf("База: %u.%02u Гц\n", g_work_freq/100, g_work_freq%100);
        if (g_state == ST_RUNNING && !g_wobble_on) vfd_set_freq(g_work_freq);
        break;

    case 'W': case 'w':
        g_wobble_on = !g_wobble_on;
        if (g_wobble_on) {
            g_wobble_t0 = esp_timer_get_time() / 1000;
            printf(">> Wobble ВКЛ  ±%u.%02u Гц  период %lu с\n",
                   g_wobble_amp/100, g_wobble_amp%100,
                   (unsigned long)(g_wobble_period_ms/1000));
        } else {
            printf(">> Wobble ВЫКЛ → фиксированная база\n");
            if (g_state == ST_RUNNING) vfd_set_freq(g_work_freq);
        }
        break;

    case '[':
        if (g_wobble_amp >= 50) g_wobble_amp -= 50;
        printf("Амплитуда: ±%u.%02u Гц\n", g_wobble_amp/100, g_wobble_amp%100);
        break;

    case ']':
        if (g_wobble_amp < 1000) g_wobble_amp += 50;
        printf("Амплитуда: ±%u.%02u Гц\n", g_wobble_amp/100, g_wobble_amp%100);
        break;

    case '{':
        if (g_wobble_period_ms > 2000) g_wobble_period_ms -= 1000;
        printf("Период: %lu с\n", (unsigned long)(g_wobble_period_ms/1000));
        break;

    case '}':
        if (g_wobble_period_ms < 60000) g_wobble_period_ms += 1000;
        printf("Период: %lu с\n", (unsigned long)(g_wobble_period_ms/1000));
        break;

    case 'H': case 'h':
        print_help();
        break;

    default:
        break;
    }
}

static void control_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    while (1) {
        uint16_t status = 0, fault = 0, rpm = 0, outf = 0;
        bool ok_st = (mb_read(REG_STATUS, &status) == ESP_OK);
        bool ok_ft = (mb_read(REG_FAULT,  &fault)  == ESP_OK);

        if (!ok_st || !ok_ft) {
            if (g_state != ST_FAULT) printf("[!] Нет связи\n");
            g_state = ST_FAULT;
            g_start_allowed = false;
            g_wobble_on = false;
            vTaskDelayUntil(&last, pdMS_TO_TICKS(200));
            continue;
        }

        g_last_status = status;
        g_last_fault  = fault;

        if (fault != 0) {
            if (g_state != ST_FAULT) printf("[!] Авария Err%u\n", fault);
            g_state = ST_FAULT;
            g_start_allowed = false;
            g_wobble_on = false;
            vTaskDelayUntil(&last, pdMS_TO_TICKS(200));
            continue;
        }

        if (g_state == ST_DIAG || g_state == ST_FAULT) {
            if (status == 1) {
                if (vfd_set_freq(SAFE_FREQ_X100) == ESP_OK) {
                    g_start_allowed = true;
                    g_state = ST_READY;
                    printf("[OK] ПЧ готов. Пуск РАЗРЕШЁН\n");
                }
            }
        }

        if (g_state == ST_STARTING) {
            mb_read(REG_OUT_FREQ, &outf);
            if (status == 2 && outf >= 900) {
                if (vfd_set_freq(g_work_freq) == ESP_OK) {
                    g_state = ST_RUNNING;
                    g_wobble_t0 = esp_timer_get_time() / 1000;
                    printf("[OK] Рабочая база %u.%02u Гц%s\n",
                           g_work_freq/100, g_work_freq%100,
                           g_wobble_on ? " + Wobble" : "");
                }
            }
        }

        if (g_state == ST_RUNNING) {
            // Программный Wobble: обновляем задание частоты
            if (g_wobble_on) {
                uint16_t f = wobble_freq_now();
                vfd_set_freq(f);
            }

            if (mb_read(REG_RPM, &rpm) == ESP_OK) {
                g_last_rpm = rpm;
                static int cnt = 0;
                if (++cnt >= 5) {
                    cnt = 0;
                    uint16_t of = 0;
                    mb_read(REG_OUT_FREQ, &of);
                    printf("  RPM=%u  f_out=%u.%02u  wobble=%s\n",
                           rpm, of/100, of%100, g_wobble_on ? "ON" : "off");
                }
            }
            if (status != 2) {
                printf("[!] ПЧ остановился\n");
                g_state = ST_DIAG;
                g_start_allowed = false;
                g_wobble_on = false;
            }
        }

        // Wobble обновляем чаще в RUNNING — цикл 100 мс
        vTaskDelayUntil(&last, pdMS_TO_TICKS(g_state == ST_RUNNING && g_wobble_on ? 100 : 200));
    }
}

void app_main(void)
{
    printf("\n========================================\n");
    printf(" ESP32-C3  EasyDrive + Wobble/Traverse\n");
    printf(" ESP-IDF 6.1  |  Fail-Safe\n");
    printf("========================================\n");

    ESP_ERROR_CHECK(mb_init());
    vTaskDelay(pdMS_TO_TICKS(300));
    print_help();

    xTaskCreate(control_task, "ctrl", 4096, NULL, 5, NULL);

    uint8_t buf[16];
    while (1) {
        int n = uart_read_bytes(UART_NUM_0, buf, sizeof(buf), pdMS_TO_TICKS(50));
        for (int i = 0; i < n; i++) handle_uart_cmd((char)buf[i]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
