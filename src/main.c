/**
 * ESP32-C3 + ESP-IDF 6.1.0
 * Modbus RTU Master (RS485, 3 pins: TX / RX / DE)
 *
 * Логика:
 *   4 раза подряд:
 *     1. Команда ПУСК  (holding reg 8192 = 18)
 *     2. Работа 5 секунд
 *     3. Команда СТОП  (holding reg 8192 = 1)
 *
 * Пины (можно изменить в CONFIG):
 *   TX  = GPIO7
 *   RX  = GPIO6
 *   DE  = GPIO5   (DE + /RE tied together)
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "MB_MASTER";

// ==================== Hardware (3 pins) ====================
#define MB_UART_NUM        UART_NUM_1
#define MB_UART_TXD        7          // DI  → MAX485 DI
#define MB_UART_RXD        6          // RO  → MAX485 RO
#define MB_UART_RTS        5          // DE+/RE → MAX485 DE+RE
#define MB_BAUD_RATE       9600
#define MB_BUF_SIZE        256

// ==================== Modbus ====================
#define MB_SLAVE_ADDR      1
#define MB_REG_CMD         8192       // 0x2000  — команда ПУСК/СТОП
#define MB_CMD_START       18
#define MB_CMD_STOP        1
#define MB_REG_ACCEL       8193       // 0x2001  — скорость разгона (опционально)
#define MB_ACCEL_VALUE     1000

#define CYCLE_COUNT        4
#define WORK_TIME_MS       5000       // 5 секунд работы
#define PAUSE_BETWEEN_MS   1000       // пауза между циклами

// ==================== CRC16 Modbus ====================
static uint16_t modbus_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// ==================== UART + RS485 init ====================
static esp_err_t mb_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = MB_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(MB_UART_NUM, MB_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MB_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(MB_UART_NUM, MB_UART_TXD, MB_UART_RXD, MB_UART_RTS, UART_PIN_NO_CHANGE));

    // RS485 half-duplex: RTS автоматически управляет DE
    ESP_ERROR_CHECK(uart_set_mode(MB_UART_NUM, UART_MODE_RS485_HALF_DUPLEX));

    ESP_LOGI(TAG, "UART%d RS485 init: TX=%d RX=%d DE=%d @ %d baud", 
             MB_UART_NUM, MB_UART_TXD, MB_UART_RXD, MB_UART_RTS, MB_BAUD_RATE);
    return ESP_OK;
}

// ==================== Write Single Holding Register (FC06) ====================
static esp_err_t mb_write_holding_reg(uint8_t slave, uint16_t reg, uint16_t value)
{
    uint8_t frame[8];
    frame[0] = slave;
    frame[1] = 0x06;                    // FC06
    frame[2] = (reg >> 8) & 0xFF;
    frame[3] = reg & 0xFF;
    frame[4] = (value >> 8) & 0xFF;
    frame[5] = value & 0xFF;

    uint16_t crc = modbus_crc16(frame, 6);
    frame[6] = crc & 0xFF;              // CRC low
    frame[7] = (crc >> 8) & 0xFF;       // CRC high

    // Очистка буфера RX
    uart_flush_input(MB_UART_NUM);

    int written = uart_write_bytes(MB_UART_NUM, frame, 8);
    if (written != 8) {
        ESP_LOGE(TAG, "uart_write failed: %d", written);
        return ESP_FAIL;
    }
    uart_wait_tx_done(MB_UART_NUM, pdMS_TO_TICKS(100));

    // Ждём ответ (эхо FC06 = те же 8 байт)
    uint8_t resp[16];
    int len = uart_read_bytes(MB_UART_NUM, resp, sizeof(resp), pdMS_TO_TICKS(300));

    if (len < 8) {
        ESP_LOGW(TAG, "Нет ответа от slave %d (получено %d байт)", slave, len);
        return ESP_ERR_TIMEOUT;
    }

    // Простая проверка: адрес + FC + регистр + значение
    if (resp[0] == slave && resp[1] == 0x06 &&
        resp[2] == frame[2] && resp[3] == frame[3] &&
        resp[4] == frame[4] && resp[5] == frame[5]) {
        ESP_LOGI(TAG, "OK: slave=%d reg=%u value=%u", slave, reg, value);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Ответ не совпадает (len=%d)", len);
    return ESP_FAIL;
}

// ==================== Команды ====================
static esp_err_t motor_start(void)
{
    ESP_LOGI(TAG, ">>> ПУСК двигателя");
    esp_err_t err = mb_write_holding_reg(MB_SLAVE_ADDR, MB_REG_CMD, MB_CMD_START);
    if (err == ESP_OK) {
        // Опционально: скорость разгона
        mb_write_holding_reg(MB_SLAVE_ADDR, MB_REG_ACCEL, MB_ACCEL_VALUE);
    }
    return err;
}

static esp_err_t motor_stop(void)
{
    ESP_LOGI(TAG, ">>> СТОП двигателя");
    return mb_write_holding_reg(MB_SLAVE_ADDR, MB_REG_CMD, MB_CMD_STOP);
}

// ==================== app_main ====================
void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " ESP32-C3 Modbus RTU Master (IDF 6.1)");
    ESP_LOGI(TAG, " 4 цикла × 5 сек работы");
    ESP_LOGI(TAG, "========================================");

    ESP_ERROR_CHECK(mb_uart_init());

    vTaskDelay(pdMS_TO_TICKS(500));   // дать шине стабилизироваться

    for (int cycle = 1; cycle <= CYCLE_COUNT; cycle++) {
        ESP_LOGI(TAG, "──────── Цикл %d / %d ────────", cycle, CYCLE_COUNT);

        if (motor_start() != ESP_OK) {
            ESP_LOGE(TAG, "Не удалось запустить двигатель");
        }

        ESP_LOGI(TAG, "Работа %d мс...", WORK_TIME_MS);
        vTaskDelay(pdMS_TO_TICKS(WORK_TIME_MS));

        if (motor_stop() != ESP_OK) {
            ESP_LOGE(TAG, "Не удалось остановить двигатель");
        }

        if (cycle < CYCLE_COUNT) {
            ESP_LOGI(TAG, "Пауза %d мс перед следующим циклом", PAUSE_BETWEEN_MS);
            vTaskDelay(pdMS_TO_TICKS(PAUSE_BETWEEN_MS));
        }
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " Все %d циклов завершены. Конец.", CYCLE_COUNT);
    ESP_LOGI(TAG, "========================================");

    // Дальше просто спим
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
