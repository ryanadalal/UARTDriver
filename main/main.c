/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"

#define UART_TX_PIN CONFIG_UART_TX_PIN
#define UART_RX_PIN CONFIG_UART_RX_PIN
#define UART_PORT_NUM UART_NUM_1
#define UART_BAUD_RATE 9600
#define UART_BUFFER_SIZE 1024

static void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    uart_param_config(UART_PORT_NUM, &uart_config);

    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_driver_install(UART_PORT_NUM, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0);

    ESP_LOGI("UART_INIT", "UART initialized with TX pin %d and RX pin %d", UART_TX_PIN, UART_RX_PIN);
}

void app_main(void) {
    uart_init();

    uint8_t buf[128];
    while(1){
        // send
        uart_write_bytes(UART_PORT_NUM, "Hello UART\n", 11);
        // receive
        int len = uart_read_bytes(UART_PORT_NUM, buf, sizeof(buf), pdMS_TO_TICKS(500));
        if(len > 0){
            ESP_LOGI("UART", "Received data: %.*s", len, buf);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}