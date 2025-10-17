#include "uart_driver.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "UART";

#define UART2_RX_PIN CONFIG_UART_RX_PIN
#define UART2_TX_PIN CONFIG_UART_TX_PIN

int app_main(void){
    // Initialize UART2 with GPIO configuration
    uart_init(&uart2, 9600, UART2_RX_PIN, UART2_TX_PIN);
    
    ESP_LOGI(TAG, "Starting test...");
    
    uart_write_byte(&uart2, 42);
    ESP_LOGI(TAG, "Wrote byte 42");
    
    uint8_t received;
    if (uart_read_byte(&uart2, &received)) {
        ESP_LOGI(TAG, "Immediately read: %d", received);
    } else {
        ESP_LOGI(TAG, "No immediate data");
        
        // Wait up to 1 second
        for (int i = 0; i < 100; i++) {
            if (uart_read_byte(&uart2, &received)) {
                ESP_LOGI(TAG, "Read after %d ms: %d", i*10, received);
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    
    ESP_LOGI(TAG, "Test complete");
    
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    return 0;
}
