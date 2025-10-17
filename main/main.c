#include "uart_driver.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

int app_main(void){
    // Initialize UART2 with GPIO configuration
    uart_init(&uart2, 9600, 16, 17);
    uint8_t count = 0;
    while(1){
        uart_write_byte(&uart2, count);
        count++;
        uint8_t c;
        while (!uart_read_byte(&uart2, &c)) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        printf("%d\n", c);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    return 0;
}
