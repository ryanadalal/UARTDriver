#include "uart_driver.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

int app_main(void){
    // Initialize UART2 with GPIO configuration
    uart_init(&uart2, 9600);

    //uart_init(&uart0, 9600);

    while(1){
        //uart_write_string(&uart1, "Hello UART\n");
        uart_write_byte(&uart2, 4);
        uint8_t c;
        while (!uart_read_byte(&uart2, &c)) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        printf("Hello UART\n %d", c);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return 0;
}
