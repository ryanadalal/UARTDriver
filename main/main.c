#include "uart_driver.h"

int main(void){
    uart_init(9600);

    while(1){
        uart_write_string("Hello UART\n");
        uint8_t c = uart_read_byte(); // loopback testing
        uart_write_byte(c);           // echo back
    }

    return 0;
}
