#include <stdint.h>

#define UART_NUM 1
#define UART_TX_PIN 17
#define UART_RX_PIN 16
#define UART_BAUD 9600

void uart_init(int baud_rate);
void uart_write_byte(uint8_t byte);
uint8_t uart_read_byte(void);
void uart_write_string(const char* str);