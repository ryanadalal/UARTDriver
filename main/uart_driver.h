#include <stdint.h>

#define UART_TX_PIN 17
#define UART_RX_PIN 16

typedef struct {
    volatile uint32_t *fifo_reg;
    volatile uint32_t *status_reg;
    volatile uint32_t *conf0_reg;
    volatile uint32_t *conf1_reg;
    volatile uint32_t *clkdiv_reg;
} uart_t;

extern uart_t uart0;
extern uart_t uart1;
extern uart_t uart2;

void uart_init(uart_t* uart_num, int baud_rate, uint8_t rx_gpio_num, uint8_t tx_gpio_num);
void uart_write_byte(uart_t* uart_num, uint8_t byte);
uint8_t uart_read_byte(uart_t* uart_num, uint8_t* out_byte);
void uart_write_string(uart_t* uart_num, const char* str);
void configure_gpio_for_uart2(uint8_t rx_gpio_num, uint8_t tx_gpio_num);