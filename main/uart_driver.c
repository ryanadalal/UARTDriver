#include "uart_driver.h"

#define UART1_BASE       0x3FF50000
#define UART_CONF0_REG   (UART1_BASE + 0x20)
#define UART_CONF1_REG   (UART1_BASE + 0x24)
#define UART_CLKDIV_REG   (UART1_BASE + 0x14)
#define UART_FIFO_REG    (UART1_BASE + 0x0)
#define UART_STATUS_REG  (UART1_BASE + 0x1C)

#define DPORT_PERIP_CLK_EN_REG (0x3FF000C0)
#define DPORT_PERIP_RST_EN_REG (0x3FF000C4)

// conf 0 register
#define UART_BIT_NUM (0x3 << 2)
#define UART_PARITY_EN (0x0 << 1)

// conf 1 register
#define UART_RX_FLOW_EN (0x0 << 23)
#define UART_TXFIFO_EMPTY_THRHD (0x10 << 8) // trigger refill of TX FIFO when <= 16 bytes
#define UART_RXFIFO_FULL_THRHD (0x10 << 0) // trigger read of RX FIFO when >= 16 bytes
#define UART_RX_TOUT_EN (0x1 << 31)
#define UART_RX_TOUT_THRHD (0X8 << 24) // force read RX FIFO when no data for 8 bit times

// read write counts
#define UART_RXFIFO_CNT (0xFF << 0)
#define UART_TXFIFO_CNT (0xFF << 16)

// peripheral enable and reset
#define DPORT_UART_MEM_CLOCK_EN (1 << 24)
#define DPORT_UART1_CLK_EN (1 << 5)
#define DPORT_UART1_RST_HOLD (1 << 5)

// max size of buffers
#define UART_FIFO_LEN 128


void uart_init(int baud_rate){
    // enable UART1 peripheral clock
    volatile uint32_t *clk_en_reg = (volatile uint32_t *)DPORT_PERIP_CLK_EN_REG;
    uint32_t clk_en_val = *clk_en_reg;
    clk_en_val &= ~(DPORT_UART1_CLK_EN | DPORT_UART_MEM_CLOCK_EN);
    clk_en_val |= DPORT_UART1_CLK_EN | DPORT_UART_MEM_CLOCK_EN;
    *clk_en_reg = clk_en_val;
    // reset UART1 peripheral
    volatile uint32_t *rst_reg = (volatile uint32_t *)DPORT_PERIP_RST_EN_REG;
    *rst_reg |= DPORT_UART1_RST_HOLD;
    *rst_reg &= ~DPORT_UART1_RST_HOLD;


    volatile uint32_t *conf_0_reg = (volatile uint32_t *)UART_CONF0_REG;
    uint32_t conf_0_val = *conf_0_reg;
    conf_0_val &= ~(UART_BIT_NUM | UART_PARITY_EN);
    conf_0_val |= UART_BIT_NUM | UART_PARITY_EN;
    *conf_0_reg = conf_0_val;

    uint32_t clk_div = (80000000 / (baud_rate * 16));
    volatile uint32_t *clk_div_reg = (volatile uint32_t *)UART_CLKDIV_REG;
    *clk_div_reg = clk_div;

    volatile uint32_t *conf_1_reg = (volatile uint32_t *)UART_CONF1_REG;
    uint32_t conf_1_val = *conf_1_reg;
    conf_1_val &= ~(UART_RX_FLOW_EN | UART_TXFIFO_EMPTY_THRHD | UART_RXFIFO_FULL_THRHD | UART_RX_TOUT_EN | UART_RX_TOUT_THRHD);
    conf_1_val |= UART_RX_FLOW_EN | UART_TXFIFO_EMPTY_THRHD | UART_RXFIFO_FULL_THRHD | UART_RX_TOUT_EN | UART_RX_TOUT_THRHD;
    *conf_1_reg = conf_1_val;
}

void uart_write_byte(uint8_t byte){
    volatile uint32_t *status = (volatile uint32_t *)UART_STATUS_REG;
    volatile uint32_t *fifo = (volatile uint32_t *)UART_FIFO_REG;

    // wait until there is space in the TX FIFO
    while ( (((*status) & UART_TXFIFO_CNT) >> 16) >= UART_FIFO_LEN) {
        ;
    }

    *fifo = byte;
}

uint8_t uart_read_byte(void){
    volatile uint32_t *status = (volatile uint32_t *)UART_STATUS_REG;
    volatile uint32_t *fifo = (volatile uint32_t *)UART_FIFO_REG;

    // wait until there is data in the RX FIFO
    while ( ((*status) & UART_RXFIFO_CNT) == 0) {
        ;
    }

    return (uint8_t)(*fifo & 0xFF);
}

void uart_write_string(const char* str){
    while(*str){
        uart_write_byte((uint8_t)(*str++));
    }
}
