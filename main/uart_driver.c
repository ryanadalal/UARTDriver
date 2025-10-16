#include "uart_driver.h"
#include <unistd.h>

// constants
#define UART_FIFO_LEN 128 // max size of buffers


// base addresses for UART peripherals
#define UART0_BASE 0x3FF40000
#define UART1_BASE 0x3FF50000
#define UART2_BASE 0x3FF6E000

// relative register positions from the base address
#define UART_CONF0_REG (0x20)
#define UART_CONF1_REG (0x24)
#define UART_CLKDIV_REG (0x14)
#define UART_FIFO_REG (0x0)
#define UART_STATUS_REG (0x1C)

/*
* Masks for UART Registers
*/
// conf 0 masks
#define UART_BIT_NUM_MASK (0x3 << 2)
#define UART_BIT_NUM_8BITS (0x3 << 2) // 8 data bits
#define UART_PARITY_EN_MASK (0x1 << 1)
#define UART_PARITY_DISABLE (0x0 << 1) // disable parity
#define UART_STOP_BIT_NUM_MASK (0x3 << 4)
#define UART_STOP_BIT_1 (0x1 << 4) // 1 stop bit
// conf 1 masks
#define UART_RX_FLOW_EN_MASK (0x1 << 23) 
#define UART_RX_FLOW_DISABLE (0x0 << 23) // disable flow control
#define UART_TXFIFO_EMPTY_THRHD_MASK (0x7F << 8) 
#define UART_TXFIFO_EMPTY_THRHD (0x10 << 8) // trigger refill when <= 16 bytes
#define UART_RXFIFO_FULL_THRHD_MASK (0x7F << 0) 
#define UART_RXFIFO_FULL_THRHD (0x10 << 0) // trigger read when >= 16 bytes
#define UART_RX_TOUT_EN_MASK (0x1 << 31) 
#define UART_RX_TOUT_EN (0x1 << 31) // enable RX timeout
#define UART_RX_TOUT_THRHD_MASK (0x7F << 24) 
#define UART_RX_TOUT_THRHD (0x8 << 24) // timeout after 8 bit times
// read write masks
#define UART_RXFIFO_CNT_MASK (0xFF << 0) // mask for RX FIFO count
#define UART_TXFIFO_CNT_MASK (0xFF << 16) // mask for TX FIFO count

//
// ==========================================================================
//

// GPIO Matrix and Pin Configuration
#define GPIO_OUT_REG 0x3FF44004
#define GPIO_OUT_W1TS_REG 0x3FF44008
#define GPIO_OUT_W1TC_REG 0x3FF4400C
#define GPIO_ENABLE_REG 0x3FF44020
#define GPIO_ENABLE_W1TS_REG 0x3FF44024
#define GPIO_ENABLE_W1TC_REG 0x3FF44028
#define GPIO_IN_REG 0x3FF4403C
#define GPIO_PIN16_REG 0x3FF44088
#define GPIO_PIN17_REG 0x3FF4408C
#define GPIO_FUNC16_OUT_SEL_REG 0x3FF44570
#define GPIO_FUNC17_OUT_SEL_REG 0x3FF44574
#define GPIO_FUNC_IN_SEL_CFG_REG(n) (0x3FF44130 + (n)*4)

// UART signal numbers for GPIO matrix
#define U2TXD_OUT_IDX 198
#define U2RXD_IN_IDX 198
//
// ==========================================================================
//

// peripheral enable and reset
// register positions
#define DPORT_PERIP_CLK_EN_REG (0x3FF000C0)
#define DPORT_PERIP_RST_EN_REG (0x3FF000C4)
// bit masks
#define DPORT_UART_MEM_CLOCK_EN_MASK (1 << 24)

#define DPORT_UART1_CLK_EN_MASK (1 << 5)
#define DPORT_UART0_CLK_EN_MASK (1 << 2)
#define DPORT_UART2_CLK_EN_MASK (1 << 23)

#define DPORT_UART1_RST_HOLD_MASK (1 << 5)
#define DPORT_UART0_RST_HOLD_MASK (1 << 2)
#define DPORT_UART2_RST_HOLD_MASK (1 << 23)

uart_t uart1 = {
    .fifo_reg = (volatile uint32_t *)(UART1_BASE + UART_FIFO_REG),
    .status_reg = (volatile uint32_t *)(UART1_BASE + UART_STATUS_REG),
    .conf0_reg = (volatile uint32_t *)(UART1_BASE + UART_CONF0_REG),
    .conf1_reg = (volatile uint32_t *)(UART1_BASE + UART_CONF1_REG),
    .clkdiv_reg = (volatile uint32_t *)(UART1_BASE + UART_CLKDIV_REG),
};

uart_t uart0 = {
    .fifo_reg = (volatile uint32_t *)(UART0_BASE + UART_FIFO_REG),
    .status_reg = (volatile uint32_t *)(UART0_BASE + UART_STATUS_REG),
    .conf0_reg = (volatile uint32_t *)(UART0_BASE + UART_CONF0_REG),
    .conf1_reg = (volatile uint32_t *)(UART0_BASE + UART_CONF1_REG),
    .clkdiv_reg = (volatile uint32_t *)(UART0_BASE + UART_CLKDIV_REG),
};

uart_t uart2 = {
    .fifo_reg = (volatile uint32_t *)(UART2_BASE + UART_FIFO_REG),
    .status_reg = (volatile uint32_t *)(UART2_BASE + UART_STATUS_REG),
    .conf0_reg = (volatile uint32_t *)(UART2_BASE + UART_CONF0_REG),
    .conf1_reg = (volatile uint32_t *)(UART2_BASE + UART_CONF1_REG),
    .clkdiv_reg = (volatile uint32_t *)(UART2_BASE + UART_CLKDIV_REG),
};

//
// ==========================================================================
//
void configure_gpio_for_uart2(void) {
    // Configure GPIO16 as input (RX)
    volatile uint32_t *gpio_enable_reg = (volatile uint32_t *)GPIO_ENABLE_REG;
    *gpio_enable_reg &= ~(1 << 16); // Disable output for GPIO16
    
    // Configure GPIO17 as output (TX) 
    *gpio_enable_reg |= (1 << 17); // Enable output for GPIO17
    
    // Set IO_MUX for GPIO16 and GPIO17 to function 2 (GPIO function)
    *(volatile uint32_t*)0x3FF4904C = (*(volatile uint32_t*)0x3FF4904C & ~0x7) | 2; // GPIO16
    *(volatile uint32_t*)0x3FF49050 = (*(volatile uint32_t*)0x3FF49050 & ~0x7) | 2; // GPIO17
    
    // Configure GPIO matrix routing
    // Route UART2 TX signal to GPIO17
    *(volatile uint32_t*)GPIO_FUNC17_OUT_SEL_REG = U2TXD_OUT_IDX;
    
    // Route GPIO16 to UART2 RX signal
    *(volatile uint32_t*)GPIO_FUNC_IN_SEL_CFG_REG(U2RXD_IN_IDX) = (1 << 7) | 16; // bit 7 = sig_in_sel, bits 5:0 = gpio_num
    
    // Configure GPIO pin registers
    *(volatile uint32_t*)GPIO_PIN16_REG = 0; // Clear any special config
    *(volatile uint32_t*)GPIO_PIN17_REG = 0; // Clear any special config
}
//
// ==========================================================================
//

void uart_init(uart_t* uart_num, int baud_rate){
    // enable UART1 peripheral clock
    volatile uint32_t *clk_en_reg = (volatile uint32_t *)DPORT_PERIP_CLK_EN_REG;
    uint32_t clk_en_val = *clk_en_reg;
    clk_en_val &= ~(DPORT_UART2_CLK_EN_MASK | DPORT_UART1_CLK_EN_MASK | DPORT_UART0_CLK_EN_MASK | DPORT_UART_MEM_CLOCK_EN_MASK);
    clk_en_val |= DPORT_UART2_CLK_EN_MASK | DPORT_UART1_CLK_EN_MASK | DPORT_UART0_CLK_EN_MASK | DPORT_UART_MEM_CLOCK_EN_MASK;
    *clk_en_reg = clk_en_val;
    // reset UART1 peripheral
    volatile uint32_t *rst_reg = (volatile uint32_t *)DPORT_PERIP_RST_EN_REG;
    *rst_reg |= DPORT_UART2_RST_HOLD_MASK | DPORT_UART1_RST_HOLD_MASK | DPORT_UART0_RST_HOLD_MASK;
    *rst_reg &= ~(DPORT_UART2_RST_HOLD_MASK | DPORT_UART1_RST_HOLD_MASK | DPORT_UART0_RST_HOLD_MASK);


    // UART for 8N1 bits - 8 data, no parity, 1 stop bit
    volatile uint32_t *conf_0_reg = uart_num->conf0_reg;
    uint32_t conf_0_val = *conf_0_reg;
    conf_0_val &= ~(UART_BIT_NUM_MASK | UART_PARITY_EN_MASK | UART_STOP_BIT_NUM_MASK);
    conf_0_val |= UART_BIT_NUM_8BITS | UART_PARITY_DISABLE | UART_STOP_BIT_1;
    *conf_0_reg = conf_0_val;

    uint32_t clk_div = (80000000 / (baud_rate * 16));
    volatile uint32_t *clk_div_reg = uart_num->clkdiv_reg;
    *clk_div_reg = clk_div;

    volatile uint32_t *conf_1_reg = uart_num->conf1_reg;
    uint32_t conf_1_val = *conf_1_reg;
    conf_1_val &= ~(UART_RX_FLOW_EN_MASK | UART_TXFIFO_EMPTY_THRHD_MASK | UART_RXFIFO_FULL_THRHD_MASK | UART_RX_TOUT_EN_MASK | UART_RX_TOUT_THRHD_MASK);
    conf_1_val |= UART_RX_FLOW_DISABLE | UART_TXFIFO_EMPTY_THRHD | UART_RXFIFO_FULL_THRHD | UART_RX_TOUT_EN | UART_RX_TOUT_THRHD;
    *conf_1_reg = conf_1_val;
    
    // If this is UART2, configure the GPIO pins
    if (uart_num == &uart2) {
        configure_gpio_for_uart2();
    }
}

void uart_write_byte(uart_t* uart_num, uint8_t byte){
    volatile uint32_t *status = uart_num->status_reg;
    volatile uint32_t *fifo = uart_num->fifo_reg;

    // wait until there is space in the TX FIFO
    while ( (((*status) & UART_TXFIFO_CNT_MASK) >> 16) >= UART_FIFO_LEN) {
        ;
    }

    *fifo = byte;
}

uint8_t uart_block_read_byte(uart_t* uart_num){
    volatile uint32_t *status = uart_num->status_reg;
    volatile uint32_t *fifo = uart_num->fifo_reg;

    // wait until there is data in the RX FIFO
    while ( ((*status) & UART_RXFIFO_CNT_MASK) == 0) {
        ;
    }

    return (uint8_t)(*fifo & 0xFF);
}
uint8_t uart_read_byte(uart_t* uart_num, uint8_t* out_byte){
    volatile uint32_t *status = uart_num->status_reg;
    volatile uint32_t *fifo = uart_num->fifo_reg;

    if ( ((*status) & UART_RXFIFO_CNT_MASK) != 0) {
        *out_byte = (uint8_t)(*fifo & 0xFF);
        return 1; // indicate success
    }

    return 0; // indicate no data available
}

void uart_write_string(uart_t* uart_num, const char* str){
    while(*str){
        uart_write_byte(uart_num, (uint8_t)(*str++));
    }
}


int _write(int fd, const void *buf, size_t count) {
    const uint8_t *data = (const uint8_t *)buf;

    if (fd != 1 && fd != 2) return -1; // stdout/stderr only

    for (size_t i = 0; i < count; i++) {
        uart_write_byte(&uart0, data[i]); 
    }

    return count;
}