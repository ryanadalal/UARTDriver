/**
 * @file uart_driver.h
 * @brief Header for custom ESP32 UART driver.
 *
 * Provide basic UART functions for an ESP32 in pure C.
 */

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Represents one UART peripheral.
 *
 * This struct contains pointers to ESP32's hardware registers for
 * one specific UART port.
 */
typedef struct {
    volatile uint32_t *fifo_reg;    // Pointer to the UART FIFO register for reading/writing data.
    volatile uint32_t *status_reg;  // Pointer to the UART status register.
    volatile uint32_t *conf0_reg;   // Pointer to the UART configuration register 0.
    volatile uint32_t *conf1_reg;   // Pointer to the UART configuration register 1.
    volatile uint32_t *clkdiv_reg;  // Pointer to the UART clock divider register.
} uart_t;

/**
 * @brief Declare the 3 UART peripherals on the ESP32.
 */
extern uart_t uart0; // UART Port 0
extern uart_t uart1; // UART Port 1
extern uart_t uart2; // UART Port 2

/**
 * @brief Initialize a UART peripheral.
 *
 * Configures the chosen UART port with a baud rate and
 * specified GPIO pins for RX and TX.
 *
 * @param uart_num Pointer to the UART instance to initialize
 * @param baud_rate The target baud rate
 * @param rx_gpio_num GPIO pin for receiving data (RX).
 * @param tx_gpio_num GPIO pin for transmitting data (TX).
 */
void uart_init(uart_t* uart_num, int baud_rate, uint8_t rx_gpio_num, uint8_t tx_gpio_num);

/**
 * @brief Configures GPIO matrix for UART.
 *
 * Configures ESP32's GPIO matrix and connects UART.
 * to the specified GPIO pins.
 *
 * @param uart_num Pointer to the UART instance.
 * @param rx_gpio_num GPIO pin for RX.
 * @param tx_gpio_num GPIO pin for TX.
 */
void configure_gpio_for_uart(uart_t* uart_num, uint8_t rx_gpio_num, uint8_t tx_gpio_num);

/**
 * @brief Writes 1 byte to the UART TX FIFO.
 *
 * Writes 1 byte and busy-waits until the TX FIFO is not full.
 *
 * @param uart_num Pointer to the UART instance.
 * @param byte The byte to be written.
 */
void uart_write_byte(uart_t* uart_num, uint8_t byte);

/**
 * @brief Writes a string to the UART port.
 *
 * Writes 1 byte at a time and busy-waits when the TX FIFO is full.
 * 
 * @param uart_num Pointer to the UART instance.
 * @param str The string to write.
 */
void uart_write_string(uart_t* uart_num, const char* str);

/**
 * @brief Reads 1 byte from the UART RX FIFO (non-blocking).
 *
 * Checks if at least 1 byte is available returns false if not.
 * Reads the byte and stores it in out_byte and returns true
 *
 * @param uart_num Pointer to the UART instance.
 * @param out_byte Pointer to where the read byte should be stored.
 * @return 1 if byte read, 0 otherwise.
 */
uint8_t uart_read_byte(uart_t* uart_num, uint8_t* out_byte);

/**
 * @brief Reads 1 byte from the UART RX FIFO (blocking).
 *
 * Busy-wait until a byte is available to be read.
 * Read the byte and stores it in out_byte.
 *
 * @param uart_num Pointer to the UART instance.
 * @param out_byte Pointer to where the read byte should be stored.
 */
void uart_block_read_byte(uart_t* uart_num, uint8_t* out_byte);

/**
 * @brief Reads a string from the UART port
 * 
 * Reads characters and stores them in the buffer until either
 * the specified delimeter is found or the max length is reached
 * Busy-wait for each character
 *
 * @param uart_num Pointer to the UART instance.
 * @param buffer Pointer to where the string should be stored.
 * @param max_len The max number of characters to read (usually the size of the buffer).
 * @param delimeter The character to end the string.
 */
void uart_read_string(uart_t* uart_num, char* buffer, size_t max_len, char delimeter);