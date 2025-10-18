# ESP32 Custom UART Driver

## Overview

This project is a lightweight, low-level UART driver for the Espressif ESP32. It was written from scratch in pure C, and works directly with hardware to manipulate registers neccessary for UART communication. The register code bypasses the abstractions in the ESP-IDF driver.

Technical reference manual - [ESP32 Technical Reference Manual Version 5.5] (https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf#uart)

Board used for testing - ESP32 Devkit V1
Chip - ESP32-WROOM-32

## Features

- **Direct Register Access:** Interacts directly with UART and GPIO matrix registers bypassing the IO Mux
- **Support for all UART Ports:** Compatible with `UART0`, `UART1`, and `UART2`. Note: `UART1` is not exposed on the board used for testing, and by default, `UART0` is connected to USB output and used by logging functions.
- **Dynamic Pin Configuration:** Using the GPIO matrix, any RX/TX pin can be assigned to any UART controller.
- **Basic I/O Functions:** Provides basic functions for reading and writing bytes as well as strings.
- **Non-Blocking Read:** Includes a non-blocking `uart_read_byte` function to enable greater control.

## API Reference

### `void uart_init(uart_t* uart_num, int baud_rate, uint8_t rx_gpio_num, uint8_t tx_gpio_num)`
Initializes UART peripheral and configures the GPIO pins and baud_rate.

- `uart_num`: A pointer to the UART instance to initialize (`&uart0`, `&uart1`, `&uart2`).
- `baud_rate`: The communication speed (9600, 115200, etc.).
- `rx_gpio_num`: GPIO pin for receiving data.
- `tx_gpio_num`: GPIO pin for transmitting data.

### `void uart_write_byte(uart_t* uart_num, uint8_t byte)`
Writes 1 byte to the chosen UART's TX FIFO buffer.

- `uart_num`: A pointer to the UART instance to initialize (`&uart0`, `&uart1`, `&uart2`).
- `byte`: The byte to be written

### `uint8_t uart_read_byte(uart_t* uart_num, uint8_t* out_byte)`
Reads 1 byte from the chosen UART's RX FIFO buffer (non-blocking).

- `uart_num`: A pointer to the UART instance to initialize (`&uart0`, `&uart1`, `&uart2`).
- `out_byte`: The pointer to where the read byte should be stored
- Returns: `1` - success; `0` - no data / error

### `void uart_block_read_byte(uart_t* uart_num, uint8_t* out_byte)`
Reads 1 byte from the chosen UART's RX FIFO buffer. Note: this function is blocking - if no string is available the watchdog will throw an error, since the function fails to yield for too long. 

- `uart_num`: A pointer to the UART instance to initialize (`&uart0`, `&uart1`, `&uart2`).
- `out_byte`: The pointer to where the read byte should be stored

### `void uart_write_string(uart_t* uart_num, const char* str)`
Writes a string to the UART port.

- `uart_num`: A pointer to the UART instance to initialize (`&uart0`, `&uart1`, `&uart2`).
- `str`: The string to be written in the form of a char array

### `void uart_read_string(uart_t* uart_num, char* buffer, size_t max_len, char delimeter)`
Reads characters from the UART port until `delimeter` is read or `max_len` is reached. Note: this function is blocking - if the end of the string is not reached the watchdog will throw an error, since the function fails to yield for too long.

- `uart_num`: A pointer to the UART instance to initialize (`&uart0`, `&uart1`, `&uart2`).
- `buffer`: The pointer to where the read chars should be stored
- `max-len`: The maximum length of string to read (usually the buffer size)
- `delimeter`: The character to look for to end the string

## How to Use

`main.c` - an example usage of reading and writing bytes and strings.

## Building the Project

This project is built using the ESP-IDF framework.

[ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html)

1.  **Configure:**
    ```bash
    idf.py menuconfig
    ```
    Set the TX and RX pins in `sdkconfig.defaults`

2.  **Build:**
    ```bash
    idf.py build
    ```

3.  **Flash:**
    ```bash
    idf.py -p /dev/ttyUSB0 flash
    ```

4.  **Monitor:**
    ```bash
    idf.py -p /dev/ttyUSB0 monitor
    ```

## Known issues

1. The write and read functions feature busy loops which can trigger the watch dog
2. This driver enables only a single UART configuration type
3. UART1 and UART0 remain untested using this driver