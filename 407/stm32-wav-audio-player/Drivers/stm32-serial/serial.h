/*
 * serial.h
 *
 *  Created on: 10 Jan 2020
 *      Author: david
 */

#ifndef SERIAL_H_
#define SERIAL_H_

// Include the main.h header to ensure the macro for the relevant MCU has been defined
#include "main.h"

#ifdef STM32L1
#include "stm32l1xx_ll_usart.h"
#elif defined STM32F3
#include "stm32f3xx_ll_usart.h"
#elif defined STM32F4
#include "stm32f4xx_ll_usart.h"
#elif defined STM32L4
#include "stm32l4xx_ll_usart.h"
#elif defined STM32WB
#include "stm32wbxx_ll_usart.h"
#endif

struct serial;

typedef struct serial serial_t;

typedef enum {
    SERIAL_LINE_ENDING_NONE,
    SERIAL_LINE_ENDING_CR,
    SERIAL_LINE_ENDING_LF,
    SERIAL_LINE_ENDING_CRLF
} serial_line_ending_t;

serial_t *serial_init(USART_TypeDef *usart);
void serial_set_line_ending(serial_t *serial, serial_line_ending_t ending);
void serial_set_baud_rate(serial_t *serial, uint32_t baud_rate);
uint32_t serial_get_baud_rate(serial_t *serial);

// DMA routines
void serial_configure_dma(serial_t *serial, DMA_TypeDef *dma_instance, uint32_t channel);
size_t serial_print_bytes_dma(serial_t *serial, const uint8_t *data, uint16_t len);
size_t serial_print_bytes_dma_no_wait(serial_t *serial, const uint8_t *data, uint16_t len);
size_t serial_print_dma(serial_t *serial, const char *msg);
size_t serial_println_dma(serial_t *serial, const char *msg);

// Transmission routines
size_t serial_print(serial_t *serial, const char *msg);
size_t serial_println(serial_t *serial, const char *msg);
size_t serial_print_bytes(serial_t *serial, const uint8_t *data, int len);

// Receive routines
int serial_read_bytes(serial_t *serial, char *buf, int max_len);
int serial_available(serial_t *serial);

// ISR callback
void serial_isr_callback(serial_t *serial);

void serial_disable_rx_interrupt(serial_t *serial);
void serial_enable_rx_interrupt(serial_t *serial);

#endif /* SERIAL_H_ */
