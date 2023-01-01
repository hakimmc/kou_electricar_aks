/*
 * serial.c
 *
 *  Created on: 10 Jan 2020
 *      Author: david
 */

#include "serial.h"
#include <string.h>

#ifndef SERIAL_TX_BUF_SIZE
#define SERIAL_TX_BUF_SIZE 128
#endif

#ifndef SERIAL_RX_BUF_SIZE
#define SERIAL_RX_BUF_SIZE 128
#endif

#ifndef SERIAL_N_INTERFACES
#define SERIAL_N_INTERFACES 2
#endif

typedef struct {
    int pos;
    int len;
    char data[SERIAL_TX_BUF_SIZE];
} serial_tx_buf_t;

typedef struct {
    int rd;
    int wr;
    char data[SERIAL_RX_BUF_SIZE];
} serial_rx_buf_t;

struct serial {
    volatile serial_tx_buf_t tx_buf;
    volatile serial_rx_buf_t rx_buf;
    USART_TypeDef *usart;
    serial_line_ending_t line_ending;
    int overrun_error_count;
    DMA_TypeDef *dma_instance;
    uint32_t dma_channel;
};

static serial_t ifs[SERIAL_N_INTERFACES];
static uint8_t next_if = 0; // next interface
static volatile uint8_t rx_lock;

serial_t *serial_init(USART_TypeDef *_usart) {
    if (next_if >= SERIAL_N_INTERFACES) {
        return NULL;
    }
    serial_t *s = &ifs[next_if++];
    s->usart = _usart;
    s->line_ending = SERIAL_LINE_ENDING_LF;
    LL_USART_EnableIT_RXNE(s->usart);

    return s;
}

void serial_configure_dma(serial_t *serial, DMA_TypeDef *dma_instance, uint32_t channel) {
    serial->dma_instance = dma_instance;
    serial->dma_channel = channel;
    LL_USART_EnableDMAReq_TX(serial->usart);
}

#ifndef STM32WB
void serial_set_baud_rate(serial_t *serial, uint32_t baud_rate) {
    LL_RCC_ClocksTypeDef rcc_clocks;
    uint32_t clock_frequency;

    LL_RCC_GetSystemClocksFreq(&rcc_clocks);
    if (serial->usart == USART1) {
        clock_frequency = rcc_clocks.PCLK2_Frequency;
    }
#ifdef USART2
    else if (serial->usart == USART2) {
        clock_frequency = rcc_clocks.PCLK1_Frequency;
    }
#endif
#ifdef USART3
    else if (serial->usart == USART3) {
        clock_frequency = rcc_clocks.PCLK1_Frequency;
    }
#endif

    LL_USART_SetBaudRate(serial->usart, clock_frequency,
            LL_USART_OVERSAMPLING_16, baud_rate);
}

uint32_t serial_get_baud_rate(serial_t *serial) {
    LL_RCC_ClocksTypeDef rcc_clocks;
    uint32_t clock_frequency;

    LL_RCC_GetSystemClocksFreq(&rcc_clocks);
    if (serial->usart == USART1) {
        clock_frequency = rcc_clocks.PCLK2_Frequency;
    }
#ifdef USART2
    else if (serial->usart == USART2) {
        clock_frequency = rcc_clocks.PCLK1_Frequency;
    }
#endif
#ifdef USART3
    else if (serial->usart == USART3) {
        clock_frequency = rcc_clocks.PCLK1_Frequency;
    }
#endif

    return LL_USART_GetBaudRate(serial->usart, clock_frequency, LL_USART_OVERSAMPLING_16);
}
#endif

static void on_tx_ready(serial_t *serial) {
    LL_USART_EnableIT_TXE(serial->usart);
    while (LL_USART_IsEnabledIT_TXE(serial->usart)) {};
}

/**
 * DMA routines
 */

static uint32_t dma_transfer_complete(DMA_TypeDef *dma, uint32_t channel) {
	switch (channel) {
	case LL_DMA_CHANNEL_1:
		return LL_DMA_IsActiveFlag_TC1(dma);
	case LL_DMA_CHANNEL_2:
		return LL_DMA_IsActiveFlag_TC2(dma);
	case LL_DMA_CHANNEL_3:
		return LL_DMA_IsActiveFlag_TC3(dma);
	case LL_DMA_CHANNEL_4:
		return LL_DMA_IsActiveFlag_TC4(dma);
	case LL_DMA_CHANNEL_5:
		return LL_DMA_IsActiveFlag_TC5(dma);
	case LL_DMA_CHANNEL_6:
		return LL_DMA_IsActiveFlag_TC6(dma);
	case LL_DMA_CHANNEL_7:
		return LL_DMA_IsActiveFlag_TC7(dma);
	default:
		return 0;
	}
}

static void dma_clear_flags(DMA_TypeDef *dma, uint32_t channel) {
	switch (channel) {
	case LL_DMA_CHANNEL_1:
		LL_DMA_ClearFlag_TC1(dma);
		break;
	case LL_DMA_CHANNEL_2:
		LL_DMA_ClearFlag_TC2(dma);
		break;
	case LL_DMA_CHANNEL_3:
		LL_DMA_ClearFlag_TC3(dma);
		break;
	case LL_DMA_CHANNEL_4:
		LL_DMA_ClearFlag_TC4(dma);
		break;
	case LL_DMA_CHANNEL_5:
		LL_DMA_ClearFlag_TC5(dma);
		break;
	case LL_DMA_CHANNEL_6:
		LL_DMA_ClearFlag_TC6(dma);
		break;
	case LL_DMA_CHANNEL_7:
		LL_DMA_ClearFlag_TC7(dma);
		break;
	default:
		break;
	}
}

size_t serial_print_bytes_dma_no_wait(serial_t *serial, const uint8_t *data, uint16_t len) {
    serial_tx_buf_t *buf = &(serial->tx_buf);
    len = len > sizeof(buf->data) ? sizeof(buf->data) : len;
    memcpy(buf->data, data, len);
    // technically we don't need to set these parameters for DMA
    buf->len = len;
    buf->pos = 0;
    // Configure channel ...
    LL_DMA_DisableChannel(serial->dma_instance, serial->dma_channel);
#ifdef STM32L1
    LL_DMA_SetPeriphAddress(serial->dma_instance, serial->dma_channel, (uint32_t) &(serial->usart->DR));
#else
    LL_DMA_SetPeriphAddress(serial->dma_instance, serial->dma_channel, (uint32_t) &(serial->usart->DR));
#endif
    LL_DMA_SetMemoryAddress(serial->dma_instance, serial->dma_channel, (uint32_t) buf->data);
    LL_DMA_SetDataLength(serial->dma_instance, serial->dma_channel, len);
    // Clear previous flags and enable transmission ...
    dma_clear_flags(serial->dma_instance, serial->dma_channel);
    LL_DMA_EnableChannel(serial->dma_instance, serial->dma_channel);

    return len;
}

size_t serial_print_bytes_dma(serial_t *serial, const uint8_t *data, uint16_t len) {
    size_t count = serial_print_bytes_dma_no_wait(serial, data, len);
    // Wait for DMA transfer to complete ...
    while (!dma_transfer_complete(serial->dma_instance, serial->dma_channel));
    // Wait for USART transmission to complete ...
    while (!LL_USART_IsActiveFlag_TC(serial->usart));

    return count;
}

size_t serial_print_dma(serial_t *serial, const char *msg) {
    int len = strlen(msg);
    return serial_print_bytes_dma(serial, (uint8_t *)msg, len);
}

size_t serial_println_dma(serial_t *serial, const char *msg) {
    size_t count = serial_print_dma(serial, msg);
    switch (serial->line_ending) {
        case SERIAL_LINE_ENDING_LF:
            count += serial_print_dma(serial, "\n");
            break;
        case SERIAL_LINE_ENDING_CR:
            count += serial_print_dma(serial, "\r");
            break;
        case SERIAL_LINE_ENDING_CRLF:
            count += serial_print_dma(serial, "\r\n");
            break;
        default:
            break;
    }

    return count;
}

/**
 * End DMA routines
 */


size_t serial_print(serial_t *serial, const char *msg) {
    int len = strlen(msg);
    return serial_print_bytes(serial, msg, len);
}

size_t serial_println(serial_t *serial, const char *msg) {
    size_t count = serial_print(serial, msg);
    switch (serial->line_ending) {
        case SERIAL_LINE_ENDING_LF:
            count += serial_print(serial, "\n");
            break;
        case SERIAL_LINE_ENDING_CR:
            count += serial_print(serial, "\r");
            break;
        case SERIAL_LINE_ENDING_CRLF:
            count += serial_print(serial, "\r\n");
            break;
        default:
            break;
    }

    return count;
}

size_t serial_print_bytes(serial_t *serial, const uint8_t *data, int len) {
    serial_tx_buf_t *buf = &(serial->tx_buf);
    len = len > sizeof(buf->data) ? sizeof(buf->data) : len;
    memcpy(buf->data, data, len);
    buf->len = len;
    buf->pos = 0;
    on_tx_ready(serial);

    return len;
}

void serial_set_line_ending(serial_t *serial, serial_line_ending_t ending) {
    serial->line_ending = ending;
}

int serial_available(serial_t *serial) {
    serial_rx_buf_t *buf = &(serial->rx_buf);

    if (buf->wr == buf->rd) { return 0; }
    if (buf->wr > buf->rd) { return buf->wr - buf->rd; }

    return buf->wr + sizeof(buf->data) - buf->rd;
}

int serial_read_bytes(serial_t *serial, char *dest, int max_len) {
    if (rx_lock) {
        return 0;
    }
    serial_rx_buf_t *buf = &(serial->rx_buf);

    int available = serial_available(serial);

    int len = max_len > available ? available : max_len;

    if (len <= 0) {
        return 0;
    }

    char *src = buf->data + buf->rd;
    int n_bytes_at_end = sizeof(buf->data) - buf->rd;
    if (buf->wr > buf->rd || n_bytes_at_end >= len) {
        memcpy(dest, src, len);
        buf->rd += len;
        if (buf->rd >= sizeof(buf->data)) {
            buf->rd = 0;
        }
        return len;
    }

    // We need to copy some bytes from end of buffer and some from start of buffer ...
    memcpy(dest, src, n_bytes_at_end);
    int n_bytes_at_start = len - n_bytes_at_end;
    memcpy(dest + n_bytes_at_end, buf->data, n_bytes_at_start);
    buf->rd = n_bytes_at_start;

    return len;
}

static void serial_tx_callback(serial_t *serial) {
    if (LL_USART_IsEnabledIT_TXE(serial->usart) && LL_USART_IsActiveFlag_TXE(serial->usart)) {
        serial_tx_buf_t *buf = &serial->tx_buf;
        uint8_t byte = buf->data[buf->pos++];
        LL_USART_TransmitData8(serial->usart, byte);

        if (buf->pos >= buf->len) {
            LL_USART_DisableIT_TXE(serial->usart);
        }
    }
}

static void serial_rx_callback(serial_t *serial) {
    if (LL_USART_IsActiveFlag_ORE(serial->usart)) {
        LL_USART_ClearFlag_ORE(serial->usart);
        serial->overrun_error_count++;
    }
    if (LL_USART_IsEnabledIT_RXNE(serial->usart) && LL_USART_IsActiveFlag_RXNE(serial->usart)) {
        rx_lock = 1;
        serial_rx_buf_t *buf = &(serial->rx_buf);
        uint8_t byte = LL_USART_ReceiveData8(serial->usart);
        buf->data[buf->wr++] = byte;
        if (buf->wr >= sizeof(buf->data)) {
            buf->wr = 0;
        }
        rx_lock = 0;
    }
}

void serial_isr_callback(serial_t *serial) {
    serial_tx_callback(serial);
    serial_rx_callback(serial);
}

void serial_disable_rx_interrupt(serial_t *serial) {
    LL_USART_DisableIT_RXNE(serial->usart);
}

void serial_enable_rx_interrupt(serial_t *serial) {
    LL_USART_EnableIT_RXNE(serial->usart);
}


