/*
 * low_level_spi.c
 *
 *  Created on: Nov 24, 2020
 *      Author: david
 */

#define USE_HARDWARE_NSS

#ifdef STM32F3
#include "stm32f3xx_ll_spi.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_dma.h"
#endif
#ifdef STM32F4
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_dma.h"
#endif
#include "main.h"

#include <stdint.h>

/**
 *  If you want to use software control of the NSS signal, comment out the following line.
 *
 *  In that case, you must define a GPIO pin as output - with the label 'SD_CARD_CS'.
 */
//#define USE_HARDWARE_NSS

#define DEFAULT_SPI_PERIPH SPI1

static SPI_TypeDef *spi = DEFAULT_SPI_PERIPH;
static DMA_TypeDef *dma = DMA2;
static uint32_t dma_tx_channel = LL_DMA_STREAM_2;
static uint32_t dma_rx_channel = LL_DMA_STREAM_0;

static uint8_t is_dma_transfer_complete(DMA_TypeDef *dma_instance, uint32_t channel) {
  switch (channel) {
  case LL_DMA_STREAM_0:
    return LL_DMA_IsActiveFlag_TC0(dma_instance);
  case LL_DMA_STREAM_2:
    return LL_DMA_IsActiveFlag_TC2(dma_instance);
  case LL_DMA_STREAM_3:
    return LL_DMA_IsActiveFlag_TC3(dma_instance);
  default:
    return 0;
  }
}

static void clear_dma_flags(DMA_TypeDef *dma, uint32_t channel) {
  switch (channel) {
  case LL_DMA_STREAM_0:
    LL_DMA_ClearFlag_TC0(dma);
    LL_DMA_ClearFlag_HT0(dma);
    break;
  case LL_DMA_STREAM_2:
    LL_DMA_ClearFlag_TC2(dma);
    LL_DMA_ClearFlag_HT2(dma);
    break;
  case LL_DMA_STREAM_3:
    LL_DMA_ClearFlag_TC3(dma);
    LL_DMA_ClearFlag_HT3(dma);
    break;
  default:
    break;
  }
}

static void clear_all_dma_flags(void) {
  clear_dma_flags(dma, dma_rx_channel);
  clear_dma_flags(dma, dma_tx_channel);
}

static void do_dma_transfer(void) {
  clear_all_dma_flags();

  LL_SPI_EnableDMAReq_RX(spi);
#ifdef STM32F4
  LL_DMA_EnableStream(dma, dma_rx_channel);
  LL_DMA_EnableStream(dma, dma_tx_channel);
#else
  LL_DMA_EnableChannel(dma, dma_rx_channel);
  LL_DMA_EnableChannel(dma, dma_tx_channel);
#endif
  LL_SPI_EnableDMAReq_TX(spi);

  LL_SPI_Enable(spi);

  while (LL_SPI_IsActiveFlag_BSY(spi)) {}
  while (!is_dma_transfer_complete(dma, dma_rx_channel)) {}

#ifdef STM32F4
  LL_DMA_DisableStream(dma, dma_tx_channel);
  LL_DMA_DisableStream(dma, dma_rx_channel);
#else
  LL_DMA_DisableChannel(dma, dma_tx_channel);
  LL_DMA_DisableChannel(dma, dma_rx_channel);
#endif
  LL_SPI_Disable(spi);
  LL_SPI_DisableDMAReq_RX(spi);
  LL_SPI_DisableDMAReq_TX(spi);
}

int sd_card_spi_write(uint8_t *data, size_t len) {
  uint8_t dummy;

  LL_DMA_SetDataLength(dma, dma_tx_channel, len);
  LL_DMA_SetMemoryAddress(dma, dma_tx_channel, (uint32_t)data);
  LL_DMA_SetMemoryIncMode(dma, dma_tx_channel, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetDataLength(dma, dma_rx_channel, len);
  LL_DMA_SetMemoryAddress(dma, dma_rx_channel, (uint32_t)&dummy);
  LL_DMA_SetMemoryIncMode(dma, dma_rx_channel, LL_DMA_MEMORY_NOINCREMENT);

  do_dma_transfer();

  return 0;
}

int sd_card_spi_read(uint8_t *data, size_t len) {
  uint8_t dummy = 0xff;

  LL_DMA_SetMemoryAddress(dma, dma_tx_channel, (uint32_t)&dummy);
  LL_DMA_SetDataLength(dma, dma_tx_channel, len);
  LL_DMA_SetMemoryIncMode(dma, dma_tx_channel, LL_DMA_MEMORY_NOINCREMENT);

  LL_DMA_SetMemoryAddress(dma, dma_rx_channel, (uint32_t)data);
  LL_DMA_SetDataLength(dma, dma_rx_channel, len);
  LL_DMA_SetMemoryIncMode(dma, dma_rx_channel, LL_DMA_MEMORY_INCREMENT);

  do_dma_transfer();

  return 0;
}

void sd_card_select(void) {
#ifndef USE_HARDWARE_NSS
  LL_GPIO_ResetOutputPin(SD_CARD_CS_GPIO_Port, SD_CARD_CS_Pin);
#endif
}

void sd_card_deselect(void) {
#ifndef USE_HARDWARE_NSS
  LL_GPIO_SetOutputPin(SD_CARD_CS_GPIO_Port, SD_CARD_CS_Pin);
#endif
}

void init_sd_card_spi(void *param) {
  spi = (SPI_TypeDef *)param;

  LL_DMA_SetPeriphAddress(dma, dma_tx_channel, (uint32_t)&(spi->DR));
  LL_DMA_SetPeriphAddress(dma, dma_rx_channel, (uint32_t)&(spi->DR));
}

