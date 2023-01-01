/*
 * audio_playback.c
 *
 *  Created on: 18 Apr 2020
 *      Author: david
 */

#include "audio_playback.h"
#include "serial.h"

#include <string.h>

#include "stm32f4xx_hal.h"

extern serial_t *console;

static fill_audio_buf_cb_t fill_audio_buf_cb;
static I2S_HandleTypeDef *i2s;
static uint8_t loop;
static uint32_t offset;
static uint32_t total_len;

static void (*on_playback_stopped)(void);

static uint8_t audio_bufs[2][AUDIO_PLAYBACK_BUF_SIZE];

void playback_init(void *i2s_handle) {
	i2s = (I2S_HandleTypeDef *) i2s_handle;
}

static void update_audio_buf(uint8_t buf_number) {
  if (!fill_audio_buf_cb) {
    // If we are only transmitting audio data to ensure that the
    // audio DAC has an I2S clock source, then we don't need to
    // copy anything in to the audio buffers ...
    return;
  }
	uint8_t *dest = audio_bufs[buf_number];

	size_t len = AUDIO_PLAYBACK_BUF_SIZE;
	size_t remaining = total_len - offset;
	if (len > remaining) {
		len = remaining;
	}
	fill_audio_buf_cb(dest, len);
	offset += len;
}

void playback_stop(void) {
	HAL_I2S_DMAStop(i2s);
}

void playback_pause(void) {
  HAL_I2S_DMAPause(i2s);
}

void playback_resume(void) {
  HAL_I2S_DMAResume(i2s);
}

int playback_start(fill_audio_buf_cb_t cb, const uint32_t len, uint8_t loop_audio) {
	total_len = len;
	offset = 0;
	fill_audio_buf_cb = cb;
	update_audio_buf(0);
	loop = loop_audio;
	size_t count = AUDIO_PLAYBACK_BUF_SIZE;
	if (count > len) {
		count = len;
	}
	HAL_StatusTypeDef status = HAL_I2S_Transmit_DMA(i2s, (uint16_t *)audio_bufs[0], count);
	if (status != HAL_OK) {
		return 1;
	}

	return 0;
}

void playback_i2s_txfer_cplt_callback(void) {
	if (offset >= total_len) {
		if (loop) {
			offset = 0;
		}
		else {
			playback_stop();
			if (on_playback_stopped)
			  on_playback_stopped();
		}
	}
	else {
		update_audio_buf(1);
	}
}

void playback_i2s_txfer_half_cplt_callback(void) {
	if (loop || offset < total_len) {
		update_audio_buf(0);
	}
}

void playback_set_on_stop_callback(void (*callback)(void)) {
	on_playback_stopped = callback;
}

int playback_provide_i2s_clock(void) {
	return playback_start(NULL, AUDIO_PLAYBACK_BUF_SIZE*16, 1);
}

void playback_halt_i2s_clock(void) {
	playback_stop();
}

