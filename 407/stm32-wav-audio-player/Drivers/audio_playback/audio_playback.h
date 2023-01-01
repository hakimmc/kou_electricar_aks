/*
 * audio_playback.h
 *
 *  Created on: 18 Apr 2020
 *      Author: david
 */

#ifndef AUDIO_PLAYBACK_H_
#define AUDIO_PLAYBACK_H_

#include <stdint.h>

#define AUDIO_PLAYBACK_BUF_SIZE 4096

typedef void (*fill_audio_buf_cb_t)(uint8_t *dest, uint32_t count);

void playback_init(void *i2s_handle);
int playback_start(fill_audio_buf_cb_t cb, const uint32_t len, uint8_t loop_audio);
void playback_stop(void);
void playback_pause(void);
void playback_resume(void);

void playback_i2s_txfer_cplt_callback(void);
void playback_i2s_txfer_half_cplt_callback(void);
void playback_set_on_stop_callback(void (*callback)(void));
int playback_provide_i2s_clock(void);
void playback_halt_i2s_clock(void);

#endif /* AUDIO_PLAYBACK_H_ */
