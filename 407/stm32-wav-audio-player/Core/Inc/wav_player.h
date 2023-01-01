/*
 * wav_player.h
 *
 *  Created on: 18 May 2021
 *      Author: david
 */

#ifndef INC_WAV_PLAYER_H_
#define INC_WAV_PLAYER_H_

#include <stdint.h>

int init_wav_player(void *param);

void wav_play_noise(void);
int wav_play(const uint8_t track_number);
void wav_stop(void);
void wav_pause(void);
void wav_resume(void);

int wav_set_volume(uint8_t percent);
int wav_increase_volume(void);
int wav_decrease_volume(void);
uint8_t wav_get_volume(void);

unsigned wav_get_track_count(void);
void dump_all_tracks_info(void);
const char *wav_get_track_name(const uint8_t track_number);

#endif /* INC_WAV_PLAYER_H_ */
