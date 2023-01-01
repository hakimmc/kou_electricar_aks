/*
 * wav_player.c
 *
 *  Created on: 18 May 2021
 *      Author: david
 */

#include "audio_playback.h"
#include "cs43l22.h"
#include "serial.h"
#include "wav_parser.h"
#include "fatfs.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define LOG(val) serial_print(console, val)
#define LOGLN(val) serial_println(console, val)

#define DEFAULT_MASTER_VOLUME_PERCENT 55
#define AUDIO_DIRECTORY "audio"
#define MAX_N_TRACKS 64

typedef struct {
  char filename[16];
  audio_format_t fmt;
} track_info_t;

static char msg[64];

static track_info_t tracks[MAX_N_TRACKS];
static unsigned n_tracks;
static track_info_t *current_track;
static FIL track_fp;
static uint32_t track_offset;

extern serial_t  *console;

static uint8_t noisebuf[AUDIO_PLAYBACK_BUF_SIZE];

static void load_next_audio_chunk(uint8_t *dest, uint32_t len) {
  size_t rem = current_track->fmt.data_size_bytes - track_offset;
  if (len > rem) {
    len = rem;
  }
  if (len == 0) {
    return;
  }
  UINT count;
  f_read(&track_fp, dest, len, &count);
  track_offset += len;
}

static int get_track_info(int track_idx, const char *filename) {
  FIL fp;
  uint8_t header[WAV_HEADER_LEN];
  char path[48];

  strcpy(path, AUDIO_DIRECTORY);
  strcat(path, "/");
  strcat(path, filename);

  FRESULT fr = f_open(&fp, path, FA_READ);
  if (fr != FR_OK) {
    return 1;
  }

  UINT read_count;
  size_t headerlen = sizeof(header);
  fr = f_read(&fp, header, headerlen, &read_count);
  f_close(&fp);
  if (fr != FR_OK || read_count != headerlen)
    return 1;

  track_info_t *track = &tracks[track_idx];

  if (parse_wav_header(header, &track->fmt) != 0)
    return 1;

  strcpy(track->filename, filename);

  return 0;
}

static int init_audio_chip(void *i2c) {
  if (cs43l22_init(i2c) != 0) {
    LOGLN("Failed to initialize CS43L22 audio chip!");
    return 1;
  }
  if (cs43l22_set_audio_format(DAC_FORMAT_I2S) != 0) {
    LOGLN("Failed to set DAC audio format ...");
    return 1;
  }
  if (cs43l22_set_audio_word_length(DAC_AUDIO_WORD_LENGTH_16) != 0) {
    LOGLN("Failed to set audio word length ...");
    return 1;
  }
  if (cs43l22_enable_headphones() != 0) {
    LOGLN("Failed to enable headphones ...");
    return 1;
  }
  if (cs43l22_dac_power_on() != 0) {
    LOGLN("Failed to turn audio DAC on ...");
    return 1;
  }
  if (cs43l22_set_master_volume_percent(DEFAULT_MASTER_VOLUME_PERCENT) != 0) {
    LOGLN("Failed to set master volume ...");
    return 1;
  }

  return 0;
}

int init_wav_player(void *param) {
  DIR dir;
  FILINFO info;

  if (init_audio_chip(param) != 0) {
    return 1;
  }

  FRESULT fr = f_opendir(&dir, AUDIO_DIRECTORY);
  if (fr != FR_OK) {
    LOGLN("Failed to open audio directory ...");
    return 1;
  }

  unsigned count = 0;
  while (fr == FR_OK) {
    fr = f_readdir(&dir, &info);
    if (fr != FR_OK || info.fname[0] == '\0') {
      break;
    }
    else if (strstr(info.fname, ".wav") || strstr(info.fname, ".WAV")) {
      LOG("WAV file: ");
      LOGLN(info.fname);
      if (get_track_info(count, info.fname) != 0)
        LOGLN("Failed to retrieve track info ...");
      else
        LOGLN("Got track info successfully ...");
      count++;
    }
  }
  f_closedir(&dir);
  n_tracks = count;

  return 0;
}

int wav_play(const uint8_t track_number) {
  unsigned idx = track_number-1;
  if (idx >= n_tracks) {
    LOGLN("Track index is outside of range ...");
    return 1;
  }

  current_track = &tracks[idx];
  char path[64];
  strcpy(path, AUDIO_DIRECTORY);
  strcat(path, "/");
  strcat(path, current_track->filename);
  FRESULT fr = f_open(&track_fp, path, FA_READ);
  if (fr != FR_OK) {
    LOGLN("Failed to open audio track.");
    return 1;
  }

  if (f_lseek(&track_fp, WAV_HEADER_LEN) != FR_OK) {
    LOGLN("Failed to seek to beginning of audio data.");
    return 1;
  }
  track_offset = WAV_HEADER_LEN;

  return playback_start(load_next_audio_chunk, current_track->fmt.data_size_bytes, 0);
}

void wav_pause(void) {
  playback_pause();
}

void wav_resume(void) {
  playback_resume();
}

static void read_noise_buf(uint8_t *dest, uint32_t count) {
  memcpy(dest, noisebuf, count);
}

void wav_play_noise(void) {
  size_t buflen = sizeof(noisebuf);
  // Fill buffer with noise ...
  for (int i=0; i<buflen; i++) {
    noisebuf[i] = rand();
  }

  playback_start(read_noise_buf, buflen, 1);
}

void wav_stop(void) {
  playback_stop();
}

int wav_set_volume(uint8_t percent) {
  if (percent > 100) {
    percent = 100;
  }
  return cs43l22_set_master_volume_percent(percent);
}

int wav_increase_volume(void) {
  uint8_t current = cs43l22_get_master_volume_percent();
  if (current > 100) {
    LOGLN("Volume is already at max");
    return 1;
  }
  uint8_t new = current+1;
  return cs43l22_set_master_volume_percent(new);
}

int wav_decrease_volume(void) {
  uint8_t current = cs43l22_get_master_volume_percent();
  if (current < 2) {
    LOGLN("Volume is already at min");
    return 1;
  }
  uint8_t new = current-1;
  return cs43l22_set_master_volume_percent(new);
}

uint8_t wav_get_volume(void) {
  return cs43l22_get_master_volume_percent();
}

void dump_all_tracks_info(void) {
  sprintf(msg, "%u tracks in total ...", n_tracks);
  LOGLN(msg);

  for (int i=0; i<n_tracks; i++) {
    track_info_t *track = &tracks[i];
    sprintf(msg, "Filename: %s. Length: %u bytes. Sample-rate: %u. Channels: %u.",
        track->filename, track->fmt.data_size_bytes,
        track->fmt.sample_rate, track->fmt.n_channels);
    LOGLN(msg);
  }
}

unsigned wav_get_track_count(void) {
  return n_tracks;
}

const char *wav_get_track_name(const uint8_t track_number) {
  unsigned idx = track_number-1;
  if (idx >= n_tracks) {
    LOGLN("Track index is outside of range ...");
    return NULL;
  }

  track_info_t *track = &tracks[idx];

  return track->filename;
}

