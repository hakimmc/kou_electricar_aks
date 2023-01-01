
#include "rotary_encoder.h"
#include "serial.h"
#include "wav_player.h"

#include "main.h"
#include <stdio.h>

#define INTERVAL_SHORT_PRESS_MS 200
#define INTERVAL_NORMAL_PRESS_MS 750

#define LOG(val) serial_print(console, val)
#define LOGLN(val) serial_println(console, val)

typedef enum {
  MODE_TRACK,
  MODE_VOLUME
} control_mode_t;

typedef enum {
  PB_STOPPED,
  PB_PLAYING,
  PB_PAUSED
} playback_state_t;

extern serial_t *console;
static char msg[48];

static uint32_t t_press;
static control_mode_t mode = MODE_TRACK;
static playback_state_t pb_state = PB_STOPPED;

static uint8_t track_num = 1;

static void toggle_playback(void) {
  if (pb_state == PB_STOPPED) {
    LOGLN("Starting playback ...");
    wav_play(track_num);
    pb_state = PB_PLAYING;
  }
  else {
    LOGLN("Stopping playback ...");
    wav_stop();
    pb_state = PB_STOPPED;
  }
}

static void on_short_press(void) {
  LOGLN("short press");
  if (mode != MODE_TRACK) {
    LOGLN("Exiting volume control mode ...");
    mode = MODE_TRACK;
    return;
  }

  switch (pb_state) {
  case PB_PLAYING:
    LOGLN("Pausing playback");
    wav_pause();
    pb_state = PB_PAUSED;
    break;
  case PB_PAUSED:
    LOGLN("Resuming playback");
    wav_resume();
    pb_state = PB_PLAYING;
    break;
  case PB_STOPPED:
  default:
    break;
  }
}

static void on_normal_press(void) {
  LOGLN("normal press");
  if (mode == MODE_TRACK) {
    toggle_playback();
  }
  else {
    LOGLN("Exiting volume control mode ...");
    mode = MODE_TRACK;
  }
}

static void on_long_press(void) {
  LOGLN("long press");
  if (mode == MODE_TRACK) {
    LOGLN("Switching to volume control mode");
    mode = MODE_VOLUME;
  }
  else {
    LOGLN("Exiting volume control mode ...");
    mode = MODE_TRACK;
  }
}

static void on_switch_press(void) {
  t_press = HAL_GetTick();
  LOGLN("Switch press");
}

static void on_switch_release(void) {
  uint32_t now = HAL_GetTick();
  if (now - t_press < INTERVAL_SHORT_PRESS_MS) {
    on_short_press();
  }
  else if (now - t_press < INTERVAL_NORMAL_PRESS_MS) {
    on_normal_press();
  }
  else {
    on_long_press();
  }
}

void on_encoder_switch(uint8_t dir) {
  if (dir) {
    on_switch_release();
  }
  else {
    on_switch_press();
  }
}

void on_encoder_rotate(uint8_t dir, int16_t pos) {
  if (mode == MODE_TRACK) {
    unsigned n_tracks = wav_get_track_count();
    //sprintf(msg, "Rotate. Dir: %s. Pos: %d", dir ? "CW" : "CCW", pos);
    if (dir) {
      track_num++;
      if (track_num > n_tracks)
        track_num = 1;
    }
    else if (track_num > 1) {
      track_num--;
    } else {
      LOGLN("Track index outside range!");
    }
    const char *name = wav_get_track_name(track_num);
    sprintf(msg, "Track num: %u. Name: %s", track_num, name);
    LOGLN(msg);
  }
  else {
    if (dir) {
      wav_increase_volume();
    }
    else {
      wav_decrease_volume();
    }
    uint8_t volume = wav_get_volume();
    sprintf(msg, "Volume -> %u %%", volume);
    LOGLN(msg);
  }
}
