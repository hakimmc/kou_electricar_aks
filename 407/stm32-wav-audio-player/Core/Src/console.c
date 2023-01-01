/*
 * console.c
 *
 *  Created on: 18 May 2021
 *      Author: david
 */

#include "serial.h"
#include <string.h>
#include <stdlib.h>

#include "wav_player.h"

#define LOG(val) serial_print(console, val)
#define LOGLN(val) serial_println(console, val)

extern serial_t *console;

static char cmd[48];
static unsigned idx;

static void display_help(void) {
	LOGLN("Available commands:");
	LOGLN("\thelp - Display this help menu");
	LOGLN("\tplay noise - Play noise");
	LOGLN("\tpause - Pause playback");
	LOGLN("\tresume - Resume playback");
	LOGLN("\tstop - Stop playback");
	LOGLN("\tdump track info - Dump info on all audio tracks");
	LOGLN("\tplay track <index> - Play track at index <index> (starting from 1)");
	LOGLN("\tset volume <0-100> - Set master volume in per-cent");
}

static void process_command(void) {
	if (!strncmp(cmd, "help", 4)) {
		display_help();
	}
	else if (!strncmp(cmd, "play noise", 10)) {
	  LOGLN("Playing noise ...");
	  wav_play_noise();
	}
	else if (!strncmp(cmd, "stop", 4)) {
	  LOGLN("Stopping playback ...");
	  wav_stop();
	}
	else if (!strncmp(cmd, "pause", 5)) {
	  LOGLN("Pausing playback ...");
	  wav_pause();
	}
	else if (!strncmp(cmd, "resume", 6)) {
	  LOGLN("Resuming playback ...");
	  wav_resume();
	}
	else if (!strncmp(cmd, "dump track info", 15)) {
	  dump_all_tracks_info();
	}
	else if (!strncmp(cmd, "play track", 10)) {
	  char *p = cmd + 11;
	  if (!p) {
	    LOGLN("You must specify the track index.");
	    return;
	  }
	  uint8_t idx = atoi(p);
	  LOGLN("Trying to play track ...");
	  wav_play(idx);
	}
	else if (!strncmp(cmd, "set volume", 10)) {
	  char *p = cmd + 11;
	  if (!p) {
	    LOGLN("You must specify the volume in dB");
	    return;
	  }
	  uint8_t percent = atoi(p);
	  LOGLN("Trying to set master volume ...");
	  if (wav_set_volume(percent) != 0) {
	    LOGLN("Failed.");
	  }
	  else {
	    LOGLN("Okay.");
	  }
	}
	else {
	  LOG("Did not understand command: ");
	  LOGLN(cmd);
	}
}

static void process_data(const char *data, int len) {
	while (len--) {
		char ch = *(data++);
		if (ch == '\r') {
			cmd[idx] = '\0';
			idx = 0;
			process_command();
		}
		else if (ch == '\n') {} // ignore
		else {
			cmd[idx++] = ch;
			if (idx >= sizeof(cmd)-1)
				idx = 0;
		}
	}
}

void console_monitor(void) {
	char buf[16];

	int avail = serial_available(console);

	if (avail > 0) {
		size_t max = sizeof(buf);
		if (avail > max)
			avail = max;
		int count = serial_read_bytes(console, buf, count);
		if (count > 0)
			process_data(buf, count);
	}
}

