/*
 * wav_parser.c
 *
 *  Created on: 10 May 2021
 *      Author: david
 */

#include "wav_parser.h"
#include <string.h>

int parse_wav_header(const uint8_t *header, audio_format_t *format)
{
  const uint8_t *p = header;
  if (strncmp(p, "RIFF", 4) != 0) {
    return 1;
  }

  p = header + 8;
  if (strncmp(p, "WAVE", 4) != 0) {
    return 1;
  }
  p = header + 12;
  if (strncmp(p, "fmt", 3) != 0) {
    return 1;
  }

  // Type (PCM, ?)
  p = header + 20;
  format->type = p[0] | (p[1] << 8);

  // Channel count ...
  p = header + 22;
  format->n_channels = p[0] | (p[1] << 8);

  // Sample rate ...
  p = header + 24;
  format->sample_rate = 0;
  for (int i = 0; i < 4; i++) {
    format->sample_rate |= p[i] << (8 * i);
  }

  // Bit depth / bits per sample
  p = header + 34;
  format->bit_depth = p[0] | (p[1] << 8);

  p = header + 36;
  if (strncmp(p, "data", 4) != 0) {
    return 1;
  }

  p = header + 40;
  format->data_size_bytes = 0;
  for (int i = 0; i < 4; i++) {
    format->data_size_bytes |= p[i] << (8 * i);
  }

  return 0;
}

