/*
 * wav_parser.h
 *
 *  Created on: 10 May 2021
 *      Author: david
 */

#ifndef WAV_PARSER_WAV_PARSER_H_
#define WAV_PARSER_WAV_PARSER_H_

#include <stdint.h>

#define WAV_HEADER_LEN 44

typedef struct {
  uint8_t type;
  uint16_t sample_rate;
  uint8_t bit_depth;
  uint8_t n_channels;
  uint32_t data_size_bytes;
} audio_format_t;

int parse_wav_header(const uint8_t *header, audio_format_t *format);

#endif /* WAV_PARSER_WAV_PARSER_H_ */
