/*
 * cs43l22.h
 *
 *  Created on: 7 Aug 2019
 *      Author: david
 */

#ifndef CS43L22_H_
#define CS43L22_H_

#include <stdint.h>

typedef enum {
  DAC_REG_ID = 0x01,
  DAC_REG_POWER_CTRL1 = 0x02,
  DAC_REG_POWER_CTRL2 = 0x04,
  DAC_REG_CLOCK_CTRL = 0x05,
  DAC_REG_INTERFACE_CTRL_1 = 0x06,
  DAC_REG_PLAYBACK_CTRL_1 = 0x0d,
  DAC_REG_PLAYBACK_CTRL_2 = 0x0f,
  DAC_REG_BEEP_FREQ = 0x1c,
  DAC_REG_BEEP_VOL = 0x1d,
  DAC_REG_BEEP_CFG = 0x1e,
  DAC_REG_MASTER_VOLUME_A = 0x20,
  DAC_REG_MASTER_VOLUME_B = 0x21,
  DAC_REG_STATUS = 0x2e
} dac_i2c_reg_t;

typedef enum {
  DAC_FORMAT_LEFT_JUSTIFIED, DAC_FORMAT_I2S, DAC_FORMAT_RIGHT_JUSTIFIED
} cs43l22_dac_format_t;

typedef enum {
  DAC_AUDIO_WORD_LENGTH_24,
  DAC_AUDIO_WORD_LENGTH_20,
  DAC_AUDIO_WORD_LENGTH_18,
  DAC_AUDIO_WORD_LENGTH_16
} cs43l22_audio_word_length_t;

// Beep / Tone generator

typedef enum {
  DAC_BEEP_FREQUENCY_C4,
  DAC_BEEP_FREQUENCY_C5,
  DAC_BEEP_FREQUENCY_D5,
  DAC_BEEP_FREQUENCY_E5,
  DAC_BEEP_FREQUENCY_F5,
  DAC_BEEP_FREQUENCY_G5,
  DAC_BEEP_FREQUENCY_A5,
  DAC_BEEP_FREQUENCY_B5,
  DAC_BEEP_FREQUENCY_C6,
  DAC_BEEP_FREQUENCY_D6,
  DAC_BEEP_FREQUENCY_E6,
  DAC_BEEP_FREQUENCY_F6,
  DAC_BEEP_FREQUENCY_G6,
  DAC_BEEP_FREQUENCY_A6,
  DAC_BEEP_FREQUENCY_B6,
  DAC_BEEP_FREQUENCY_C7
} dac_beep_frequency_t;

typedef enum {
  DAC_BEEP_OFF, DAC_BEEP_SINGLE, DAC_BEEP_MULTIPLE, DAC_BEEP_CONTINUOUS
} cs43l22_beep_mode_t;

struct cs43l22_ctx {
  uint8_t chip_id;
  uint8_t rev_id;
};

int cs43l22_init(void *i2c);

int cs43l22_dac_power_on(void);
int cs43l22_dac_power_off(void);

int cs43l22_set_audio_format(cs43l22_dac_format_t format);
int cs43l22_set_audio_word_length(cs43l22_audio_word_length_t length);

int cs43l22_disable_headphones(void);
int cs43l22_enable_headphones(void);

// Volume control
int cs43l22_set_master_volume(uint8_t volume);
int cs43l22_set_master_volume_db(int8_t db);
int cs43l22_set_master_volume_percent(uint8_t percent);
uint8_t cs43l22_get_master_volume(void);
int8_t cs43l22_get_master_volume_db(void);
uint8_t cs43l22_get_master_volume_percent(void);

// Miscellaneous ...
void cs43l22_get_info(char *);

dac_beep_frequency_t cs43l22_string_to_frequency(const char *desc);

int cs43l22_beep_set_frequency(dac_beep_frequency_t frequency);
int cs43l22_beep_single(dac_beep_frequency_t frequency);
int cs43l22_beep_continuous(void);
int cs43l22_beep_off(void);

#endif /* CS43L22_H_ */
