/*
 * cs43l22.c
 *
 *  Created on: 7 Aug 2019
 *      Author: david
 */

#include "cs43l22.h"
#include "cs43l22_i2c_st_hal.h"

#include <ctype.h>
#include <stdio.h>
#include "main.h"

#define max_volume_db 12
#define min_volume_db -102
#define volume_range_db (max_volume_db - min_volume_db)

static struct cs43l22_ctx cs43l22;

int (*i2c_read_reg)(uint8_t addr, uint8_t *val) = st_hal_i2c_read_reg;
int (*i2c_write_reg)(uint8_t addr, uint8_t val) = st_hal_i2c_write_reg;

int cs43l22_init(void *i2c) {
	uint8_t reg_val = 0x00;

	st_hal_i2c_init(i2c);

	LL_GPIO_SetOutputPin(Audio_RST_GPIO_Port, Audio_RST_Pin);
	HAL_Delay(1000);

	if (i2c_read_reg(DAC_REG_ID, &reg_val) != 0) {
		return 1;
	}

	cs43l22.chip_id = (reg_val >> 3) & 0x1f;
	cs43l22.rev_id = reg_val & 0x07;

	// Register settings should be loaded while the
	// (audio processing section of the) chip is powered down. (Obviously
	// the control interface will remain powered up.)
	i2c_write_reg(DAC_REG_POWER_CTRL1, 0x01);

	// Disable all audio output (headphones and speakers) initially ...
	i2c_write_reg(DAC_REG_POWER_CTRL2, 0xFF);

	// Enable auto-detection of clock speed
	i2c_write_reg(DAC_REG_CLOCK_CTRL, 0x80);

	// Now that register settings are loaded, we can re-apply power.
	i2c_write_reg(DAC_REG_POWER_CTRL1, 0x9E);

	return 0;
}

int cs43l22_dac_power_on(void) {
	return i2c_write_reg(DAC_REG_POWER_CTRL1, 0x9E);
}

int cs43l22_dac_power_off(void) {
	return i2c_write_reg(DAC_REG_POWER_CTRL1, 0x01);
}

int cs43l22_set_audio_format(cs43l22_dac_format_t format) {
	uint8_t reg = 0;

	if (i2c_read_reg(DAC_REG_INTERFACE_CTRL_1, &reg) != 0) {
		return 1;
	}
	reg &= 0xf3;
	reg |= format << 2;

	return i2c_write_reg(DAC_REG_INTERFACE_CTRL_1, reg);
}

int cs43l22_set_audio_word_length(cs43l22_audio_word_length_t length) {
	uint8_t reg = 0;

	if (i2c_read_reg(DAC_REG_INTERFACE_CTRL_1, &reg) != 0) {
		return 1;
	}
	reg &= 0xfc;
	reg |= length;

	return i2c_write_reg(DAC_REG_INTERFACE_CTRL_1, reg);
}

int cs43l22_mute_headphones(void) {
	uint8_t reg = 0;
	if (i2c_read_reg(DAC_REG_PLAYBACK_CTRL_2, &reg) != 0) {
		return 1;
	}
	reg &= 0x3f;
	reg |= 0xc0;

	return i2c_write_reg(DAC_REG_PLAYBACK_CTRL_2, reg);
}

int cs43l22_unmute_headphones(void) {
	uint8_t reg = 0;
	if (i2c_read_reg(DAC_REG_PLAYBACK_CTRL_2, &reg) != 0) {
		return 1;
	}
	reg &= 0x3f;

	return i2c_write_reg(DAC_REG_PLAYBACK_CTRL_2, reg);
}

int cs43l22_enable_headphones(void) {
	uint8_t reg_val = 0;
	if (i2c_read_reg(DAC_REG_POWER_CTRL2, &reg_val) != 0) {
		return 1;
	}

	reg_val &= 0x0f;
	reg_val |= 0xa0;

	return i2c_write_reg(DAC_REG_POWER_CTRL2, reg_val);
}

int cs43l22_disable_headphones(void) {
	uint8_t reg_val = 0;
	if (i2c_read_reg(DAC_REG_POWER_CTRL2, &reg_val) != 0) {
		return 1;
	}

	reg_val |= 0xf0;

	return i2c_write_reg(DAC_REG_POWER_CTRL2, reg_val);
}

void cs43l22_get_info(char *info) {
	sprintf(info, "Audio DAC info. Chip ID: 0x%02x. Revision ID: 0x%02x\r\n",
			cs43l22.chip_id, cs43l22.rev_id);
}

/**
 * Set the DAC master volume
 *
 * This is specified in increments of .5 dB.
 *
 * A value of 0 = 0dB.
 */
int cs43l22_set_master_volume(uint8_t volume) {
	if (i2c_write_reg(DAC_REG_MASTER_VOLUME_A, volume) != 0) {
		return 1;
	}
	return i2c_write_reg(DAC_REG_MASTER_VOLUME_B, volume);
}

int cs43l22_set_master_volume_db(int8_t db) {
  if (db < min_volume_db)
    db = min_volume_db;
  if (db > max_volume_db)
    db = max_volume_db;

  uint8_t reg = 2*db;
  return cs43l22_set_master_volume(reg);
}

int cs43l22_set_master_volume_percent(uint8_t percent) {
  if (percent > 100)
    percent = 100;
  int8_t db = min_volume_db + (percent * volume_range_db) / 100;
  cs43l22_set_master_volume_db(db);
}

uint8_t cs43l22_get_master_volume(void) {
  uint8_t val = 0;
  i2c_read_reg(DAC_REG_MASTER_VOLUME_A, &val);
  return val;
}

int8_t cs43l22_get_master_volume_db(void) {
  int8_t db;

  uint8_t reg = cs43l22_get_master_volume();

  if (reg <= 24)
    db = reg / 2;
  else if (reg >= 25 && reg <= 52)
    db = min_volume_db;
  else {
    int16_t tmp = reg - 256;
    db = tmp / 2;
  }

  return db;
}

uint8_t cs43l22_get_master_volume_percent(void) {
  int8_t db = cs43l22_get_master_volume_db();
  int8_t db_offset = db - min_volume_db;

  int16_t tmp = db_offset * 100;
  uint8_t percent = tmp / volume_range_db;
  if (tmp % volume_range_db != 0)
    percent++; // avoid rounding down

  return percent;
}

static uint8_t letter_to_note(int l) {
	int c = tolower(l);
	if (!(c >= 'a' && c <= 'g')) {
		return 0;
	}

	if (c < 'c') {
		return c - 'a' + 5;
	}
	return c - 'a' - 2;
}

dac_beep_frequency_t cs43l22_string_to_frequency(const char *desc) {
	uint8_t note = letter_to_note(desc[0]);
	uint8_t octave = desc[1]-48;
	if (octave <= 4) {
		return DAC_BEEP_FREQUENCY_C4;
	}
	octave -= 5;

	dac_beep_frequency_t frequency = DAC_BEEP_FREQUENCY_C5;
	frequency += octave*7;
	frequency += note;

	return frequency;
}

int cs43l22_beep_set_frequency(dac_beep_frequency_t frequency) {
	uint8_t reg = 0;
	if (i2c_read_reg(DAC_REG_BEEP_FREQ, &reg) != 0) {
		return 1;
	}

	reg &= 0x0f;
	reg |= frequency << 4;

	return i2c_write_reg(DAC_REG_BEEP_FREQ, reg);
}

int cs43l22_beep_continuous(void) {
	uint8_t reg = 0x20 | DAC_BEEP_CONTINUOUS << 6;
	return i2c_write_reg(DAC_REG_BEEP_CFG, reg);
}

int cs43l22_beep_single(dac_beep_frequency_t frequency) {
	// Need to first of all clear the beep state.
	cs43l22_beep_off();
	// Then set the frequency and trigger the beep.
	cs43l22_beep_set_frequency(frequency);
	uint8_t reg = 0x20 | DAC_BEEP_SINGLE << 6;
	return i2c_write_reg(DAC_REG_BEEP_CFG, reg);
}

int cs43l22_beep_off(void) {
	uint8_t reg = 0;

	if (i2c_read_reg(DAC_REG_BEEP_CFG, &reg) != 0) {
		return 1;
	}

	reg &= 0x3f;

	return i2c_write_reg(DAC_REG_BEEP_CFG, reg);
}
