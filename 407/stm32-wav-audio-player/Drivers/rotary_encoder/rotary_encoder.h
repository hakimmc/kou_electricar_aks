/*
 * rotary_encoder.h
 *
 *  Created on: May 18, 2021
 *      Author: david
 */

#ifndef INC_ROTARY_ENCODER_H_
#define INC_ROTARY_ENCODER_H_

#include <stdint.h>

void encoder_set_switch_callback(void (*cb)(void));
void encoder_set_rotation_callback(void (*cb)(uint8_t, int16_t));
const int16_t encoder_get_position(void);

void encoder_reset_position(void);
void encoder_monitor(void);

#endif /* INC_ROTARY_ENCODER_H_ */
