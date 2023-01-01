/*
 * rotary_encoder.c
 *
 *  Created on: May 18, 2021
 *      Author: david
 */
#include <stdint.h>

#include "main.h"
#include "stm32f4xx_ll_gpio.h"

#define POLL_INTERVAL_MS 3

static volatile int16_t pos;

static uint32_t last_a_state = 1;
static uint32_t last_sw_state = 1;
static uint32_t t_last_poll;

static void (*switch_callback)(uint8_t dir);
static void (*rotation_callback)(uint8_t dir, int16_t pos);

void encoder_set_switch_callback(void (*cb)(void)) {
  switch_callback = cb;
}

void encoder_set_rotation_callback(void (*cb)(uint8_t, int16_t)) {
  rotation_callback = cb;
}

const int16_t encoder_get_position(void) {
  return pos;
}

void encoder_reset_position(void) {
  pos = 0;
}

void encoder_monitor(void) {
  uint32_t now = HAL_GetTick();
  if (now - t_last_poll < POLL_INTERVAL_MS) {
    return;
  }

  t_last_poll = now;

  uint32_t sw_state = LL_GPIO_IsInputPinSet(ROTARY_SW_GPIO_Port, ROTARY_SW_Pin);
  if (sw_state != last_sw_state) {
    last_sw_state = sw_state;
    if (switch_callback) {
      switch_callback(sw_state);
    }
  }

  uint32_t a_state = LL_GPIO_IsInputPinSet(ROTARY_CLK_GPIO_Port, ROTARY_CLK_Pin);
  if (a_state != last_a_state) {
    last_a_state = a_state;
    if (a_state) { // rising edge
      uint32_t b_state = LL_GPIO_IsInputPinSet(ROTARY_DAT_GPIO_Port, ROTARY_DAT_Pin);
      uint8_t dir;
      if (b_state == 0) {
        dir = 1;
        pos++;
      }
      else {
        dir = 0;
        pos--;
      }
      if (rotation_callback) {
        rotation_callback(dir, pos);
      }
    }
  }
}

