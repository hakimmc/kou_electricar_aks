/*
 * cs43l22_i2c.c
 *
 *  Created on: Apr 21, 2020
 *      Author: david
 */
#include "stm32f4xx_hal.h"
#include <stdint.h>

#define CS43L22_I2C_ADDR 0x94

static I2C_HandleTypeDef *hi2c;

void st_hal_i2c_init(void *i2c) {
	hi2c = i2c;
}

int st_hal_i2c_read_reg(uint8_t addr, uint8_t *val) {
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, CS43L22_I2C_ADDR,
			(uint8_t[] ) { addr }, 1, 1000);
	if (status != HAL_OK) {
		return 1;
	}

	status = HAL_I2C_Master_Receive(hi2c, CS43L22_I2C_ADDR, val, 1, 1000);
	if (status != HAL_OK) {
		return 1;
	}

	return 0;
}

int st_hal_i2c_write_reg(uint8_t addr, uint8_t val) {
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, CS43L22_I2C_ADDR,
			(uint8_t[] ) { addr, val }, 2, 1000);
	return status == HAL_OK ? 0 : 1;
}
