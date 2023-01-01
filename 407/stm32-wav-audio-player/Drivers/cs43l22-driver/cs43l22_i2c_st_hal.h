/*
 * cs43l22_i2c_st_hal.h
 *
 *  Created on: Apr 21, 2020
 *      Author: david
 */

#ifndef CS43L22_CS43L22_I2C_ST_HAL_H_
#define CS43L22_CS43L22_I2C_ST_HAL_H_

void st_hal_i2c_init(void *i2c);
int st_hal_i2c_read_reg(uint8_t addr, uint8_t *val);
int st_hal_i2c_write_reg(uint8_t addr, uint8_t val);

#endif /* CS43L22_CS43L22_I2C_ST_HAL_H_ */
