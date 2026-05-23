/*
 * i2c_driver.h
 *
 *  Created on: May 18, 2026
 *      Author: xushu
 */

#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_

#include <stdint.h>

void i2c1_init(void);
void i2c1_write_reg(uint8_t slave_addr, uint8_t reg_addr, uint8_t data);
void i2c1_read_regs(uint8_t slave, uint8_t reg, uint8_t *buf, uint8_t len);

#endif /* I2C_DRIVER_H_ */
