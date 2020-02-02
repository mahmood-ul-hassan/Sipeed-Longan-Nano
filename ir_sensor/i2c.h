/*!
    \file  i2c.h
    \brief the header file of I2C
    
    \version 2019-06-05, V1.0.0, demo for GD32VF103
*/
#ifndef I2C_H
#define I2C_H

#include "gd32vf103.h"
#include "systick.h"

#define I2C0_SPEED              400000	// AMG8833 sensor
#define I2C1_SPEED              100000	// MLX90614 sensor (It doesnt work at 400k I2C clock)

#define I2C0_MASTER_ADDRESS7    0x99
#define I2C1_MASTER_ADDRESS7    0x99

/* configure the GPIO ports */
void gpio_config(uint32_t i2c_periph);
/* configure the I2C interfaces */
void i2c_config(uint32_t i2c_periph);

uint8_t i2c_timeout(uint64_t start_i2c_time, uint32_t timeout);
uint8_t i2c_read(uint32_t i2c_periph, uint8_t slave_address, uint8_t read_address, uint8_t* p_buffer, uint8_t number_of_byte, uint32_t timeout);
uint8_t i2c_write(uint32_t i2c_periph, uint8_t slave_address, uint8_t write_address, uint8_t* p_buffer, uint8_t number_of_byte, uint32_t timeout);

#endif  /* I2C_H */
