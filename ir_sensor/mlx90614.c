/*!
    \file  MLX90614.c
    \brief the read and write function file
    
    \version 2019-06-05, V1.0.0, demo for GD32VF103
*/

#include "ir_sensor/MLX90614.h"
#include "ir_sensor/i2c.h"
#include <stdint.h>
#include <stdio.h>
#include "systick.h"

float ir_readTemp(uint8_t reg);
uint16_t ir_read16(uint8_t addr);

/*!
    \brief      initialize peripherals used by the I2C IR Sensor
    \param[in]  none
    \param[out] none
    \retval     1
*/
uint8_t ir_init(void) {
  i2c_config(MLX90614_I2CPORT);
  return 1;
}

double ir_readObjectTempF(void) {
  return (ir_readTemp(MLX90614_TOBJ1) * 9.0 / 5.0) + 32.0;
}

double ir_readAmbientTempF(void) {
  return (ir_readTemp(MLX90614_TA) * 9.0 / 5.0) + 32.0;
}

double ir_readObjectTempC(void) {
  return ir_readTemp(MLX90614_TOBJ1);
}

double ir_readAmbientTempC(void) {
  return ir_readTemp(MLX90614_TA);
}

float ir_readTemp(uint8_t reg) {
  float temp;
  
  temp = ir_read16(reg);
  temp *= .02;
  temp  -= 273.15;
  return temp;
}

/*!
    \brief      I2C read 16bit function
    \param[in]  Address
    \param[out] none
    \retval     16bit read Value 
*/
uint16_t ir_read16(uint8_t addr) {
  uint16_t ret=0;
  uint8_t p_buffer[2];
  i2c_read(MLX90614_I2CPORT, MLX90614_I2CADDR, addr, p_buffer, 2, 1000);
  ret=p_buffer[0]|(p_buffer[1]<<8);
  return ret;
}
