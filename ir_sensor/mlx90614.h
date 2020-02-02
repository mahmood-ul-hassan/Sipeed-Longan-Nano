/*!
    \file  MLX90614.h
    \brief the header file of MLX90614
    
    \version 2019-06-05, V1.0.0, demo for GD32VF103
*/

#ifndef MLX90614_H
#define MLX90614_H

#include "gd32vf103.h"

#define MLX90614_I2CPORT I2C1
#define MLX90614_I2CADDR 0x5A<<1

// RAM
#define MLX90614_RAWIR1 0x04
#define MLX90614_RAWIR2 0x05
#define MLX90614_TA 0x06
#define MLX90614_TOBJ1 0x07
#define MLX90614_TOBJ2 0x08
// EEPROM
#define MLX90614_TOMAX 0x20
#define MLX90614_TOMIN 0x21
#define MLX90614_PWMCTRL 0x22
#define MLX90614_TARANGE 0x23
#define MLX90614_EMISS 0x24
#define MLX90614_CONFIG 0x25
#define MLX90614_ADDR 0x0E
#define MLX90614_ID1 0x3C
#define MLX90614_ID2 0x3D
#define MLX90614_ID3 0x3E
#define MLX90614_ID4 0x3F


/* initialize IR sensor */
uint8_t ir_init();

double ir_readObjectTempC(void);
double ir_readAmbientTempC(void);
double ir_readObjectTempF(void);
double ir_readAmbientTempF(void);

#endif  /* MLX90614_H */
