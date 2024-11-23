#ifndef _MS5607_H
#define _MS5607_H

#include "stm32l4xx_ll_i2c.h"

#define MS5607_ADDR 0x77
#define MS5607_RESET_CMD 0x1E

#define MS5607_READ_C1_CMD 0xA2
#define MS5607_READ_C2_CMD 0xA4
#define MS5607_READ_C3_CMD 0xA6
#define MS5607_READ_C4_CMD 0xA8
#define MS5607_READ_C5_CMD 0xAA
#define MS5607_READ_C6_CMD 0xAC
#define MS5607_READ_CRC_CMD 0xAE

#define MS5607_PRESS_CONV_CMD 0x48 // OSR = 4096
#define MS5607_TEMP_CONV_CMD 0x58  // OSR = 4096
#define MS5607_READ_CMD 0x00

int ms5607_init(I2C_TypeDef *i2cx);
int ms5607_get_press_temp(uint32_t *pressure, int32_t *temperature);
int ms5607_reset(void);

#endif // _MS5607_H
