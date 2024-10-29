#ifndef _MS5607_H
#define _MS5607_H

#include "stm32l4xx_ll_i2c.h"

int ms5607_init(I2C_TypeDef *i2cx);
int ms5607_get_press_temp(uint32_t double *pressure, int32_t *temperature);
int ms5607_reset(void);

#endif // _MS5607_H
