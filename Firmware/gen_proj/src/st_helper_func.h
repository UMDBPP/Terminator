#include <stdint.h>

#include "stm32l4xx_ll_gpio.h"

#define GPIO_LOW 0
#define GPIO_HIGH 1

void set_gpio_pin(GPIO_TypeDef *gpio_bank, uint32_t pin, uint32_t value);
uint32_t gpio_int_to_mask(uint32_t pin);