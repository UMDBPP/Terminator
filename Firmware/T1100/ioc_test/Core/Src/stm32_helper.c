#include "stm32_helper.h"
#include <stdlib.h>
#include <string.h>

void gpio_put(gpio_t pin, uint8_t value) {
	if (value)
		LL_GPIO_SetOutputPin(pin.port, pin.pin);
	else
		LL_GPIO_ResetOutputPin(pin.port, pin.pin);
}
