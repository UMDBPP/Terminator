#include "st_helper_func.h"

#include <stdint.h>

#include "stm32l4xx_ll_gpio.h"

void set_gpio_pin(GPIO_TypeDef *gpio_bank, uint32_t pin, uint32_t value) {
    pin = gpio_int_to_mask(pin);

    switch (value) {
        case GPIO_LOW:
            LL_GPIO_ResetOutputPin(gpio_bank, pin);
            return;
        default:
            LL_GPIO_SetOutputPin(gpio_bank, pin);
            return;
    }
}

uint32_t gpio_int_to_mask(uint32_t pin) {
    switch (pin) {
        case 0:
            return LL_GPIO_PIN_0;
        case 1:
            return LL_GPIO_PIN_1;
        case 2:
            return LL_GPIO_PIN_2;
        case 3:
            return LL_GPIO_PIN_3;
        case 4:
            return LL_GPIO_PIN_4;
        case 5:
            return LL_GPIO_PIN_5;
        case 6:
            return LL_GPIO_PIN_6;
        case 7:
            return LL_GPIO_PIN_7;
        case 8:
            return LL_GPIO_PIN_8;
        case 9:
            return LL_GPIO_PIN_9;
        case 10:
            return LL_GPIO_PIN_10;
        case 11:
            return LL_GPIO_PIN_11;
        case 12:
            return LL_GPIO_PIN_12;
        case 13:
            return LL_GPIO_PIN_13;
        case 14:
            return LL_GPIO_PIN_14;
        case 15:
            return LL_GPIO_PIN_15;
    }
}