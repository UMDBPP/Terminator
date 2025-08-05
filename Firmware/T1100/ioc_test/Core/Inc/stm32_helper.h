#ifndef _STM32_HELPER_H
#define _STM32_HELPER_H

#include <stdint.h>

#include "stm32wlxx_ll_adc.h"
//#include "stm32wlxx_ll_comp.h"
#include "stm32wlxx_ll_exti.h"
#include "stm32wlxx_ll_i2c.h"
//#include "stm32wlxx_ll_crs.h"
#include "stm32wlxx_ll_rcc.h"
#include "stm32wlxx_ll_bus.h"
#include "stm32wlxx_ll_system.h"
#include "stm32wlxx_ll_cortex.h"
#include "stm32wlxx_ll_utils.h"
#include "stm32wlxx_ll_pwr.h"
#include "stm32wlxx_ll_dma.h"
#include "stm32wlxx_ll_tim.h"
#include "stm32wlxx_ll_gpio.h"
#include "stm32wlxx_ll_lptim.h"
#include "stm32wlxx_ll_lpuart.h"
//#include "stm32wlxx_ll_wwdg.h"

#define PA8_HIGH LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);
#define PA8_LOW LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);
#define PA8_TOGGLE LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_8);
#define PA6_HIGH LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);
#define PA6_LOW LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);

typedef struct _gpio_t {
	GPIO_TypeDef *port;
	uint32_t pin;
} gpio_t;

void gpio_put(gpio_t pin, uint8_t value);

void spi_read(SPI_TypeDef *spix, uint8_t *const buf, uint32_t num_bytes);
void spi_read_write(SPI_TypeDef *spix, uint8_t *const tx_buf,
		uint8_t *const rx_buf, uint32_t num_bytes);

void spi_write(SPI_TypeDef *spix, const uint8_t *const buf, uint32_t num_bytes);

#endif // _STM32_HELPER_H
