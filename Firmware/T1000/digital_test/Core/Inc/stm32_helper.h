#ifndef _STM32_HELPER_H
#define _STM32_HELPER_H
 
#include <stdint.h>

#include "stm32l4xx_ll_adc.h"
#include "stm32l4xx_ll_comp.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_i2c.h"
#include "stm32l4xx_ll_crs.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_spi.h"
#include "stm32l4xx_ll_tim.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_lptim.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_iwdg.h"

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

void spi_write(SPI_TypeDef *spix, const uint8_t *const buf,
               uint32_t num_bytes);



// Auto generated (mostly) peripheral init functions
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
void MX_COMP1_Init(void);
void MX_I2C1_Init(void);
void MX_SPI1_Init(void);
void MX_TIM1_Init(void);
void MX_LPTIM1_Init(void);
void MX_USART2_UART_Init(void);
void MX_IWDG_Init(void);

#endif // _STM32_HELPER_H
