/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "stm32l412xx.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_crs.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_spi.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_utils.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

__STATIC_INLINE void setup_spi(SPI_TypeDef *spix,
                               LL_SPI_InitTypeDef *spix_init) {
    //  set up SPI2 device
    LL_SPI_StructInit(spix_init);
    LL_SPI_Init(spix, spix_init);

    LL_SPI_SetTransferDirection(spix, LL_SPI_FULL_DUPLEX);
    LL_SPI_SetMode(spix, LL_SPI_MODE_MASTER);
    LL_SPI_SetDataWidth(spix, LL_SPI_DATAWIDTH_8BIT);
    LL_SPI_SetClockPolarity(spix, LL_SPI_POLARITY_LOW);
    LL_SPI_SetClockPhase(spix, LL_SPI_PHASE_1EDGE);
    LL_SPI_SetNSSMode(spix, LL_SPI_NSS_SOFT);
    LL_SPI_SetBaudRatePrescaler(spix, LL_SPI_BAUDRATEPRESCALER_DIV4);
    LL_SPI_SetTransferBitOrder(spix, LL_SPI_MSB_FIRST);
    LL_SPI_DisableCRC(spix);
}

void spi_read(SPI_TypeDef *spix, uint8_t *const buf,
                              uint32_t num_bytes);

void spi_write(SPI_TypeDef *spix, const uint8_t *const buf,
                               uint32_t num_bytes);

/* Private defines -----------------------------------------------------------*/
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0                                     \
    ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority, \
                                4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1                                     \
    ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority, \
                                3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2                                     \
    ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority, \
                                2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3                                     \
    ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority, \
                                1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4                                     \
    ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority, \
                                0 bit  for subpriority */
#endif

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
