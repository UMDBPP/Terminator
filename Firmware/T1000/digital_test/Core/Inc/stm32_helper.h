#include <stdint.h>
#include "stm32l4xx_ll_spi.h"
#include "stm32l4xx_ll_i2c.h"

void spi_read(SPI_TypeDef *spix, uint8_t *const buf, uint32_t num_bytes);

void spi_write(SPI_TypeDef *spix, const uint8_t *const buf,
               uint32_t num_bytes);

uint8_t SPI_TxRx(SPI_TypeDef *spix, uint8_t data);

int8_t I2C_MasterRx(uint8_t devAddr, uint8_t *buffer, uint8_t len,
                    uint16_t ms);
