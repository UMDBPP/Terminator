#include "fram.h"

#include "stm32l412xx.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_spi.h"

int fram_wren(fram_t *dev);
int fram_wrdi(fram_t *dev);
int fram_rdsr(fram_t *dev);
int fram_wrsr(fram_t *dev);
int fram_get_id(fram_t *dev);


int fram_init(fram_t *dev, SPI_TypeDef *SPIx, uint8_t cs_pin, uint8_t sck_pin,
              uint8_t mosi_pin, uint8_t miso_pin) {
    dev->spi_device = SPIx;

    dev->cs_pin = cs_pin;
    dev->sck_pin = sck_pin;
    dev->mosi_pin = mosi_pin;
    dev->miso_pin = miso_pin;

    dev->wp_pin = 255;
    dev->hold_pin = 255;

	// TODO actually make this driver use the given CS pin
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);

    fram_get_id(dev);

    return 0;
}

int fram_get_id(fram_t *dev) {
    uint8_t cmd = RDID_CMD;

    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);

    spi_write(dev->spi_device, &cmd, 1);

    spi_read(dev->spi_device, (uint8_t *)&(dev->device_id), 4);

    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);

    return 0;
}

// TODO actually check for errors
// Set write enable latch (enable writes)
int fram_wren(fram_t *dev) {
	uint8_t cmd = WREN_CMD;

    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);

    spi_write(dev->spi_device, &cmd, 1);

    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);

	return 0;
}

// Reset write enable latch (disable writes)
int fram_wrdi(fram_t *dev) {
	uint8_t cmd = WRDI_CMD;

    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);

    spi_write(dev->spi_device, &cmd, 1);

    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);
}

int fram_rdsr(fram_t *dev) {
	// TODO
	return -1;
}

int fram_wrsr(fram_t *dev) {
	// TODO
	return -1;
}

int fram_write(fram_t *dev, SPI_TypeDef *SPIx, uint32_t addr, uint8_t *buf, uint32_t num_bytes) {
	uint8_t cmd = WRITE_CMD;
	
	uint8_t addr_byte_1 = (addr >> 8) & 0xFF;
	uint8_t addr_byte_2 = addr & 0xFF;

	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);

    spi_write(dev->spi_device, &cmd, 1);

	spi_write(dev->spi_device, &addr_byte_1, 1);
	spi_write(dev->spi_device, &addr_byte_2, 1);

	spi_write(dev->spi_device, &buf, num_bytes);

    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);

	return 0;
}

int fram_read(fram_t *dev, SPI_TypeDef *SPIx, uint32_t addr, uint8_t *buf, uint32_t num_bytes) {
	uint8_t cmd = READ_CMD;
	
	uint8_t addr_byte_1 = (addr >> 8) & 0xFF;
	uint8_t addr_byte_2 = addr & 0xFF;

	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);

    spi_write(dev->spi_device, &cmd, 1);

	spi_write(dev->spi_device, &addr_byte_1, 1);
	spi_write(dev->spi_device, &addr_byte_2, 1);

	spi_read(dev->spi_device, &buf, num_bytes);

    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);
	
	return 0;
}
