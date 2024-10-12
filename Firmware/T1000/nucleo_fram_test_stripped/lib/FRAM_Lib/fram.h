#ifndef FRAM_H
#define FRAM_H

#include <stdint.h>

#include "stm32l4xx_ll_spi.h"
#include "lfs.h"

#define WPEN_MASK 0x80
#define BP1_MASK 0x08
#define BP0_MASK 0x04
#define WEL_MASK 0x02
#define MAX_ADDR 131072

#define WREN_CMD 0x06
#define WRDI_CMD 0x04
#define RDSR_CMD 0x05
#define WRSR_CMD 0x01
#define READ_CMD 0x03
#define WRITE_CMD 0x02
#define RDID_CMD 0x9F

typedef struct fram {
    SPI_TypeDef *spi_device;
    uint8_t cs_pin;
    uint8_t sck_pin;
    uint8_t mosi_pin;
    uint8_t miso_pin;
    uint8_t wp_pin;    // write protect pin for status register, protected when
                       // low
    uint8_t hold_pin;  // hold pin for serial transfer, holds when low

    uint8_t wpen;
    uint8_t bp1;
    uint8_t bp0;
    uint8_t wel;
    uint32_t device_id;
} fram_t;

typedef enum { PB6 } pin;

int fram_init(fram_t *dev, SPI_TypeDef *SPIx, uint8_t cs_pin, uint8_t sck_pin,
              uint8_t mosi_pin, uint8_t miso_pin);

int fram_write(fram_t *dev, SPI_TypeDef *SPIx, uint32_t addr, uint8_t *buf, uint32_t num_bytes);

int fram_read(fram_t *dev, SPI_TypeDef *SPIx, uint32_t addr, uint8_t *buf, uint32_t num_bytes);

int fs_flash_read(const struct lfs_config *cfg, lfs_block_t block,
        lfs_off_t off, void *buffer, lfs_size_t size);

int fs_flash_prog(const struct lfs_config *cfg, lfs_block_t block,
            lfs_off_t off, const void *buffer, lfs_size_t size);

int fs_flash_erase(const struct lfs_config *cfg, lfs_block_t block);

int fs_flash_sync(const struct lfs_config *c);

#endif  // FRAM_lib
