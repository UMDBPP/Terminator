#include <stdio.h>
#include <string.h>

#include "ff.h"
#include "diskio.h"
#include "lfs.h"
#include "fram.h"

#include "stm32l412xx.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_spi.h"

static uint32_t init_flag = 0;

static fram_t *memory = NULL;

void disk_set_fram(fram_t *dev) {
	memory = dev;
}

DSTATUS disk_status(BYTE pdrv) {
    DSTATUS status = init_flag;

    return status;
}

DSTATUS disk_initialize(BYTE pdrv) {

	if (memory == NULL) return RES_ERROR;

    return RES_OK;
}

DRESULT disk_read(BYTE pdrv, BYTE* buff, LBA_t sector, UINT count) {
    if (!(disk_status(pdrv) && 0x01)) return RES_NOTRDY;

    if (FF_MIN_SS == FF_MAX_SS) {
		fram_read(memory, memory->spi_device, (sector * FF_MIN_SS), (uint8_t *)buff, (count * FF_MIN_SS));
    } else {
        printf("Min/Max sector size not equal");
        return RES_PARERR;
    }

    return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count) {
    if (!(disk_status(pdrv) && 0x01)) return RES_NOTRDY;

    if (FF_MIN_SS == FF_MAX_SS) {
		fram_write(memory, memory->spi_device, (sector * FF_MIN_SS), (uint8_t *)buff, (count * FF_MIN_SS));
    } else {
        printf("Min/Max sector size not equal");
        return RES_PARERR;
    }

    return RES_OK;
}
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void* buff) {
    if (cmd == CTRL_SYNC) {
        return RES_OK;
    } else if (cmd == GET_SECTOR_COUNT) {
		*((LBA_t *)buff) = 16;
		return RES_OK;
	} else if (cmd == GET_BLOCK_SIZE) {
		*((DWORD *) buff) = 1;
		return RES_OK;
	} else {
        // nothing to do here since the SPI writes are blocking
        return RES_ERROR;
    }
}

DWORD get_fattime(void) {
    const DWORD timestamp =
        0x2E290000;  // forgive the magic number, since there's no reliable time
                     // keeping every file will just get this time stamp which
                     // is January 9th, 2003

    return timestamp;
}

int fs_flash_read(const struct lfs_config *cfg, lfs_block_t block,
        lfs_off_t off, void *buffer, lfs_size_t size)
{
    assert(off  % cfg->read_size == 0);
    assert(size % cfg->read_size == 0);
    assert(block < cfg->block_count);

	uint32_t startAddress = block*(cfg->block_size) + off;
    int ret = fram_read(memory, memory->spi_device, startAddress, (uint8_t *)buffer, size); 

    if (ret == -1)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

int fs_flash_prog(const struct lfs_config *cfg, lfs_block_t block,
            lfs_off_t off, const void *buffer, lfs_size_t size)
{

    assert(off  % cfg->prog_size == 0);
    assert(size % cfg->prog_size == 0);
    assert(block < cfg->block_count);  

	uint32_t startAddress = block*(cfg->block_size) + off;
    int ret = fram_write(memory, memory->spi_device, startAddress, (uint8_t *)buffer, size); 

    if (ret == -1)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

int fs_flash_erase(const struct lfs_config *cfg, lfs_block_t block)
{
    assert(block < cfg->block_count);  
    uint8_t zero = 0x00;

	uint32_t startAddress = block*(cfg->block_size);
    int ret = fram_write(memory, memory->spi_device, startAddress, &zero, 128);

    if (ret == -1)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

int fs_flash_sync(const struct lfs_config *c)
{
    return 0;
}
