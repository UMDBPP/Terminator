#include <stdio.h>
#include <string.h>

#include "fram.h"
#include "lfs.h"

#include "stm32l422xx.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_spi.h"

static uint32_t init_flag = 0;

// relies on an external FRAM memory device declared elsewhere (like in main.c)
extern fram_t memory;

// or you can set the device
void disk_set_fram(fram_t dev) { memory = dev; }

// Mounts, opens file, write to file, closes file, unmounts, returns -1 if mount
// fails
int write_buf_to_fs(lfs_t *lfs, const struct lfs_config *config,
                    lfs_file_t *file, const char *path, char *buf,
                    uint32_t bytes) {

  int err = lfs_mount(lfs, config);

  if (err) {
    return -1;
  }

  lfs_file_open(lfs, file, path, LFS_O_RDWR | LFS_O_CREAT | LFS_O_APPEND);

  lfs_file_write(lfs, file, buf, bytes);

  lfs_file_close(lfs, file);
  lfs_unmount(lfs);

  return 0;
}

// littleFS port functions

int fs_flash_read(const struct lfs_config *cfg, lfs_block_t block,
                  lfs_off_t off, void *buffer, lfs_size_t size) {
  assert(off % cfg->read_size == 0);
  assert(size % cfg->read_size == 0);
  assert(block < cfg->block_count);

  uint32_t startAddress = block * (cfg->block_size) + off;
  int ret = fram_read(&memory, memory.spi_device, startAddress,
                      (uint8_t *)buffer, size);

  if (ret == -1) {
    return -1;
  } else {
    return 0;
  }
}

int fs_flash_prog(const struct lfs_config *cfg, lfs_block_t block,
                  lfs_off_t off, const void *buffer, lfs_size_t size) {

  assert(off % cfg->prog_size == 0);
  assert(size % cfg->prog_size == 0);
  assert(block < cfg->block_count);

  uint32_t startAddress = block * (cfg->block_size) + off;
  int ret = fram_write(&memory, memory.spi_device, startAddress,
                       (uint8_t *)buffer, size);

  if (ret == -1) {
    return -1;
  } else {
    return 0;
  }
}

int fs_flash_erase(const struct lfs_config *cfg, lfs_block_t block) {
  assert(block < cfg->block_count);
  uint8_t zero = 0x00;

  uint32_t startAddress = block * (cfg->block_size);
  int ret = fram_write(&memory, memory.spi_device, startAddress, &zero, 128);

  if (ret == -1) {
    return -1;
  } else {
    return 0;
  }
}

// SPI implementation is blocking
int fs_flash_sync(const struct lfs_config *c) { return 0; }
