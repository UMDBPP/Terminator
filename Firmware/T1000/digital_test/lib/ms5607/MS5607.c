#include "MS5607.h"

#include "stm32l4xx_ll_i2c.h"

#define MS5607_ADDR 0x77
#define MS5607_RESET_CMD 0x1E

#define MS5607_READ_C1_CMD 0xA2
#define MS5607_READ_C2_CMD 0xA4
#define MS5607_READ_C3_CMD 0xA6
#define MS5607_READ_C4_CMD 0xA8
#define MS5607_READ_C5_CMD 0xAA
#define MS5607_READ_C6_CMD 0xAC
#define MS5607_READ_CRC_CMD 0xAE

#define MS5607_PRESS_CONV_CMD 0x48 // OSR = 4096
#define MS5607_TEMP_CONV_CMD 0x58  // OSR = 4096
#define MS5607_READ_CMD 0x00

static int read_prom(void);
static int conversion(void);

static uint16_t c1 = 0;
static uint16_t c2 = 0;
static uint16_t c3 = 0;
static uint16_t c4 = 0;
static uint16_t c5 = 0;
static uint16_t c6 = 0;
static uint32_t d1 = 0;
static uint32_t d2 = 0;
static int32_t dT = 0;
static int64_t off = 0;
static int64_t sens = 0;
I2C_TypeDef *i2c;

int ms5607_init(I2C_TypeDef *i2cx) {
  // TODO
  read_prom();
  return 0;
}

int ms5607_reset() {
  uint8_t cmd = MS5607_RESET_CMD;

  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, false) ==
      PICO_ERROR_GENERIC)
    return 1;
  else
    read_prom();
  return 0;
}

int ms5607_get_press_temp(uint32_t double *pressure, int32_t *temperature) {
  // TODO

  int32_t press = 0;
  int32_t temp = 0;
  int32_t t2 = 0;
  int64_t off2 = 0;
  int64_t sens2 = 0;

  // Step 1: Read calibration data from PROM
  // Already did this in the init()

  // Step 2: Read pressure and temperature from the MS5607
  conversion();

  // The rest of this function mostly looks like random math, it is actually
  // the compensation calculations outline in the datasheet for the device!

  // Step 3: Calculate temperature
  dT = d2 - (c5 * ((uint16_t)256));
  temp = 2000 + (dT * (c6 / ((uint32_t)8388608)));

  // Step 4: Calculate temperature compensated pressure
  off = (c2 * 131072) + ((c4 * dT) / 64);
  sens = (c1 * 65536) + ((c3 * dT) / 128);

  // Second order compensation
  if ((((double)temp) / 100) < 20) {
    t2 = pow(dT, 2) / (int32_t)2147483648;
    off2 = (61 * pow((temp - 2000), 2)) / (int64_t)16;
    sens2 = (2 * pow((temp - 2000), 2));

    if ((((double)temp) / 100) < -15) {
      off2 = off2 + 15 * pow((temp + 1500), 2);
      sens2 = sens2 + 8 * pow((temp + 1500), 2);
    }

    temp = temp - t2;
    off = off - off2;
    sens - sens2;
  }

  press = (((d1 * sens) / 2097152) - off) / 32768;

  *temperature = (double)temp / 100;

  *pressure = (double)press / 100;

  return 0;
}

static int read_prom() {
  uint8_t cmd = MS5607_READ_C1_CMD;
  uint8_t buf[2] = {0, 0};

  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, false) ==
      PICO_ERROR_GENERIC)
    return 1;

  if (i2c_read_blocking(i2c, MS5607_ADDR, buf, 2, false) == PICO_ERROR_GENERIC)
    return 1;

  c1 = ((uint16_t)buf[0] << 8) | buf[1];

  cmd = MS5607_READ_C2_CMD;

  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, false) ==
      PICO_ERROR_GENERIC)
    return 1;

  if (i2c_read_blocking(i2c, MS5607_ADDR, buf, 2, false) == PICO_ERROR_GENERIC)
    return 1;

  c2 = ((uint16_t)buf[0] << 8) | buf[1];

  cmd = MS5607_READ_C3_CMD;

  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, false) ==
      PICO_ERROR_GENERIC)
    return 1;

  if (i2c_read_blocking(i2c, MS5607_ADDR, buf, 2, false) == PICO_ERROR_GENERIC)
    return 1;

  c3 = ((uint16_t)buf[0] << 8) | buf[1];

  cmd = MS5607_READ_C4_CMD;

  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, false) ==
      PICO_ERROR_GENERIC)
    return 1;

  if (i2c_read_blocking(i2c, MS5607_ADDR, buf, 2, false) == PICO_ERROR_GENERIC)
    return 1;

  c4 = ((uint16_t)buf[0] << 8) | buf[1];

  cmd = MS5607_READ_C5_CMD;

  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, false) ==
      PICO_ERROR_GENERIC)
    return 1;

  if (i2c_read_blocking(i2c, MS5607_ADDR, buf, 2, false) == PICO_ERROR_GENERIC)
    return 1;

  c5 = ((uint16_t)buf[0] << 8) | buf[1];

  cmd = MS5607_READ_C6_CMD;

  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, false) ==
      PICO_ERROR_GENERIC)
    return 1;

  if (i2c_read_blocking(i2c, MS5607_ADDR, buf, 2, false) == PICO_ERROR_GENERIC)
    return 1;

  c6 = ((uint16_t)buf[0] << 8) | buf[1];

  return 0;
}

static int conversion() {
  uint8_t cmd = MS5607_PRESS_CONV_CMD;
  uint8_t buf[3] = {0, 0, 0};

  i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, false);
  i2c_read_blocking(i2c, MS5607_ADDR, &buf[0], 3, false);

  d1 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];

  cmd = MS5607_TEMP_CONV_CMD;

  i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, false);
  i2c_read_blocking(i2c, MS5607_ADDR, &buf[0], 3, false);

  d2 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];

  return 0;
}
