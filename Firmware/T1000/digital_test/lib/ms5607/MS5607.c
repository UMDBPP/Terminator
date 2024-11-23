#include "MS5607.h"

#include "fram.h"
#include "stm32l4xx_ll_i2c.h"
#include "stm32l4xx_ll_utils.h"

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
static int64_t dT = 0;
static int64_t off = 0;
static int64_t sens = 0;
static I2C_TypeDef *i2c;

#define DELAY 10

// these two I2C functions are not good, the args are mostly unused but writing
// the signature like this made copying RP2040 code faster
static int i2c_read_blocking(I2C_TypeDef *i2cx, uint8_t addr, uint8_t *buf,
                             uint32_t bytes, uint32_t temp) {
  int timeout = 0; // I2C software timeout counter

  LL_I2C_HandleTransfer(I2C1, (addr << 1), LL_I2C_ADDRSLAVE_7BIT, bytes,
                        LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

  for (int i = 0; i < bytes; i++) {

    while (LL_I2C_IsActiveFlag_RXNE(I2C1) == 0) { // wait for response
      if (timeout >= 2000000) {
        return -1;
      }
      timeout++;
    }

    buf[i] = LL_I2C_ReceiveData8(I2C1);
  }
  return 0;
}

static int i2c_write_blocking(I2C_TypeDef *i2cx, uint8_t addr, uint8_t *buf,
                              uint32_t bytes, uint32_t temp) {
  int timeout = 0; // I2C software timeout counter

  LL_I2C_HandleTransfer(I2C1, (addr << 1), LL_I2C_ADDRSLAVE_7BIT, bytes,
                        LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

  for (int i = 0; i < bytes; i++) {

    while (LL_I2C_IsActiveFlag_TXE(I2C1) == 0) {
      if (timeout >= 2000000) {
        return -1;
      }
      timeout++;
    }

    LL_I2C_TransmitData8(I2C1, buf[i]);
  }
  return 0;
}

int ms5607_init(I2C_TypeDef *i2cx) {
  if (read_prom() == -1)
    return -1;
  else
    return 0;
}

int ms5607_reset() {
  uint8_t cmd = MS5607_RESET_CMD;

  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, 0) == -1)
    return -1;
  else
    read_prom();
  return 0;
}

int ms5607_get_press_temp(uint32_t *pressure, int32_t *temperature) {

  int32_t press = 0;
  int32_t temp = 0;
  int32_t t2 = 0;
  int64_t off2 = 0;
  int64_t sens2 = 0;

  // Step 1: Read calibration data from PROM
  // Already did this in the init()

  // Step 2: Read pressure and temperature from the MS5607
  if (conversion() == -1)
    return -1;

  // The rest of this function mostly looks like random math, it is actually
  // the compensation calculations outline in the datasheet for the device!

  // Step 3: Calculate temperature
  dT = d2 - ((int64_t)c5 << 8);

  // temp is e.g. 2000 = 20.00 deg C
  temp = (int64_t)2000 + ((dT * (int64_t)c6) >> 23);

  // Step 4: Calculate temperature compensated pressure
  off = ((int64_t)c2 * (int64_t)131072) + (((int64_t)c4 * dT) / (int64_t)64);
  sens = ((int64_t)c1 * (int64_t)65536) + (((int64_t)c3 * dT) / (int64_t)128);

  // Second order compensation
  if (temp < (int)2000) {

    t2 = ((dT * dT) / ((int64_t)2147483648));
    off2 = (61 * ((temp - 2000) * (temp - 2000)) / 16);
    sens2 = (2 * ((temp - 2000) ^ 2));

    if (temp < -15) {

      off2 = (off2 + (15 * ((temp + 1500) * (temp + 1500))));
      sens2 = (sens2 + (8 * ((temp + 1500) * (temp + 1500))));
    }

    temp = temp - t2;
    off = off - off2;
    sens = sens - sens2;
  }

  press = (((d1 * sens) / (2097152)) - off) / (32768);

  // in "centi-celsius" e.g. 2000 = 20.00 deg C
  *temperature = temp;

  // in "centi-millibar" e.g. 110002 = 1100.02 mbar
  *pressure = press;

  return 0;
}

static int read_prom() {
  uint8_t buf[2] = {0, 0};

  uint8_t cmd = MS5607_READ_C1_CMD;
  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, 0) == -1)
    return -1;
  LL_mDelay(DELAY);
  if (i2c_read_blocking(i2c, MS5607_ADDR, buf, 2, 0) == -1)
    return -1;
  c1 = ((uint16_t)buf[0] << 8) | buf[1];

  cmd = MS5607_READ_C2_CMD;
  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, 0) == -1)
    return -1;
  LL_mDelay(DELAY);
  if (i2c_read_blocking(i2c, MS5607_ADDR, buf, 2, 0) == -1)
    return -1;
  c2 = ((uint16_t)buf[0] << 8) | buf[1];

  cmd = MS5607_READ_C3_CMD;
  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, 0) == -1)
    return -1;
  LL_mDelay(DELAY);
  if (i2c_read_blocking(i2c, MS5607_ADDR, buf, 2, 0) == -1)
    return -1;
  c3 = ((uint16_t)buf[0] << 8) | buf[1];

  cmd = MS5607_READ_C4_CMD;
  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, 0) == -1)
    return -1;
  LL_mDelay(DELAY);
  if (i2c_read_blocking(i2c, MS5607_ADDR, buf, 2, 0) == -1)
    return -1;
  c4 = ((uint16_t)buf[0] << 8) | buf[1];

  cmd = MS5607_READ_C5_CMD;
  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, 0) == -1)
    return -1;
  LL_mDelay(DELAY);
  if (i2c_read_blocking(i2c, MS5607_ADDR, buf, 2, 0) == -1)
    return -1;
  c5 = ((uint16_t)buf[0] << 8) | buf[1];

  cmd = MS5607_READ_C6_CMD;
  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, 0) == -1)
    return -1;
  LL_mDelay(DELAY);
  if (i2c_read_blocking(i2c, MS5607_ADDR, buf, 2, 0) == -1)
    return -1;
  c6 = ((uint16_t)buf[0] << 8) | buf[1];

  return 0;
}

static int conversion() {
  uint8_t buf[3] = {0, 0, 0};

  uint8_t cmd = MS5607_PRESS_CONV_CMD;
  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, 0) == -1)
    return -1;
  LL_mDelay(DELAY);

  cmd = MS5607_READ_CMD;
  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, 0) == -1)
    return -1;
  LL_mDelay(DELAY);

  if (i2c_read_blocking(i2c, MS5607_ADDR, buf, 3, 0) == -1)
    return -1;

  d1 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];

  buf[0] = 0;
  buf[1] = 0;
  buf[2] = 0;

  cmd = MS5607_TEMP_CONV_CMD;
  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, 0) == -1)
    return -1;
  LL_mDelay(DELAY);

  cmd = MS5607_READ_CMD;
  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, 0) == -1)
    return -1;
  LL_mDelay(DELAY);

  if (i2c_read_blocking(i2c, MS5607_ADDR, buf, 3, 0) == -1)
    return -1;

  d2 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];

  return 0;
}
