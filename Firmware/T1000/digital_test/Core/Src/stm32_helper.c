#include "stm32_helper.h"
#include <stdlib.h>
#include <string.h>

void spi_read(SPI_TypeDef *spix, uint8_t *const buf, uint32_t num_bytes) {
  for (int i = 0; i < num_bytes; i++) {
    buf[i] = SPI_TxRx(spix, 0x00);
  }
}

void spi_write(SPI_TypeDef *spix, const uint8_t *const buf,
               uint32_t num_bytes) {
  for (int i = 0; i < num_bytes; i++)
    SPI_TxRx(spix, buf[i]);
}

// Lifted and modified from https://github.com/eziya/STM32_LL_EXAMPLES
// is blocking
uint8_t SPI_TxRx(SPI_TypeDef *spix, uint8_t data) {
  // transmit
  LL_SPI_TransmitData8(spix, data);
  while (!LL_SPI_IsActiveFlag_TXE(spix))
    ;

  // receive
  while (!LL_SPI_IsActiveFlag_RXNE(spix))
    ;
  return LL_SPI_ReceiveData8(spix);
}

// Below I2C functions lifted from
// https://github.com/eziya/STM32_LL_EXAMPLES/blob/main/STM32F4_LL_I2C_MASTER_BME280/Core/Src/main.c

// Master transmitter
int8_t I2C_MasterTx(uint8_t devAddr, uint8_t *buffer, uint16_t len,
                    uint16_t ms) {
  // timeout counter
  uint16_t cnt = 0;

  // 1. start condition
  LL_I2C_GenerateStartCondition(I2C1);

  // 2. check start bit flag
  while (!LL_I2C_IsActiveFlag_BUSY(I2C1)) {
    if (cnt++ > (ms * 25000))
      return -1;
  }

  // 3. write device address (WRITE)
  LL_I2C_TransmitData8(I2C1, (devAddr << 1) | 0x00);

  // 4. wait address sent
  while (!LL_I2C_IsActiveFlag_TXE(I2C1)) {
    if (cnt++ > (ms * 25000))
      return -1;
  }

  // 5. clear ADDR flag
  // LL_I2C_ClearFlag_ADDR(I2C1);

  // 6. check TXE flag & write data
  for (int i = 0; i < len; i++) {
    while (!LL_I2C_IsActiveFlag_TXE(I2C1)) {
      if (cnt++ > (ms * 25000))
        return -1;
    }

    LL_I2C_TransmitData8(I2C1, buffer[i]);
  }

  // 7. wait BTF flag (TXE flag set & empty DR condition)
  while (!LL_I2C_IsActiveFlag_TXE(I2C1)) {
    if (cnt++ > (ms * 25000))
      return -1;
  }

  // 8. stop condition
  LL_I2C_GenerateStopCondition(I2C1);

  return 0;
}

// Master receiver
int8_t I2C_MasterRx(uint8_t devAddr, uint8_t *buffer, uint8_t len,
                    uint16_t ms) {
  // timeout counter
  uint16_t cnt = 0;

  // 1. fast NACK configuration when receiving single byte.
  if (len == 1) {
    LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
  } else {
    LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
  }

  // LL_I2C_SetSlaveAddr(I2C1, (uint32_t) devAddr);
  // LL_I2C_SetTransferRequest(I2C1, LL_I2C_REQUEST_READ);

  LL_I2C_HandleTransfer(I2C1, (uint32_t)devAddr, LL_I2C_ADDRSLAVE_7BIT,
                        (uint32_t)len, LL_I2C_MODE_AUTOEND,
                        LL_I2C_GENERATE_START_READ);

  // 2. start condition
  LL_I2C_GenerateStartCondition(I2C1);

  // 3. check start bit flag
  while (LL_I2C_IsActiveFlag_BUSY(I2C1)) {
    if (cnt++ > (ms * 25000))
      return -1;
  }

  // 4. write device address (READ)
  LL_I2C_TransmitData8(I2C1, (devAddr << 1) | 0x01);

  // 5. wait address sent
  while (!LL_I2C_IsActiveFlag_TXE(I2C1)) {
    if (cnt++ > (ms * 25000))
      return -1;
  }

  // 6. clear ADDR flag
  // LL_I2C_ClearFlag_ADDR(I2C1);

  // 7. check RXNE flag & read data
  for (int i = 0; i < len; i++) {
    // 8. NACK at last byte
    if (i == len - 1) {
      LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
    }

    while (!LL_I2C_IsActiveFlag_RXNE(I2C1)) {
      if (cnt++ > (ms * 25000))
        return -1;
    }

    buffer[i] = LL_I2C_ReceiveData8(I2C1);
  }

  // 9. stop condition
  LL_I2C_GenerateStopCondition(I2C1);

  return 0;
}

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data,
                     uint16_t len) {
  // send register
  if (I2C_MasterTx(id, &reg_addr, 1, 50) < 0)
    return -1;
  // read data
  if (I2C_MasterRx(id, data, len, 50) < 0)
    return -1;
  return 0;
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data,
                      uint16_t len) {
  int8_t *buf;
  buf = malloc(len + 1);
  buf[0] = reg_addr;
  memcpy(buf + 1, data, len);

  // send register + data
  if (I2C_MasterTx(id, (uint8_t *)buf, len + 1, 50) < 0) {
    free(buf);
    return -1;
  }

  free(buf);
  return 0;
}
