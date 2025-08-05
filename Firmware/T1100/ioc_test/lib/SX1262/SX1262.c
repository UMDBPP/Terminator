#include "SX1262.h"

// #include <math.h>
// #include <stdio.h>
// #include <stdlib.h>
#include <string.h>

#include "stm32_helper.h"

#include "stm32wle5xx.h"
#include "stm32wlxx_ll_gpio.h"
#include "stm32wlxx_ll_spi.h"
#include "stm32wlxx_ll_utils.h"

#define INCLUDE_DEBUG 1

const uint8_t read_reg_cmd = SX126X_CMD_READ_REGISTER;
const uint8_t get_status_cmd = SX126X_CMD_GET_STATUS;
const uint8_t nop_cmd = 0x00;
const uint8_t addr2 = 0x07;
const uint8_t addr1 = 0x40;
uint8_t msg = 0x00;
const uint8_t StdbyConfig = 0x01;
const uint8_t set_standby_cmd = SX126X_CMD_SET_STANDBY;
const uint8_t get_err_cmd = SX126X_CMD_GET_DEVICE_ERRORS;
const uint8_t set_packet_type_cmd = SX126X_CMD_SET_PACKET_TYPE;
const uint8_t packet_type_lora = 0x01;
const uint8_t packet_type_fsk = 0x00;
const uint8_t pa_config_cmd = SX126X_CMD_SET_PA_CONFIG;
const uint8_t set_rf_freq_cmd = SX126X_CMD_SET_RF_FREQUENCY;
const uint8_t set_tx_params_cmd = SX126X_CMD_SET_TX_PARAMS;
const uint8_t set_buffer_base_addr_cmd = SX126X_CMD_SET_BUFFER_BASE_ADDRESS;
const uint8_t write_radio_buffer_cmd = SX126X_CMD_WRITE_BUFFER;
const uint8_t set_modulation_param_cmd = SX126X_CMD_SET_MODULATION_PARAMS;
const uint8_t write_radio_register_cmd = SX126X_CMD_WRITE_REGISTER;
const uint8_t tx_continuous_wave_cmd = SX126X_CMD_SET_TX_CONTINUOUS_WAVE;
const uint8_t set_tx_cmd = SX126X_CMD_SET_TX;
const uint8_t set_dio2_rf_ctrl_cmd = SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL;
const uint8_t set_packet_param_cmd = SX126X_CMD_SET_PACKET_PARAMS;
const uint8_t clear_radio_err_cmd = SX126X_CMD_CLEAR_DEVICE_ERRORS;
const uint8_t set_dio3_as_tcxo_cmd = SX126X_CMD_SET_DIO3_AS_TCXO_CTRL;
const uint8_t set_regulator_mode_cmd = SX126X_CMD_SET_REGULATOR_MODE;
const uint8_t set_radio_rx_cmd = SX126X_CMD_SET_RX;
const uint8_t set_radio_dio_irq_cmd = SX126X_CMD_SET_DIO_IRQ_PARAMS;
const uint8_t set_radio_clear_irq_cmd = SX126X_CMD_CLEAR_IRQ_STATUS;
const uint8_t read_buffer_cmd = SX126X_CMD_READ_BUFFER;
const uint8_t get_irq_status_cmd = SX126X_CMD_GET_IRQ_STATUS;
const uint8_t get_rx_buffer_cmd = SX126X_CMD_GET_RX_BUFFER_STATUS;
const uint8_t set_lora_symb_timeout_cmd = SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT;
const uint8_t calibrate_image_cmd = SX126X_CMD_CALIBRATE_IMAGE;

static uint8_t rx_payload_length = 0x00;
static uint8_t rx_buffer_start = 0x00;

static void set_radio_packet_type_fsk(void);
static void set_radio_fsk_modulation_param(void);
static void set_lora_symb_timeout(void);
static void calibrate_image(void);
static void set_radio_sync_word(void);
static void radio_spi_init();
static void set_buffer_base_address();
static void set_dio2_rf_switch(void);
static void set_tx(void);
static void set_fsk_packet_parameters(void);
static void set_lora_packet_parameters(void);
static void set_dio3_as_tcxo(void);
static void set_regulator_mode(void);
static void set_dio_irq(void);
static void set_radio_packet_type_lora(void);
static void set_radio_pa_config(void);
static void set_radio_rf_freq(void);
static void set_tx_params(void);
static void set_radio_lora_modulation_param();

extern sx1262_t radio;

/**
 * Writes payload data to the SX1262's buffer
 *
 * @param offset address to start writing data at
 * @param data pointer to payload data buffer
 * @param num_bytes length of buffer, should not exceed 255
 *
 * @return -1 if buffer length is too large
 */
static short write_radio_buffer(uint8_t offset, uint8_t *data,
                                size_t num_bytes);

// extern short debug_msg_en;  // controls if debug messages are
// printed

void sx1262_init() {

  // printf("Initializing Radio\n");

  // if (rst_pin <= 29) {
  //     gpio_init(rst_pin);
  //     gpio_set_dir(rst_pin, GPIO_OUT);
  //     gpio_put(rst_pin, 1);
  // }

  radio_spi_init();

  // Step 1: Enter STDBY_RC
  set_radio_standby();

  set_dio3_as_tcxo();
  set_dio2_rf_switch();
  set_dio_irq();
  set_regulator_mode();

  clear_radio_errors();

  get_radio_errors();

  // Step 2: Set Packet Type
  if (PACKET_LORA) {
    set_radio_packet_type_lora();
  } else {
    set_radio_packet_type_fsk();
  }

  // Step 3: Set RF Frequency
  set_radio_rf_freq();

  // Step 4: Set PA Config
  set_radio_pa_config();

  // Step 5: Set TX Parameters
  set_tx_params();

  // Step 6: Set Buffer Base Address
  set_buffer_base_address();

  // Step 8: Set Modulation Parameters
  set_radio_lora_modulation_param();

  // Step 9: Set Packet Parameters
  set_lora_packet_parameters();

  // Step 10: Configure DIO
  // set_dio2_rf_switch();

  // Step 11: Define Sync Word
  set_radio_sync_word();
  set_lora_symb_timeout();

  // TODO calibrate image

  read_radio_registers();
}

void get_radio_status() {
  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &get_status_cmd, 1);
  spi_read(radio.spix, &radio.status, 1);
  gpio_put(radio.cs_pin, 1);
  // printf("radio status: %x\n", status);
}

void set_radio_standby() {
  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &set_standby_cmd, 1);
  spi_write(radio.spix, &StdbyConfig, 1);
  spi_write(radio.spix, &nop_cmd, 1);
  spi_write(radio.spix, &nop_cmd, 1);
  spi_write(radio.spix, &nop_cmd, 1);
  gpio_put(radio.cs_pin, 1);
}

void get_radio_errors() {
  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &get_err_cmd, 1);
  spi_read(radio.spix, &msg, 1);
  // printf("status: %x\n", msg);
  spi_read(radio.spix, &msg, 1);
  // printf("err: %x\n", msg);
  spi_read(radio.spix, &msg, 1);
  gpio_put(radio.cs_pin, 1);
  // printf("err: %x\n", msg);
}

void read_radio_registers() {
  // printf("reg: %x%x\n", addr2, addr1);

  gpio_put(radio.cs_pin, 0);
  spi_read_write(radio.spix, &read_reg_cmd, &msg, 1);

  spi_read_write(radio.spix, &addr2, &msg, 1);

  spi_read_write(radio.spix, &addr1, &msg, 1);

  spi_read(radio.spix, &msg, 1);
  // printf("status: %x\n", msg);

  for (int j = 0; j <= 1; j++) {
    spi_read(radio.spix, &msg, 1);
    // printf("read: %x\n", msg);
  }
  gpio_put(radio.cs_pin, 1);
}

void radio_spi_init() {
  // printf("Init radio SPI\n");

  gpio_put(radio.cs_pin, 1);
  gpio_put(radio.sw_pin, 1);   // TODO, what should this be???
  gpio_put(radio.txen_pin, 0); // gpio_put(txen_pin, 1);
}

void set_radio_packet_type_lora() {
#if INCLUDE_DEBUG
  if (radio.debug_msg_en) {
    // printf("Setting Packet Type to LoRa\n");
  }
#endif

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &set_packet_type_cmd, 1);
  spi_write(radio.spix, &packet_type_lora, 1);
  gpio_put(radio.cs_pin, 1);
}

void set_radio_pa_config() {
  const uint8_t pa_duty = 0x04;
  const uint8_t hp_max = 0x07;
  const uint8_t device_sel = 0x00;
  const uint8_t pa_lut = 0x01;

#if INCLUDE_DEBUG
  if (radio.debug_msg_en) {
    // printf("Setting PA Config\n");
  }
#endif

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &pa_config_cmd, 1);
  spi_write(radio.spix, &pa_duty, 1);
  spi_write(radio.spix, &hp_max, 1);
  spi_write(radio.spix, &device_sel, 1);
  spi_write(radio.spix, &pa_lut, 1);
  gpio_put(radio.cs_pin, 1);
}

void set_radio_rf_freq() {
  const uint32_t frequency = 915000000;
  uint8_t buf[4];
  uint32_t freq = 0;

  freq = (uint32_t)((double)frequency / (double)FREQ_STEP);
  buf[0] = (uint8_t)((freq >> 24) & 0xFF);
  buf[1] = (uint8_t)((freq >> 16) & 0xFF);
  buf[2] = (uint8_t)((freq >> 8) & 0xFF);
  buf[3] = (uint8_t)(freq & 0xFF);

  // printf("Setting Frequency to %d\n", frequency);

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &set_rf_freq_cmd, 1);
  spi_write(radio.spix, &buf[0], 1);
  spi_write(radio.spix, &buf[1], 1);
  spi_write(radio.spix, &buf[2], 1);
  spi_write(radio.spix, &buf[3], 1);
  gpio_put(radio.cs_pin, 1);
}

void set_tx_params() {
  const uint8_t power = 0x00;                    // 0xF7 -> 0x16, -9 -> 22
  const uint8_t ramp_time = SX126X_PA_RAMP_200U; // 200us ramp time

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &set_tx_params_cmd, 1);
  spi_write(radio.spix, &power, 1);
  spi_write(radio.spix, &ramp_time, 1);
  gpio_put(radio.cs_pin, 1);
}

void set_buffer_base_address() {
  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &set_buffer_base_addr_cmd, 1);
  spi_write(radio.spix, &(radio.tx_buffer), 1);
  spi_write(radio.spix, &(radio.rx_buffer), 1);
  gpio_put(radio.cs_pin, 1);
}

short write_radio_buffer(uint8_t offset, uint8_t *data, size_t num_bytes) {
  if (num_bytes > 255)
    return -1;

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &write_radio_buffer_cmd, 1);
  spi_write(radio.spix, &offset, 1);
  spi_write(radio.spix, data, num_bytes);
  gpio_put(radio.cs_pin, 1);

  return 0;
}

void set_radio_lora_modulation_param() {
  const uint8_t spreading_factor = 0x0B;
  const uint8_t bandwidth = 0x03;
  const uint8_t coding_rate = 0x04;
  const uint8_t low_data_rate = 0x01;

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &set_modulation_param_cmd, 1);
  spi_write(radio.spix, &spreading_factor, 1);
  spi_write(radio.spix, &bandwidth, 1);
  spi_write(radio.spix, &coding_rate, 1);
  spi_write(radio.spix, &low_data_rate, 1);
  gpio_put(radio.cs_pin, 1);
}

void set_lora_packet_parameters() {
  const uint8_t preamble2 = 0x00;
  const uint8_t preamble1 = 0x0F;
  const uint8_t header = 0x00;
  const uint8_t length = 0x64;
  const uint8_t crc = 0x01;
  const uint8_t iq = 0x00;

  // printf("Setting LoRa Packet Parameters\n");

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &set_packet_param_cmd, 1);
  spi_write(radio.spix, &preamble2, 1);
  spi_write(radio.spix, &preamble1, 1);
  spi_write(radio.spix, &header, 1);
  spi_write(radio.spix, &length, 1);
  spi_write(radio.spix, &crc, 1);
  spi_write(radio.spix, &iq, 1);
  gpio_put(radio.cs_pin, 1);
}

void set_fsk_packet_parameters() {
  const uint8_t preamble2 = 0x00;
  const uint8_t preamble1 = 0x0F;
  const uint8_t preamble_det_len = 0x00;
  const uint8_t sync_len = 0x08;
  const uint8_t addr_comp = 0x00;
  const uint8_t packet_type = 0x01; // variable packet size
  const uint8_t payload_len = 0x01;
  const uint8_t crc = 0x01;       // CRC off
  const uint8_t whitening = 0x00; // no encoding

  // printf("Setting FSK Packet Parameters\n");

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &set_packet_param_cmd, 1);
  spi_write(radio.spix, &preamble2, 1);
  spi_write(radio.spix, &preamble1, 1);
  spi_write(radio.spix, &preamble_det_len, 1);
  spi_write(radio.spix, &sync_len, 1);
  spi_write(radio.spix, &addr_comp, 1);
  spi_write(radio.spix, &packet_type, 1);
  spi_write(radio.spix, &payload_len, 1);
  spi_write(radio.spix, &crc, 1);
  spi_write(radio.spix, &whitening, 1);
  gpio_put(radio.cs_pin, 1);
}

void set_dio2_rf_switch() {
  const uint8_t enable = 1;

#if INCLUDE_DEBUG
  if (radio.debug_msg_en) {
    // printf("Setting DIO2 as RF Switch\n");
  }
#endif

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &set_dio2_rf_ctrl_cmd, 1);
  spi_write(radio.spix, &enable, 1);
  gpio_put(radio.cs_pin, 1);
}

void set_radio_sync_word() {
  const uint8_t msb2 = 0x07;
  const uint8_t msb1 = 0x40;
  const uint8_t lsb2 = 0x07;
  const uint8_t lsb1 = 0x41;
  const uint8_t data2 = 0x34;
  const uint8_t data1 = 0x44;

#if INCLUDE_DEBUG
  if (radio.debug_msg_en) {
    // printf("Setting Radio Sync Word\n");
  }
#endif

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &write_radio_register_cmd, 1);
  spi_write(radio.spix, &msb2, 1);
  spi_write(radio.spix, &msb1, 1);
  spi_write(radio.spix, &data2, 1);
  gpio_put(radio.cs_pin, 1);

  LL_mDelay(10);

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &write_radio_register_cmd, 1);
  spi_write(radio.spix, &lsb2, 1);
  spi_write(radio.spix, &lsb1, 1);
  spi_write(radio.spix, &data1, 1);
  gpio_put(radio.cs_pin, 1);
}

void set_tx_continuous_wave() {
#if INCLUDE_DEBUG
  if (radio.debug_msg_en) {
    // printf("Setting Mode TX Tone\n");
  }
#endif

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &tx_continuous_wave_cmd, 1);
  gpio_put(radio.cs_pin, 1);
}

void set_tx() {
  const uint8_t timeout3 = 0x00;
  const uint8_t timeout2 = 0x00;
  const uint8_t timeout1 = 0x00;

#if INCLUDE_DEBUG
  if (radio.debug_msg_en) {
    // printf("Setting Mode TX\n");
  }
#endif

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &set_tx_cmd, 1);
  spi_write(radio.spix, &timeout3, 1);
  spi_write(radio.spix, &timeout2, 1);
  spi_write(radio.spix, &timeout1, 1);
  gpio_put(radio.cs_pin, 1);
}

void set_dio3_as_tcxo() {
  const uint8_t tcxoVoltage = 0x07;
  const uint8_t timeout3 = 0x01;
  const uint8_t timeout2 = 0x02;
  const uint8_t timeout1 = 0x80;

#if INCLUDE_DEBUG
  if (radio.debug_msg_en) {
    // printf("Setting DIO3 as TCXO CTRL\n");
  }
#endif

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &set_dio3_as_tcxo_cmd, 1);
  spi_write(radio.spix, &tcxoVoltage, 1);
  spi_write(radio.spix, &timeout3, 1);
  spi_write(radio.spix, &timeout2, 1);
  spi_write(radio.spix, &timeout1, 1);
  gpio_put(radio.cs_pin, 1);

  // don't delete this unless you want to play with the timeout value again
  LL_mDelay(10);
}

void set_regulator_mode() {
  const uint8_t mode = 0x01;
#if INCLUDE_DEBUG
  if (radio.debug_msg_en) {
    // printf("Setting Regulator Mode to DC DC\n");
  }
#endif

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &set_regulator_mode_cmd, 1);
  spi_write(radio.spix, &mode, 1);
  gpio_put(radio.cs_pin, 1);
}

void clear_radio_errors() {
#if INCLUDE_DEBUG
  if (radio.debug_msg_en) {
    // printf("Clearing radio errors\n");
  }
#endif

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &clear_radio_err_cmd, 1);
  spi_read(radio.spix, &msg, 1);
  spi_read(radio.spix, &msg, 1);
  gpio_put(radio.cs_pin, 1);
}

// smarter way of figuring out when transmit has ended with interrupts
void radio_send(uint8_t *data, size_t len) {
  gpio_put(radio.txen_pin, 0);
  uint8_t payload = write_radio_buffer(radio.tx_buffer, data, len);
  set_tx();
}

void disable_tx(void) { gpio_put(radio.txen_pin, 1); }

void radio_receive_cont() {
  uint8_t timeout3 = 0xFF;
  uint8_t timeout2 = 0xFF;
  uint8_t timeout1 = 0xFF;

#if INCLUDE_DEBUG
  if (radio.debug_msg_en) {
    // printf("Entering Radio Receive Mode (Continuous)\n");
  }
#endif

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &set_radio_rx_cmd, 1);
  spi_write(radio.spix, &timeout3, 1);
  spi_write(radio.spix, &timeout2, 1);
  spi_write(radio.spix, &timeout1, 1);
  gpio_put(radio.cs_pin, 1);
}

void radio_receive_single() {
  uint8_t timeout3 = 0x00;
  uint8_t timeout2 = 0x00;
  uint8_t timeout1 = 0x00;

#if INCLUDE_DEBUG
  if (radio.debug_msg_en) {
    // printf("Entering Radio Receive Mode (Single)\n");
  }
#endif

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &set_radio_rx_cmd, 1);
  spi_write(radio.spix, &timeout3, 1);
  spi_write(radio.spix, &timeout2, 1);
  spi_write(radio.spix, &timeout1, 1);
  gpio_put(radio.cs_pin, 1);
}

void set_dio_irq() {
  uint8_t irq_mask2 = 0xFF;
  uint8_t irq_mask1 = 0xFF;

  uint8_t dio1_mask2 = 0x00;
  uint8_t dio1_mask1 = 0xC3;

  uint8_t dio2_mask2 = 0x00;
  uint8_t dio2_mask1 = 0x00;
  uint8_t dio3_mask2 = 0x00;
  uint8_t dio3_mask1 = 0x00;

  // gpio_set_dir(dio1_pin, GPIO_IN);

  // gpio_set_irq_enabled(dio1_pin, GPIO_IRQ_EDGE_RISE, true);

  // printf("Setting DIO1 IRQ\n");
  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &set_radio_dio_irq_cmd, 1);
  spi_write(radio.spix, &irq_mask2, 1);
  spi_write(radio.spix, &irq_mask1, 1);
  spi_write(radio.spix, &dio1_mask2, 1);
  spi_write(radio.spix, &dio1_mask1, 1);
  spi_write(radio.spix, &dio2_mask2, 1);
  spi_write(radio.spix, &dio2_mask1, 1);
  spi_write(radio.spix, &dio3_mask2, 1);
  spi_write(radio.spix, &dio3_mask1, 1);
  gpio_put(radio.cs_pin, 1);
}

void clear_irq_status() {
  uint8_t irq_mask2 = 0xFF;
  uint8_t irq_mask1 = 0xFF;

#if INCLUDE_DEBUG
  if (radio.debug_msg_en) {
    // printf("Clearing IRQ\n");
  }
#endif

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &set_radio_clear_irq_cmd, 1);
  spi_write(radio.spix, &irq_mask2, 1);
  spi_write(radio.spix, &irq_mask1, 1);
  gpio_put(radio.cs_pin, 1);

  // irqs.RX_DONE = false;
  // irqs.TX_DONE = false;
}

short read_radio_buffer(uint8_t *data, size_t num_bytes) {
  get_rx_buffer_status();

  if (num_bytes > 255 || num_bytes < rx_payload_length)
    return -1;

  // printf("Reading Radio Buffer\n");
  gpio_put(radio.cs_pin, 0);
  spi_read_write(radio.spix, &read_buffer_cmd, &msg, 1);

  // these next two return the status of the radio
  spi_read_write(radio.spix, &rx_buffer_start, &msg, 1);
  spi_read(radio.spix, &msg, 1);

  spi_read(radio.spix, data, num_bytes);
  gpio_put(radio.cs_pin, 1);

#if INCLUDE_DEBUG
  if (radio.debug_msg_en) {
    // printf("Received data:");

    for (short i = 0; i < num_bytes; i++) {
      // printf(" %x", data[i]);
    }

    // printf("\n");
  }
#endif

  return 0;
}

void get_irq_status() {
  uint8_t status2 = 0x00;
  uint8_t status1 = 0x00;

#if INCLUDE_DEBUG
  if (radio.debug_msg_en) {
    // printf("Getting IRQ Status\n");
  }
#endif

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &get_irq_status_cmd, 1);
  spi_write(radio.spix, &nop_cmd, 1);
  spi_read(radio.spix, &status2, 1);
  spi_read(radio.spix, &status1, 1);
  gpio_put(radio.cs_pin, 1);

  // printf("IRQ Status Register %x %x\n", status2, status1);

  if (status1 & SX126X_IRQ_TX_DONE)
    radio.irqs.tx_done = 1;
  if (status1 & SX126X_IRQ_RX_DONE)
    radio.irqs.rx_done = 1;
  if (status1 & SX126X_IRQ_CAD_DONE)
    radio.irqs.cad_done = 1;
  if (status1 & SX126X_IRQ_CAD_DETECTED)
    radio.irqs.cad_det = 1;
}

void get_rx_buffer_status() {
#if INCLUDE_DEBUG
  if (radio.debug_msg_en) {
    // printf("Getting RX Buffer Status\n");
  }
#endif

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &get_rx_buffer_cmd, 1);
  spi_write(radio.spix, &nop_cmd, 1);
  spi_read(radio.spix, &rx_payload_length, 1);
  spi_read(radio.spix, &rx_buffer_start, 1);
  gpio_put(radio.cs_pin, 1);

#if INCLUDE_DEBUG
  if (radio.debug_msg_en) {
    // printf("Payload Length %x\n", rx_payload_length);
    // printf("Buffer Pointer %x\n", rx_buffer_start);
  }
#endif
}

void set_lora_symb_timeout() {
  uint8_t symb_num = 0x0F;

  spi_write(radio.spix, &set_lora_symb_timeout_cmd, 1);
  spi_write(radio.spix, &symb_num, 1);
}

void calibrate_image() {
  uint8_t freq1 = 0xE1;
  uint8_t freq2 = 0xE9;

  spi_write(radio.spix, &calibrate_image_cmd, 1);
  spi_write(radio.spix, &freq1, 1);
  spi_write(radio.spix, &freq2, 1);
}

void set_radio_packet_type_fsk() {
#if INCLUDE_DEBUG
  if (radio.debug_msg_en) {
    // printf("Setting Packet Type to FSK\n");
  }
#endif

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &set_packet_type_cmd, 1);
  spi_write(radio.spix, &packet_type_fsk, 1);
  gpio_put(radio.cs_pin, 1);
}

void get_packet_status() {
  const uint8_t rssi_cmd = SX126X_CMD_GET_PACKET_STATUS;
  uint8_t status = 0x00;
  uint8_t rssi_pkt;
  uint8_t snr_pkt;
  uint8_t signal_rssi_pkt;

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &rssi_cmd, 1);
  spi_read(radio.spix, &status, 1);
  spi_read(radio.spix, &rssi_pkt, 1);
  spi_read(radio.spix, &snr_pkt, 1);
  spi_read(radio.spix, &signal_rssi_pkt, 1);
  gpio_put(radio.cs_pin, 1);

  // memcpy(&(pkt_stat.rssi_pkt), &rssi_pkt, 1);
  memcpy(&(radio.pkt_stat.snr_pkt), &snr_pkt, 1);
  // memcpy(&(pkt_stat.signal_rssi_pkt), &signal_rssi_pkt, 1);

  radio.pkt_stat.rssi_pkt = (-rssi_pkt / 2);
  radio.pkt_stat.snr_pkt = radio.pkt_stat.snr_pkt / 4;
  radio.pkt_stat.signal_rssi_pkt = (-signal_rssi_pkt / 2);

  // printf("RSSI: 0x%x %d %d\n", rssi_pkt, (-rssi_pkt / 2),
  // pkt_stat.rssi_pkt); printf("SNR: 0x%x %d\n", snr_pkt, pkt_stat.snr_pkt);
}

// void print_radio_state() {}

void set_cad() {
  const uint8_t cmd = SX126X_CMD_SET_CAD;

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &cmd, 1);
  gpio_put(radio.cs_pin, 1);
}

void set_cad_params() {
  const uint8_t cmd = SX126X_CMD_SET_CAD_PARAMS;
  const uint8_t cad_sym_num = SX126X_CAD_ON_4_SYMB;
  const uint8_t cad_det_peak = 28;
  const uint8_t cad_det_min = 10;
  const uint8_t cad_exit_mode = SX126X_CAD_GOTO_RX;
  const uint8_t timeout3 = 0x09;
  const uint8_t timeout2 = 0xC4;
  const uint8_t timeout1 = 0x00;

  gpio_put(radio.cs_pin, 0);
  spi_write(radio.spix, &cmd, 1);
  spi_write(radio.spix, &cad_sym_num, 1);
  spi_write(radio.spix, &cad_det_peak, 1);
  spi_write(radio.spix, &cad_det_min, 1);
  spi_write(radio.spix, &cad_exit_mode, 1);
  spi_write(radio.spix, &timeout3, 1);
  spi_write(radio.spix, &timeout2, 1);
  spi_write(radio.spix, &timeout1, 1);
  gpio_put(radio.cs_pin, 1);
}
