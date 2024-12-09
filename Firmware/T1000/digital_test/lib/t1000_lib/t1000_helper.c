#include "t1000_helper.h"

#include "PID.h"
#include "fram.h"
#include "lfs.h"
#include "stm32_helper.h"
#include "sx1262.h"

extern uint32_t int_count_30;
extern uint32_t cut_counter;
extern uint32_t cut_int_timer;

extern fram_t memory;
extern sx1262_t radio;
extern log_item_t log_item;

extern const struct lfs_config cfg;
extern lfs_t lfs;
extern lfs_file_t file;
extern lfs_file_t log_file;
extern lfs_file_t int_count_save;

extern PIDController pid;

geo_vertex empty_vertex = {0, 0, '0', 0, 0, '0'};

struct _geo_fence {

  geo_vertex top_left;
  geo_vertex bot_left;
  geo_vertex top_right;
  geo_vertex bot_right;

  uint32_t altitude;

} geo_fence;

void geofence_init() {
  // set geofence parameters
  geo_fence.top_left = empty_vertex;
  geo_fence.bot_left = empty_vertex;
  geo_fence.top_right = empty_vertex;
  geo_fence.bot_right = empty_vertex;
  geo_fence.altitude = 0;
}

void dump_fram() {

  uint8_t data = 0;

  LL_USART_EnableDirectionTx(USART2);

  for (int i = 0; i < 131072; i++) {

    fram_read(&memory, SPI1, i, &data, 1);

    LL_USART_TransmitData8(USART2, data);

    int timeout = 0;

    while (LL_USART_IsActiveFlag_TC(USART2) == 0) {
      if (timeout >= 50000)
        break;
      timeout++;
    }
  }
}

void pid_blocking(void) {
  uint16_t adc_value = 0;
  uint32_t isns_value = 0;
  uint32_t batt_value = 0;

  // Enable PWM channel outputs
  LL_TIM_EnableCounter(TIM1);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
  LL_TIM_OC_SetCompareCH3(TIM1, 0);
  LL_TIM_EnableAllOutputs(TIM1);

  LL_ADC_Disable(ADC1);
  MX_ADC1_Init();
  LL_ADC_Enable(ADC1);

  PA8_LOW

  while (1) {

    PA8_TOGGLE

    LL_IWDG_ReloadCounter(IWDG);

    // -- Read ADC --
    LL_ADC_REG_StartConversion(ADC1);

    // wait end of conversion flag
    while (!LL_ADC_IsActiveFlag_EOC(ADC1))
      ;

    // read channel1 data in mV
    adc_value = LL_ADC_REG_ReadConversionData12(ADC1);

    // clear flag
    LL_ADC_ClearFlag_EOC(ADC1);

    LL_ADC_REG_StartConversion(ADC1);

    if (log_item.pressure <
        100000) // total kludge since it's difficult to figure out if on
                // battery or USB power right now, if pressure is less than
                // 1000 mbar it's pretty likely that system is airborne
      batt_value =
          __LL_ADC_CALC_DATA_TO_VOLTAGE(3000, adc_value, LL_ADC_RESOLUTION_12B);
    else
      batt_value =
          __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, adc_value, LL_ADC_RESOLUTION_12B);

    batt_value = (batt_value * 24) / 10;

    // wait end of conversion flag
    while (!LL_ADC_IsActiveFlag_EOC(ADC1))
      ;

    // read channel1 data in mV
    adc_value = LL_ADC_REG_ReadConversionData12(ADC1);

    // clear flag
    LL_ADC_ClearFlag_EOC(ADC1);

    if (log_item.pressure <
        100000) // total kludge since it's difficult to figure out if on
                // battery or USB power right now, if pressure is less than
                // 1000 mbar it's pretty likely that system is airborne
      isns_value =
          __LL_ADC_CALC_DATA_TO_VOLTAGE(3000, adc_value, LL_ADC_RESOLUTION_12B);
    else
      isns_value =
          __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, adc_value, LL_ADC_RESOLUTION_12B);
    // end ADC read

    PIDController_Update(
        &pid, 435,
        isns_value); // update PID controller with set point 1000mV
    LL_TIM_OC_SetCompareCH3(TIM1, pid.out); // pid.out
  }
}

void update_int_count(uint32_t count) {
  lfs_file_open(&lfs, &int_count_save, "int_count_30",
                LFS_O_RDWR | LFS_O_CREAT);
  lfs_file_rewind(&lfs, &int_count_save);
  lfs_file_write(&lfs, &int_count_save, &count, sizeof(count));
  lfs_file_close(&lfs, &int_count_save);
  lfs_unmount(&lfs);
}

int check_geo_fence() { return 0; }

// counts number of digits in uint32_t, returns anywhere from 1-7
int count_revifs(uint32_t n) {
  if (n > 999999)
    return 7;
  if (n > 99999)
    return 6;
  if (n > 9999)
    return 5;
  if (n > 999)
    return 4;
  if (n > 99)
    return 3;
  if (n > 9)
    return 2;
  return 1;
}

// read the log file line by line
void read_log() {

  static char buf[100] = {0};
  uint32_t file_size = 0;

  MX_USART2_UART_Init();

  LL_USART_EnableDirectionRx(USART2);
  LL_USART_EnableDirectionTx(USART2);

  int err = lfs_mount(&lfs, &cfg);

  if (err) {
    exit(-1); // trouble mounting FS
    // lfs_format(&lfs, &cfg);
    // lfs_mount(&lfs, &cfg);
  }

  err = lfs_file_open(&lfs, &log_file, "flight_log", LFS_O_RDONLY);
  lfs_file_rewind(&lfs, &log_file);

  file_size = lfs_file_size(&lfs, &log_file);

  uint32_t pos = 0;

  while (lfs_file_tell(&lfs, &log_file) < file_size) {
    for (int i = 0; i < 99; i++) {
      lfs_file_read(&lfs, &log_file, (buf + i), 1);
      LL_USART_TransmitData8(USART2, buf[i]);

      int timeout = 0;

      while (LL_USART_IsActiveFlag_TC(USART2) == 0) {
        if (timeout >= 50000)
          break;
        timeout++;
      }

      if (buf[i] == 10) {
        buf[99] = 0x00;
        if (i < 98)
          buf[i + 1] = 0x00;
        break;
      }
    }
    pos = lfs_file_tell(&lfs, &log_file);
    LL_IWDG_ReloadCounter(IWDG);
  }

  lfs_file_close(&lfs, &log_file);
  lfs_unmount(&lfs);
}

// this function waits for NMEA messages and parses it into the log item
// struct above
int get_gps_data() {

  static char buf[100] = {0};
  static uint32_t len = 100;
  static uint32_t i = 0;

  // MX_USART2_UART_Init();

  LL_USART_DisableOverrunDetect(USART2);
  LL_USART_ClearFlag_ORE(USART2);

  LL_USART_DisableDirectionRx(USART2);
  LL_USART_EnableDirectionRx(USART2);
  LL_USART_EnableDirectionTx(USART2);

  while (1) { // poll I2C bus until end of line is received

    int timeout = 0; // I2C software timeout counter

    // I2C RX
    LL_I2C_HandleTransfer(
        I2C1, (0x42 << 1), LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND,
        LL_I2C_GENERATE_START_READ); // request 1 byte from GPS

    while (LL_I2C_IsActiveFlag_RXNE(I2C1) == 0) { // wait for response
      if (timeout >= 20000) {
        return -1;
      }
      timeout++;
    }

    buf[i] = (char)LL_I2C_ReceiveData8(I2C1);

    /*
        // USART2 RX
        LL_USART_ClearFlag_ORE(USART2);

        while (LL_USART_IsActiveFlag_RXNE(USART2) == 0) { // wait for data
          if (timeout >= 1000000) {
            return -1;
          }
          timeout++;
        }
        buf[i] = (char)LL_USART_ReceiveData8(USART2);
    */
    /*
timeout = 0;

while (LL_USART_IsActiveFlag_TC(USART2) == 0) {
if (timeout >= 50000)
break;
timeout++;
}
    */

    if (buf[i] == '$') {
      i = 0;
      buf[i] = '$';
    }

    if (i >= (len - 1) || buf[i] == 10) {
      buf[len - 1] = 0x00;
      i = 0;

      if ((i + 1) < len - 1)
        buf[i + 1] = '\0';

      if (strncmp((buf + 3), "GGA", 3) == 0) { // parse GGA message

        // Print out GPS string over USART2 for debugging
        /*
                for (int i = 0; i <= 99; i++) {
          LL_USART_TransmitData8(USART2, buf[i]);
          int timeout = 0;

          while (LL_USART_IsActiveFlag_TC(USART2) == 0) {
            if (timeout >= 50000)
              // break;
              timeout++;
          }
          if (buf[i] == '\n')
            break;
        }
                                */

        write_buf_to_fs(&lfs, &cfg, &log_file, "flight_log", buf, strlen(buf));

        sscanf(buf, "%*[^,],%lu.%*lu,%ld.%lu,%c,%lu.%lu,%c,%*d,%*d,%*d,%lu",
               &(log_item.time), &(log_item.lat_int), &(log_item.lat_frac),
               &(log_item.lat_dir), &(log_item.lon_int), &(log_item.lon_frac),
               &(log_item.lon_dir), &(log_item.altitude));

        // move the decimal point left 2 for coords
        log_item.lat_frac =
            ((log_item.lat_int % 100) * count_revifs(log_item.lat_frac)) +
            log_item.lat_frac;
        log_item.lat_int = log_item.lat_int - (log_item.lat_int % 100);

        log_item.lon_frac =
            ((log_item.lon_int % 100) * count_revifs(log_item.lon_frac)) +
            log_item.lon_frac;
        log_item.lon_int = log_item.lon_int - (log_item.lon_int % 100);

        if (log_item.lat_dir == 'W')
          log_item.lat_int = log_item.lat_int * -1;

        if (log_item.lon_dir == 'S')
          log_item.lon_int = log_item.lon_int * -1;

        break;
      }

      if (strncmp((buf + 3), "RMC", 3) == 0) { // parse RMC message

        write_buf_to_fs(&lfs, &cfg, &log_file, "flight_log", buf, strlen(buf));

        sscanf(buf,
               "%*[^,],%lu,%*c,%ld.%lu,%c,%lu.%lu,%c,%*lu.%*lu,%*lu.%*lu,%lu",
               &(log_item.time), &(log_item.lat_int), &(log_item.lat_frac),
               &(log_item.lat_dir), &(log_item.lon_int), &(log_item.lon_frac),
               &(log_item.lon_dir), &(log_item.date));
      }
    }

    if (buf[i] != 0xFF)
      i++;
  }

  return 0;
}
