#include "main.h"

#include <string.h>

#include "MS5607.h"
#include "PID.h"
#include "fram.h"
#include "lfs.h"
#include "stm32_helper.h"

#define PA8_HIGH LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);
#define PA8_LOW LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);

static uint32_t read_adc1(void);
int get_gps_data(void);
void read_log(void);
static int count_revifs(uint32_t n);

#define LOG_FLAG (0x01 << 0)
#define CUT_FLAG (0x01 << 1)
#define GEO_FLAG (0x01 << 2)

uint32_t int_count_30 = 0; // 30 second interval counter as counted by LPTIM1
uint32_t cut_counter = 0;  // counts number of cut attempts

static uint32_t flags = 0;

fram_t memory;

struct _log_item {
  uint32_t time;

  int32_t lat_int;
  uint32_t lat_frac;
  char lat_dir;

  int32_t lon_int;
  uint32_t lon_frac;
  char lon_dir;

  uint32_t altitude;

  uint32_t date;

  uint32_t pressure;
  int32_t temperature;

  uint32_t log_count;
} log_item;

typedef struct _geo_vertex {

  int32_t lat_int;
  uint32_t lat_frac;
  char lat_dir;

  int32_t lon_int;
  uint32_t lon_frac;
  char lon_dir;

} geo_vertex;

geo_vertex empty_vertex = {0, 0, '0', 0, 0, '0'};

struct _geo_fence {

  geo_vertex top_left;
  geo_vertex bot_left;
  geo_vertex top_right;
  geo_vertex bot_right;

  uint32_t altitude;

} geo_fence;

static char log_buf[100] = {0};

// configuration of the filesystem is provided by this struct
const struct lfs_config cfg = {
    // block device operations
    .read = fs_flash_read,
    .prog = fs_flash_prog,
    .erase = fs_flash_erase,
    .sync = fs_flash_sync,

    // block device configuration
    .read_size = 1,
    .prog_size = 1,
    .block_size = 128,
    .block_count = 1024, // 64
    .cache_size = 1,
    .lookahead_size = 16,
    .block_cycles = -1,
};

// variables used by the filesystem
lfs_t lfs;
lfs_file_t file;
lfs_file_t log;
lfs_file_t int_count_save;

// PID System

#define PID_KP 1
#define PID_KI 1
#define PID_KD 0

#define PID_TAU 20

#define PID_LIM_MIN 0
#define PID_LIM_MAX 90

#define PID_LIM_MIN_INT -100
#define PID_LIM_MAX_INT 100

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  int err = 0;
  uint32_t boot_count = 0;
  uint16_t adcValCh1 = 0;
  uint32_t realValue = 0;

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));

  SystemClock_Config();

  // USB_Init(); // needs testing

  MX_GPIO_Init();
  MX_ADC1_Init();
  // MX_COMP1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_LPTIM1_Init();

  // enable LPTIM1 which triggers interrupt every 30 seconds
  LL_LPTIM_Enable(LPTIM1);
  LL_LPTIM_EnableIT_ARRM(LPTIM1);
  LL_LPTIM_SetAutoReload(LPTIM1, 7680); // 8533
  LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_CONTINUOUS);

  // Enable Peripherals
  LL_ADC_Enable(ADC1);
  LL_I2C_Enable(I2C1);
  LL_SPI_Enable(SPI1);

  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_0); // Radio NSS high
  PA8_LOW

  fram_init(&memory, SPI1, 0, 0, 0, 0);

  read_log();

  err = lfs_mount(&lfs, &cfg);

  if (err) {
    exit(-1);
    // err = lfs_format(&lfs, &cfg);
    // err = lfs_mount(&lfs, &cfg);
  }

  // update boot count
  lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
  lfs_file_read(&lfs, &file, &boot_count, sizeof(boot_count));
  boot_count += 1;
  lfs_file_rewind(&lfs, &file);
  lfs_file_write(&lfs, &file, &boot_count, sizeof(boot_count));
  lfs_file_close(&lfs, &file);

  // update 30 seond interval count
  lfs_file_open(&lfs, &int_count_save, "int_count_30",
                LFS_O_RDWR | LFS_O_CREAT);
  // lfs_file_read(&lfs, &int_count_save, &int_count_30, sizeof(int_count_30));
  lfs_file_close(&lfs, &int_count_save);

  // write boot message to flight log
  lfs_file_open(&lfs, &log, "flight_log",
                LFS_O_RDWR | LFS_O_CREAT | LFS_O_APPEND);
  sprintf(log_buf, "%lu, Boot\n", boot_count);
  log_buf[99] = 0x00;
  lfs_file_write(&lfs, &log, log_buf, strlen(log_buf));
  lfs_file_close(&lfs, &log);

  lfs_unmount(&lfs);

  err = 0;

  log_item.lat_dir = '0';
  log_item.lon_dir = '0';

  // set geofence parameters
  geo_fence.top_left = empty_vertex;
  geo_fence.bot_left = empty_vertex;
  geo_fence.top_right = empty_vertex;
  geo_fence.bot_right = empty_vertex;
  geo_fence.altitude = 0;

  // ms5607_reset();
  ms5607_init(I2C1);
  ms5607_get_press_temp(&log_item.pressure, &log_item.temperature);

  // PID Init
  PIDController pid = {PID_KP,          PID_KI,          PID_KD,
                       PID_TAU,         PID_LIM_MIN,     PID_LIM_MAX,
                       PID_LIM_MIN_INT, PID_LIM_MAX_INT, 0};

  PIDController_Init(&pid);

  while (1) {

    // tight PI loop
    while ((flags | CUT_FLAG) &&
           !(LL_LPTIM_IsActiveFlag_ARRM(LPTIM1) || (flags & LOG_FLAG))) {

      // wait end of conversion flag
      while (!LL_ADC_IsActiveFlag_EOC(ADC1))
        ;

      // clear flag
      LL_ADC_ClearFlag_EOC(ADC1);

      // read channel1 data
      adcValCh1 = LL_ADC_REG_ReadConversionData12(ADC1);

      realValue =
          __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, adcValCh1, LL_ADC_RESOLUTION_12B);

      PIDController_Update(&pid, 500, realValue);
      LL_TIM_OC_SetCompareCH3(TIM1, pid.out);
    }

    // regular 30 second routine
    if (LL_LPTIM_IsActiveFlag_ARRM(LPTIM1) || (flags & LOG_FLAG)) {

      PA8_HIGH
      log_item.log_count++;
      int_count_30++;

      // -- Cut Attempt --
      if ((int_count_30 >= (1 * 2)) && (cut_counter < 4)) {
        if (flags & CUT_FLAG) { // currently cutting, need to disable quickly

          LL_TIM_OC_SetCompareCH3(TIM1, 0);
          LL_TIM_DisableCounter(TIM1);

          LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);

          sprintf(log_buf, "%lu, end cut %lu at %lu\n", boot_count, cut_counter,
                  log_item.time);

          write_buf_to_fs(&lfs, &cfg, &log, "flight_log", log_buf,
                          strlen(log_buf));
          cut_counter++;

          flags = flags & ~(CUT_FLAG);

        } else { // not currently cutting

          LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_CONTINUOUS);
          LL_ADC_REG_StartConversion(ADC1);

          // Enable PWM channel outputs
          LL_TIM_EnableCounter(TIM1);
          LL_TIM_CC_EnableChannel(TIM1,
                                  LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
          LL_TIM_OC_SetCompareCH3(TIM1, 0);
          LL_TIM_EnableAllOutputs(TIM1);

          sprintf(log_buf, "%lu, start cut %lu at %lu\n", boot_count,
                  cut_counter, log_item.time);

          write_buf_to_fs(&lfs, &cfg, &log, "flight_log", log_buf,
                          strlen(log_buf));

          flags = flags | CUT_FLAG;
        }
      }

      // update 30 seond interval count
      lfs_file_open(&lfs, &int_count_save, "int_count_30",
                    LFS_O_RDWR | LFS_O_CREAT);
      lfs_file_rewind(&lfs, &int_count_save);
      lfs_file_write(&lfs, &int_count_save, &int_count_30,
                     sizeof(int_count_30));
      lfs_file_close(&lfs, &int_count_save);

      // -- Pressure Temp Conversion --
      if (ms5607_get_press_temp(&log_item.pressure, &log_item.temperature) ==
          -1)
        sprintf(log_buf, "%lu, press/temp conv fail\n", boot_count);
      else
        sprintf(log_buf, "%lu,%lu,%ld\n", boot_count, log_item.pressure,
                log_item.temperature);

      err = lfs_mount(&lfs, &cfg);

      if (err) {
        exit(-1); // trouble mounting FS
        // lfs_format(&lfs, &cfg);
        // lfs_mount(&lfs, &cfg);
      }

      lfs_file_open(&lfs, &log, "flight_log",
                    LFS_O_RDWR | LFS_O_CREAT | LFS_O_APPEND);

      lfs_file_write(&lfs, &log, log_buf, strlen(log_buf));

      lfs_file_close(&lfs, &log);
      lfs_unmount(&lfs);

      // -- GPS read --
      if (get_gps_data() < 0) {
        // GPS parse failed
        sprintf(log_buf, "%lu, GPS parse failed\n", boot_count);
        log_buf[99] = 0x00;
      } else {

        sprintf(log_buf, "%lu,%lu,%lu,%lu.%lu,%c,%lu.%lu,%c,%lu,%lu \n",
                boot_count, log_item.log_count, log_item.time, log_item.lat_int,
                log_item.lat_frac, log_item.lat_dir, log_item.lon_int,
                log_item.lon_frac, log_item.lon_dir, log_item.altitude,
                log_item.date);
        log_buf[99] = 0x00;
      }

      err = lfs_mount(&lfs, &cfg);

      if (err) {
        exit(-1); // trouble mounting FS
        // lfs_format(&lfs, &cfg);
        // lfs_mount(&lfs, &cfg);
      }

      lfs_file_open(&lfs, &log, "flight_log",
                    LFS_O_RDWR | LFS_O_CREAT | LFS_O_APPEND);

      lfs_file_write(&lfs, &log, log_buf, strlen(log_buf));

      lfs_file_close(&lfs, &log);
      lfs_unmount(&lfs);

      // clean up loop
      flags = flags & ~(LOG_FLAG);
      PA8_LOW
      LL_LPTIM_ClearFLAG_ARRM(LPTIM1);
    }
  }
}

// this function waits for a NMEA GGA message and parses it into the log item
// struct above
int get_gps_data() {

  static char buf[100] = {0};
  static uint32_t len = 100;
  static uint32_t i = 0;

  // USART2 Support on GPIO headers
  LL_USART_EnableDirectionRx(USART2);

  while (1) { // poll I2C bus until end of line is received

    int timeout = 0; // I2C software timeout counter

    /* // I2C RX
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
    */

    // USART2 RX
    while (LL_USART_IsActiveFlag_RXNE(USART2) == 0) { // wait for data
      if (timeout >= 20000) {
        return -1;
      }
      timeout++;
    }
    buf[i] = (char)LL_USART_ReceiveData8(USART2);

    if (buf[i] == '$') {
      i = 0;
      buf[i] = '$';
    }

    if (i >= (len - 1) || buf[i] == 10) {
      buf[len - 1] = 0x00;
      i = 0;

      if (strncmp((buf + 3), "GGA", 3) == 0) { // parse GGA message

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

// read the log file line by line
void read_log() {

  static char buf[100] = {0};
  uint32_t file_size = 0;

  int err = lfs_mount(&lfs, &cfg);

  if (err) {
    exit(-1); // trouble mounting FS
    // lfs_format(&lfs, &cfg);
    // lfs_mount(&lfs, &cfg);
  }

  lfs_file_open(&lfs, &log, "flight_log", LFS_O_RDONLY);
  lfs_file_rewind(&lfs, &log);

  file_size = lfs_file_size(&lfs, &log);

  uint32_t pos = 0;

  while (lfs_file_tell(&lfs, &log) < lfs_file_size(&lfs, &log)) {
    for (int i = 0; i < 99; i++) {
      lfs_file_read(&lfs, &log, (buf + i), 1);
      if (buf[i] == 10) {
        buf[99] = 0x00;
        if (i < 98)
          buf[i + 1] = 0x00;
        break;
      }
    }
    pos = lfs_file_tell(&lfs, &log);
  }

  lfs_file_close(&lfs, &log);
  lfs_unmount(&lfs);
}

void LPTIM1_IRQHandler(void) {
  flags = flags | LOG_FLAG; // set log flag to signal to main loop to do a log

  // LL_TIM_EnableCounter(TIM1);
  // LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
  // LL_TIM_OC_SetCompareCH3(TIM1, 50); // change this
  //  LL_TIM_EnableAllOutputs(TIM1);

  LL_LPTIM_ClearFLAG_ARRM(LPTIM1);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

static uint32_t read_adc1(void) {
  // no idea how to read multiple channels actually?

  uint16_t adcValCh1 = 0;
  uint32_t realValue = 0;

  // start conversion
  LL_ADC_REG_StartConversion(ADC1);

  // wait end of conversion flag
  while (!LL_ADC_IsActiveFlag_EOC(ADC1))
    ;

  // clear flag
  LL_ADC_ClearFlag_EOC(ADC1);

  // read channel1 data
  adcValCh1 = LL_ADC_REG_ReadConversionData12(ADC1);

  realValue =
      __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, adcValCh1, LL_ADC_RESOLUTION_12B);

  return realValue;
}

void USB_Init() {
  // Initialize the NVIC
  NVIC_SetPriority(USB_IRQn, 8);
  NVIC_EnableIRQ(USB_IRQn);

  // Enable USB macrocell
  USB->CNTR &= ~USB_CNTR_PDWN;

  // Wait 1μs until clock is stable
  SysTick->LOAD = 100;
  SysTick->VAL = 0;
  SysTick->CTRL = 1;
  while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) {
  }
  SysTick->CTRL = 0;

  // Enable all interrupts & the internal pullup to put 1.5K on D+ for FullSpeed
  // USB
  USB->CNTR |= USB_CNTR_RESETM | USB_CNTR_CTRM;
  USB->BCDR |= USB_BCDR_DPPU;

  // Clear the USB Reset (D+ & D- low) to start enumeration
  USB->CNTR &= ~USB_CNTR_FRES;
}

void USB_IRQHandler() {}

// counts number of digits in uint32_t, returns anywhere from 1-7
static int count_revifs(uint32_t n) {
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

static int check_geo_fence() { return 0; }
