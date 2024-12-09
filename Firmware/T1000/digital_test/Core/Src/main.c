#include "main.h"

#include <math.h>
#include <string.h>

#include "MS5607.h"
#include "PID.h"
#include "fram.h"
#include "lfs.h"
#include "stm32_helper.h"
#include "sx1262.h"
#include "t1000_helper.h"

#define LOG_FLAG (0x01 << 0)
#define CUT_FLAG (0x01 << 1)
#define GEO_FLAG (0x01 << 2)

#define INT_TIM_EN_FLAG                                                        \
  (0x01 << 3) // if this flag is set, T1xxx will attempt to terminate after
              // 2*cut_int_timer minutes

#define GEO_EN_FLAG                                                            \
  (0x01 << 4) // if this flag is set, T1xxx will attempt to terminate when it
              // detects exit from the set geofence

#define DESC_FLAG (0x01 << 5) // indicates descent detected
#define HEAT_FLAG (0x01 << 6) // indicates descent detected

// Termination Logic parameters
#define ROUTINE_INTERVAL                                                       \
  30 // interval in number of seconds that the regular routine should run at
uint32_t int_count_30 = 0;   // 30 second interval counter as counted by LPTIM1
uint32_t cut_counter = 0;    // counts number of cut attempts
uint32_t cut_int_timer = 60; // number of minutes until cut

static uint32_t flags =
    INT_TIM_EN_FLAG | GEO_EN_FLAG; // flags used for a variety of purposes

fram_t memory; // off chip memory used to store non-volatile flight logs
sx1262_t radio = {SPI1, (gpio_t){GPIOB, LL_GPIO_PIN_0}, {0, 0}};
log_item_t log_item;

static char log_buf[100] = {0};

// -- LittleFS Configuration --
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
lfs_file_t log_file;
lfs_file_t int_count_save;
// end LittleFS

// -- PID System --
#define PID_KP 1
#define PID_KI 1
#define PID_KD 0

#define PID_TAU 20

#define PID_LIM_MIN 0
#define PID_LIM_MAX 90

#define PID_LIM_MIN_INT ((int32_t)(-100))
#define PID_LIM_MAX_INT 100
PIDController pid = {PID_KP,          PID_KI,          PID_KD,
                     PID_TAU,         PID_LIM_MIN,     PID_LIM_MAX,
                     PID_LIM_MIN_INT, PID_LIM_MAX_INT, 1};
// end PID

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  int err = 0;
  uint32_t boot_count = 0;
  uint16_t adc_value = 0;
  uint32_t isns_value = 0;
  uint32_t batt_value = 0;

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));

  SystemClock_Config();

  // Init Peripherals
  // USB_Init(); // needs testing
  MX_GPIO_Init();
  // MX_ADC1_Init();
  // MX_COMP1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_LPTIM1_Init();
  // MX_USART2_UART_Init();
  MX_IWDG_Init(); // init watchdog with 32 second timer

  // enable LPTIM1 which triggers interrupt every 30 seconds
  LL_LPTIM_Enable(LPTIM1);
  LL_LPTIM_EnableIT_ARRM(LPTIM1);
  LL_LPTIM_SetAutoReload(LPTIM1, (256 * ROUTINE_INTERVAL)); // 7680 = 30 seconds
  LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_CONTINUOUS);

  // Enable Peripherals
  // LL_ADC_Enable(ADC1);
  LL_I2C_Enable(I2C1);
  LL_SPI_Enable(SPI1);

  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_0); // Radio NSS high
  PA8_LOW
  PA6_LOW

  fram_init(&memory, SPI1, 0, 0, 0, 0);
  // dump_fram();

  // -- Update Saved Values --
  err = lfs_mount(&lfs, &cfg);

  if (err) {
    // NVIC_SystemReset(); // request reset
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
  // lfs_file_read(&lfs, &int_count_save, &int_count_30,
  // sizeof(int_count_30));
  lfs_file_close(&lfs, &int_count_save);
  lfs_unmount(&lfs);
  // end update saved values

  LL_IWDG_ReloadCounter(IWDG);

  // write boot and flags to flight log
  sprintf(log_buf, "%lu, boot\n", boot_count);
  write_buf_to_fs(&lfs, &cfg, &log_file, "flight_log", log_buf,
                  strlen(log_buf));

  sprintf(log_buf, "%lu, flags: 0x%lx\n", boot_count, flags);
  write_buf_to_fs(&lfs, &cfg, &log_file, "flight_log", log_buf,
                  strlen(log_buf));

  // read_log(); // dump log over USART2 on PA2

  err = 0;
  log_item.lat_dir = '0';
  log_item.lon_dir = '0';

  geofence_init();

  // ms5607_reset();
  ms5607_init(I2C1);
  ms5607_get_press_temp(&log_item.pressure, &log_item.temperature);

  // Enable ADC here since it shares pins with USART2 used in read_log()
  MX_ADC1_Init();
  LL_ADC_Enable(ADC1);

  // PID Init
  PIDController_Init(&pid);

  pid_blocking(); // little test function for characterizing power stage

  while (1) {

    // tight PI loop
    while ((flags & CUT_FLAG) &&
           !(LL_LPTIM_IsActiveFlag_ARRM(LPTIM1) || (flags & LOG_FLAG))) {

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

      if (log_item.pressure <
          100000) // total kludge since it's difficult to figure out if on
                  // battery or USB power right now, if pressure is less than
                  // 1000 mbar it's pretty likely that system is airborne
        batt_value = __LL_ADC_CALC_DATA_TO_VOLTAGE(3000, adc_value,
                                                   LL_ADC_RESOLUTION_12B);
      else
        batt_value = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, adc_value,
                                                   LL_ADC_RESOLUTION_12B);

      batt_value = (batt_value * 24) / 10;

      LL_ADC_REG_StartConversion(ADC1);

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
        isns_value = __LL_ADC_CALC_DATA_TO_VOLTAGE(3000, adc_value,
                                                   LL_ADC_RESOLUTION_12B);
      else
        isns_value = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, adc_value,
                                                   LL_ADC_RESOLUTION_12B);
      // end ADC read

      PIDController_Update(
          &pid, 1000,
          isns_value); // update PID controller with set point 1000mV
      LL_TIM_OC_SetCompareCH3(TIM1, pid.out);
    }

    // regular 30 second routine
    if (LL_LPTIM_IsActiveFlag_ARRM(LPTIM1) || (flags & LOG_FLAG)) {

      log_item.log_count++;
      int_count_30++;

      // if currently cutting, need to disable quickly
      if (flags & CUT_FLAG) {
        LL_TIM_OC_SetCompareCH3(TIM1, 0);
        LL_TIM_DisableCounter(TIM1);

        LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);

        sprintf(log_buf, "%lu, %lu, end cut %lu at %lu\n", boot_count,
                log_item.log_count, cut_counter, log_item.time);
        write_buf_to_fs(&lfs, &cfg, &log_file, "flight_log", log_buf,
                        strlen(log_buf));

        sprintf(log_buf, "%lu, %lu, %lu isns: %lu duty: %lu \n", boot_count,
                log_item.log_count, cut_counter, isns_value, pid.out);
        write_buf_to_fs(&lfs, &cfg, &log_file, "flight_log", log_buf,
                        strlen(log_buf));

        cut_counter++;

        flags = flags & ~(CUT_FLAG);
      }

      // -- Pressure/Temp Conversion --
      log_item.prev_pressure = log_item.pressure;
      if (ms5607_get_press_temp(&log_item.pressure, &log_item.temperature) ==
          -1)
        sprintf(log_buf, "%lu, %lu, p/t conv fail\n", boot_count,
                log_item.log_count);
      else {
        sprintf(log_buf, "%lu,%lu,%lu,%ld\n", boot_count, log_item.log_count,
                log_item.pressure, log_item.temperature);
      }

      write_buf_to_fs(&lfs, &cfg, &log_file, "flight_log", log_buf,
                      strlen(log_buf));
      // end p/t

      // Save pressure/temp data and current interval count
      err = lfs_mount(&lfs, &cfg);

      if (err) {
        exit(-1); // trouble mounting FS
        // lfs_format(&lfs, &cfg);
        // lfs_mount(&lfs, &cfg);
      }

      // update 30 seond interval count
      update_int_count(int_count_30);

      // -- GPS read --
      log_item.prev_altitude = log_item.altitude;

      if (get_gps_data() < 0) {
        // GPS parse failed
        sprintf(log_buf, "%lu, %lu, GPS parse failed\n", boot_count,
                log_item.log_count);
        log_buf[99] = 0x00;
      } else {

        sprintf(log_buf, "%lu,%lu,%lu,%lu.%lu,%c,%lu.%lu,%c,%lu,%lu \n",
                boot_count, log_item.log_count, log_item.time, log_item.lat_int,
                log_item.lat_frac, log_item.lat_dir, log_item.lon_int,
                log_item.lon_frac, log_item.lon_dir, log_item.altitude,
                log_item.date);
        log_buf[99] = 0x00;
      }

      write_buf_to_fs(&lfs, &cfg, &log_file, "flight_log", log_buf,
                      strlen(log_buf));
      // end GPS read

      // -- Read ADC --
      LL_ADC_REG_StartConversion(ADC1);

      // wait end of conversion flag
      while (!LL_ADC_IsActiveFlag_EOC(ADC1))
        ;

      adc_value = LL_ADC_REG_ReadConversionData12(ADC1);

      // clear flag
      LL_ADC_ClearFlag_EOC(ADC1);

      if (log_item.pressure <
          100000) // total kludge since it's difficult to figure out if on
                  // battery or USB power right now, if pressure is less than
                  // 1000 mbar it's pretty likely that system is airborne
        batt_value = __LL_ADC_CALC_DATA_TO_VOLTAGE(3000, adc_value,
                                                   LL_ADC_RESOLUTION_12B);
      else
        batt_value = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, adc_value,
                                                   LL_ADC_RESOLUTION_12B);

      batt_value = (batt_value * 24) / 10;
      log_item.batt_v = batt_value;

      // do another conversion to get back to the start of the sequence
      LL_ADC_REG_StartConversion(ADC1);
      // wait end of conversion flag
      while (!LL_ADC_IsActiveFlag_EOC(ADC1))
        ;
      // clear flag
      LL_ADC_ClearFlag_EOC(ADC1);

      sprintf(log_buf, "%lu,%lu,batt %lu\n", boot_count, log_item.log_count,
              log_item.batt_v);

      write_buf_to_fs(&lfs, &cfg, &log_file, "flight_log", log_buf,
                      strlen(log_buf));
      // end ADC read

      // -- Thermal Management --
      if ((log_item.temperature) <= -2000 && !(flags & HEAT_FLAG)) {
        // turn on heater?
        PA6_HIGH

        // Enable PWM channel outputs
        LL_TIM_EnableCounter(TIM1);
        LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
        LL_TIM_OC_SetCompareCH3(TIM1, 15);
        LL_TIM_EnableAllOutputs(TIM1);

        sprintf(log_buf, "%lu,%lu, HEAT ON, temp is %lu at %lu\n", boot_count,
                log_item.log_count, log_item.temperature, log_item.time);
        write_buf_to_fs(&lfs, &cfg, &log_file, "flight_log", log_buf,
                        strlen(log_buf));

        flags = flags | HEAT_FLAG;

      } else if ((log_item.temperature) >= -1500 && (flags & HEAT_FLAG)) {
        PA6_LOW

        LL_TIM_OC_SetCompareCH3(TIM1, 0);
        LL_TIM_DisableCounter(TIM1);

        sprintf(log_buf, "%lu,%lu, HEAT OFF, temp is %lu at %lu\n", boot_count,
                log_item.log_count, log_item.temperature, log_item.time);
        write_buf_to_fs(&lfs, &cfg, &log_file, "flight_log", log_buf,
                        strlen(log_buf));

        flags = flags & ~(HEAT_FLAG);
      }
      // end thermal management

      // -- Termination Logic --
      if (log_item.prev_pressure <
          log_item.pressure) { // if pressure increased since last
                               // measurement, might be descending, would be
                               // more accurate with more historical data
        if (log_item.pressure - log_item.prev_pressure >= 1000) {

          sprintf(log_buf, "%lu, %lu, DESC, press diff is %lu at %lu\n",
                  boot_count, log_item.log_count,
                  (log_item.pressure - log_item.prev_pressure), log_item.time);

          write_buf_to_fs(&lfs, &cfg, &log_file, "flight_log", log_buf,
                          strlen(log_buf));

          // set the flag which we're not doing right now
        }
      }

      if (log_item.prev_altitude >
          log_item.altitude) { // if altitude descreased, might be descending
        if (log_item.prev_altitude - log_item.altitude >= 900) {

          sprintf(log_buf, "%lu,%lu, DESC, altitude diff is %lu at %lu\n",
                  boot_count, log_item.log_count,
                  (log_item.pressure - log_item.prev_pressure), log_item.time);

          write_buf_to_fs(&lfs, &cfg, &log_file, "flight_log", log_buf,
                          strlen(log_buf));

          // set the flag which we're not doing right now
        }
      }

      if (cut_counter < 4) { // limit total number of cut attempts

        if ((int_count_30 >= (cut_int_timer * 2)) &&
            (flags & INT_TIM_EN_FLAG)) { // timer trigger

          sprintf(log_buf, "%lu,%lu, cut %lu int trig: int cnt at %lu\n",
                  boot_count, log_item.log_count, cut_counter, log_item.time);

          write_buf_to_fs(&lfs, &cfg, &log_file, "flight_log", log_buf,
                          strlen(log_buf));

          flags = flags | CUT_FLAG;
        }

        if ((log_item.temperature <= -3000) &&
            (flags & INT_TIM_EN_FLAG)) { // temperature trigger

          sprintf(log_buf, "%lu,%lu, cut %lu temp trig: int cnt at %lu\n",
                  boot_count, log_item.log_count, cut_counter, log_item.time);

          write_buf_to_fs(&lfs, &cfg, &log_file, "flight_log", log_buf,
                          strlen(log_buf));

          flags = flags | CUT_FLAG;
        }
      }

      if ((flags & CUT_FLAG) &&
          (flags &
           ~(DESC_FLAG))) { // a trigger set cut flag AND descent not detected

        // disable thermal management
        PA6_LOW

        // Enable PWM channel outputs
        LL_TIM_EnableCounter(TIM1);
        LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
        LL_TIM_OC_SetCompareCH3(TIM1, 0);
        LL_TIM_EnableAllOutputs(TIM1);

        LL_ADC_Disable(ADC1);
        MX_ADC1_Init();
        LL_ADC_Enable(ADC1);

        sprintf(log_buf, "%lu, start cut %lu at %lu\n", boot_count, cut_counter,
                log_item.time);
        write_buf_to_fs(&lfs, &cfg, &log_file, "flight_log", log_buf,
                        strlen(log_buf));
      }
      // end Termination Logic

      // clean up loop
      flags = flags & ~(LOG_FLAG);
      LL_LPTIM_ClearFLAG_ARRM(LPTIM1);
    }

    LL_IWDG_ReloadCounter(IWDG);
  }
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
  /* USER CODE BEGIN Error_Handler_Debug
   * User can add their own implementation to report the HAL error return state
   */
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
