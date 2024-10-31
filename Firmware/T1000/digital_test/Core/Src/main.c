#include "main.h"

#include <string.h>

#include "MS5607.h"
#include "fram.h"
#include "lfs.h"
#include "stm32_helper.h"

#define PA8_HIGH LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);
#define PA8_LOW LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_COMP1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_LPTIM1_Init(void);

static uint32_t read_adc1(void);
int get_gps_data(void);
void read_log(void);

#define LOG_FLAG 0x01

static uint32_t flags = 0;

fram_t memory;

struct _log_item {
  uint32_t time;

  uint32_t lat_int;
  uint32_t lat_frac;
  char lat_dir;

  uint32_t lon_int;
  uint32_t lon_frac;
  char lon_dir;

  uint32_t altitude;

  uint32_t date;

  uint32_t pressure;
  int32_t temperature;
} log_item;

static char log_buf[100] = {0};
static unsigned int log_count = 0;

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

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));

  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_COMP1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_LPTIM1_Init();

  // Enable PWM channel outputs
  // LL_TIM_EnableCounter(TIM1);
  // LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
  // LL_TIM_OC_SetCompareCH3(TIM1, 50); // change this
  // LL_TIM_EnableAllOutputs(TIM1);

  // enable LPTIM1 which triggers interrupt every ~30 seconds
  LL_LPTIM_Enable(LPTIM1);
  LL_LPTIM_EnableIT_ARRM(LPTIM1);
  LL_LPTIM_SetAutoReload(LPTIM1, 8533);
  LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_CONTINUOUS);

  // Enable Peripherals
  LL_ADC_Enable(ADC1);
  LL_I2C_Enable(I2C1);
  LL_SPI_Enable(SPI1);

  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_0); // Radio NSS high
  PA8_LOW

  fram_init(&memory, SPI1, 0, 0, 0, 0);

  read_log();

  int err = lfs_mount(&lfs, &cfg);

  if (err) {
    exit(-1);
    // err = lfs_format(&lfs, &cfg);
    // err = lfs_mount(&lfs, &cfg);
  }

  // update boot count
  uint32_t boot_count = 0;
  lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
  lfs_file_read(&lfs, &file, &boot_count, sizeof(boot_count));
  boot_count += 1;
  lfs_file_rewind(&lfs, &file);
  lfs_file_write(&lfs, &file, &boot_count, sizeof(boot_count));
  lfs_file_close(&lfs, &file);

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

  // ms5607_reset();
  ms5607_init(I2C1);
  ms5607_get_press_temp(&log_item.pressure, &log_item.temperature);

  while (1) {

    if (LL_LPTIM_IsActiveFlag_ARRM(LPTIM1) ||
        (flags & LOG_FLAG)) { // regular logging event

      PA8_HIGH
      log_count++;

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
        sprintf(log_buf, "%lu, GPS parse failed", boot_count);
        log_buf[99] = 0x00;
      } else {

        sprintf(log_buf, "%lu,%u,%lu,%lu.%lu,%c,%lu.%lu,%c,%lu,%lu \n",
                boot_count, log_count, log_item.time, log_item.lat_int,
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

  while (1) { // poll I2C bus until end of line is received

    int timeout = 0; // I2C software timeout counter

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

    if (buf[i] == '$') {
      i = 0;
      buf[i] = '$';
    }

    if (i >= (len - 1) || buf[i] == 10) {
      buf[len - 1] = 0x00;
      i = 0;

      if (strncmp((buf + 3), "GGA", 3) == 0) { // parse GGA message

        sscanf(buf, "%*[^,],%lu.%*lu,%lu.%lu,%c,%lu.%lu,%c,%*d,%*d,%*d,%lu",
               &(log_item.time), &(log_item.lat_int), &(log_item.lat_frac),
               &(log_item.lat_dir), &(log_item.lon_int), &(log_item.lon_frac),
               &(log_item.lon_dir), &(log_item.altitude));
        break;
      }

      if (strncmp((buf + 3), "RMC", 3) == 0) { // parse RMC message

        sscanf(buf,
               "%*[^,],%lu,%*c,%lu.%lu,%c,%lu.%lu,%c,%*lu.%*lu,%*lu.%*lu,%lu",
               &(log_item.time), &(log_item.lat_int), &(log_item.lat_frac),
               &(log_item.lat_dir), &(log_item.lon_int), &(log_item.lon_frac),
               &(log_item.lon_dir), &(log_item.altitude), &(log_item.date));
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

/**
 * @brief System Clock Configuration
 * @retval None
 */

void SystemClock_Config(void) {
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1) {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  while (LL_PWR_IsActiveFlag_VOS() != 0) {
  }
  LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();

  /* Wait till HSE is ready */
  while (LL_RCC_HSE_IsReady() != 1) {
  }
  LL_RCC_LSI_Enable();

  /* Wait till LSI is ready */
  while (LL_RCC_LSI_IsReady() != 1) {
  }
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);

  /* Wait till System clock is ready */
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSE) {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_Init1msTick(25000000);

  LL_SetSystemCoreClock(25000000);
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA2   ------> ADC1_IN7
  PA3   ------> ADC1_IN8
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
   */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV1;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
                                 LL_ADC_PATH_INTERNAL_NONE);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC1);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US *
                      (SystemCoreClock / (100000 * 2))) /
                     10);
  while (wait_loop_index != 0) {
    wait_loop_index--;
  }

  /** Configure Regular Channel
   */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_7);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_7,
                                LL_ADC_SAMPLINGTIME_2CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_7, LL_ADC_SINGLE_ENDED);
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief COMP1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_COMP1_Init(void) {

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  LL_COMP_InitTypeDef COMP_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**COMP1 GPIO Configuration
  PA1   ------> COMP1_INP
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  COMP_InitStruct.PowerMode = LL_COMP_POWERMODE_HIGHSPEED;
  COMP_InitStruct.InputPlus = LL_COMP_INPUT_PLUS_IO3;
  COMP_InitStruct.InputMinus = LL_COMP_INPUT_MINUS_1_2VREFINT;
  COMP_InitStruct.InputHysteresis = LL_COMP_HYSTERESIS_NONE;
  COMP_InitStruct.OutputPolarity = LL_COMP_OUTPUTPOL_NONINVERTED;
  COMP_InitStruct.OutputBlankingSource = LL_COMP_BLANKINGSRC_NONE;
  LL_COMP_Init(COMP1, &COMP_InitStruct);

  /* Wait loop initialization and execution */
  /* Note: Variable divided by 2 to compensate partially CPU processing cycles
   */
  __IO uint32_t wait_loop_index = 0;
  wait_loop_index = (LL_COMP_DELAY_VOLTAGE_SCALER_STAB_US *
                     (SystemCoreClock / (1000000 * 2)));
  while (wait_loop_index != 0) {
    wait_loop_index--;
  }
  LL_EXTI_DisableEvent_0_31(LL_EXTI_LINE_21);
  LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_21);
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_PCLK1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
   */
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x004018D5;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA7   ------> SPI1_MOSI
  PB4 (NJTRST)   ------> SPI1_MISO
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5 | LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_EnableNSSPulseMgt(SPI1);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 100;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetOCRefClearInputSource(TIM1, LL_TIM_OCREF_CLR_INT_NC);
  LL_TIM_DisableExternalClock(TIM1);
  LL_TIM_ConfigETR(TIM1, LL_TIM_ETR_POLARITY_NONINVERTED,
                   LL_TIM_ETR_PRESCALER_DIV1, LL_TIM_ETR_FILTER_FDIV1);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
  TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_DISABLE;
  TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH;
  TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**TIM1 GPIO Configuration
  PB1   ------> TIM1_CH3N
  PA10   ------> TIM1_CH3
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);

  /*Configure GPIO Outputs*/
  GPIO_InitStruct.Pin =
      (LL_GPIO_PIN_4 | LL_GPIO_PIN_6 | LL_GPIO_PIN_8 | LL_GPIO_PIN_15);
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/**
 * @brief LPTIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_LPTIM1_Init(void) {

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM1_CLKSOURCE_LSI);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);

  /* LPTIM1 interrupt Init */
  NVIC_SetPriority(LPTIM1_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  // NVIC_EnableIRQ(LPTIM1_IRQn);

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  LL_LPTIM_SetClockSource(LPTIM1, LL_LPTIM_CLK_SOURCE_INTERNAL);
  LL_LPTIM_SetPrescaler(LPTIM1, LL_LPTIM_PRESCALER_DIV128);
  LL_LPTIM_SetPolarity(LPTIM1, LL_LPTIM_OUTPUT_POLARITY_REGULAR);
  LL_LPTIM_SetUpdateMode(LPTIM1, LL_LPTIM_UPDATE_MODE_IMMEDIATE);
  LL_LPTIM_SetCounterMode(LPTIM1, LL_LPTIM_COUNTER_MODE_INTERNAL);
  LL_LPTIM_TrigSw(LPTIM1);
  LL_LPTIM_SetInput1Src(LPTIM1, LL_LPTIM_INPUT1_SRC_GPIO);
  LL_LPTIM_SetInput2Src(LPTIM1, LL_LPTIM_INPUT2_SRC_GPIO);
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */
}

void LPTIM1_IRQHandler(void) {
  flags = flags | LOG_FLAG; // set log flag to signal to main loop to do a log

  LL_TIM_EnableCounter(TIM1);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
  LL_TIM_OC_SetCompareCH3(TIM1, 50); // change this
  LL_TIM_EnableAllOutputs(TIM1);

  LL_LPTIM_ClearFLAG_ARRM(LPTIM1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
