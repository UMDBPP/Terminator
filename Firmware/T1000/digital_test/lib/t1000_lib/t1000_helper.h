#ifndef _T1000_HELPER_H
#define _T1000_HELPER_H

#include "stm32l4xx_ll_adc.h"
#include "stm32l4xx_ll_comp.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_i2c.h"
#include "stm32l4xx_ll_crs.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_spi.h"
#include "stm32l4xx_ll_tim.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_lptim.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_iwdg.h"

// -- Pin Definitions --
#define I2C1_SDA ((gpio_t) {GPIOB, (uint32_t) 7})
#define I2C1_SCL ((gpio_t) {GPIOB, (uint32_t) 6})
// end pin definitions

typedef struct _log_item_t {
  uint32_t time;

  int32_t lat_int;
  uint32_t lat_frac;
  char lat_dir;

  int32_t lon_int;
  uint32_t lon_frac;
  char lon_dir;

  uint32_t prev_altitude;
  uint32_t altitude;

  uint32_t date;

  uint32_t prev_pressure;
  uint32_t pressure;
  int32_t temperature;
  int64_t press_altitude;

  uint32_t log_count;

  uint32_t batt_v; // in mV
} log_item_t;

typedef struct _geo_vertex {

  int32_t lat_int;
  uint32_t lat_frac;
  char lat_dir;

  int32_t lon_int;
  uint32_t lon_frac;
  char lon_dir;

} geo_vertex;

void update_int_count(uint32_t count);
uint32_t read_adc1(void);
int get_gps_data(void);
void read_log(void);
int count_revifs(uint32_t n);
void dump_fram(void);
void pid_blocking(void);
void geofence_init(void);


#endif // _T1000_HELPER_H
