#ifndef _T1100_HELPER_H
#define _T1100_HELPER_H

#include "app_fatfs.h"

#include "stm32wlxx_ll_adc.h"
#include "stm32wlxx_ll_exti.h"
#include "stm32wlxx_ll_i2c.h"
#include "stm32wlxx_ll_rcc.h"
#include "stm32wlxx_ll_bus.h"
#include "stm32wlxx_ll_system.h"
#include "stm32wlxx_ll_cortex.h"
#include "stm32wlxx_ll_utils.h"
#include "stm32wlxx_ll_pwr.h"
#include "stm32wlxx_ll_dma.h"
#include "stm32wlxx_ll_tim.h"
#include "stm32wlxx_ll_gpio.h"
#include "stm32wlxx_ll_lptim.h"
#include "stm32wlxx_ll_lpuart.h"

// --- SPI1 SD Card Interface Definitions ---
// Define which GPIO pin is used as Chip Select (CS) for the SD card
// This pin is manually configured in MX_GPIO_Init()
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA		// Define which GPIO port that CS pin belongs to (PA4)

// Define the SPI handle used by the SD card driver
// This must match the SPI instance initialized in MX_SPI1_Init()
// and referenced by user_diskio_spi.c
#define SD_SPI_HANDLE hspi1

// -- Pin Definitions --
#define SPI1_CS ((gpio_t) {GPIOA, (uint32_t) 4})
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

	int32_t prev_altitude;
	int32_t altitude;
	int32_t ascent_rate;
	int32_t filtered_ascent_rate;

	uint32_t date;

	uint32_t prev_pressure;
	uint32_t pressure;
	int32_t temperature;
	int64_t press_altitude;

	int32_t board_temp;

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
int count_revifs(uint32_t n);
void pid_blocking(void);
void geofence_init(void);

// -- Logging Functions --

// Returns -1 on failure
int log_init(void);

int log_system_data(void);

// end logging functions

// -- Heater --
#define TEMP_ADDR 0x48
#define T_HIGH ((int) -10)
#define T_LOW ((int) -15)

// Sets board heater parameters T_high and T_low to the specified values given in degrees C
int set_heater_params(int t_high, int t_low);

// Reads board heater parameters T_high and T_low in degrees C
int read_heater_params(int *t_high, int *t_low);

// Returns the temperature value measured by the on-board thermo-couple
int read_heater_temp(void);
// end heater functions

// these two I2C functions are not good, the args are mostly unused but writing
// the signature like this made copying RP2040 code faster
int i2c_read_blocking(I2C_TypeDef *i2cx, uint8_t addr, uint8_t *buf,
		uint32_t bytes, uint32_t temp);
int i2c_write_blocking(I2C_TypeDef *i2cx, uint8_t addr, uint8_t *buf,
		uint32_t bytes, uint32_t temp);

#endif // _T1100_HELPER_H
