#include "t1100_helper.h"

#include <string.h>
#include <stdio.h>

#include "PID.h"
#include "stm32_helper.h"
#include "sx1262.h"

extern uint32_t int_count_30;
extern uint32_t cut_counter;
extern uint32_t cut_int_timer;

extern sx1262_t radio;
extern log_item_t log_item;

extern PIDController pid;

geo_vertex empty_vertex = { 0, 0, '0', 0, 0, '0' };

// -- Logging Variables --
// Declaring FatFS related objects.; filesystem, file object for general use,
// and result code for FatFS ops.
FATFS fs;
FIL file;

// Creates file object for CSV log
// Object for number of bytes written
FIL csvFile;
UINT bytes_written;
char csv_line[256];   // Buffer for formatted CSV row
// end logging variables

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

void pid_blocking(void) {
	uint16_t adc_value = 0;
	uint32_t isns_value = 0;
	uint32_t batt_value = 0;

	// Enable PWM channel outputs
	LL_TIM_EnableCounter(TIM1);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
	LL_TIM_OC_SetCompareCH3(TIM1, 0);
	LL_TIM_EnableAllOutputs(TIM1);

	//LL_ADC_Disable(ADC1);
	//MX_ADC1_Init();
	//LL_ADC_Enable(ADC1);

	PA8_LOW

	while (1) {

		PA8_TOGGLE

		//LL_IWDG_ReloadCounter( IWDG);

		// -- Read ADC --
		LL_ADC_REG_StartConversion(ADC);

		// wait end of conversion flag
		/*
		 while (!LL_ADC_IsActiveFlag_EOC(ADC))
		 ;
		 */

		// read channel1 data in mV
		//adc_value = LL_ADC_REG_ReadConversionData12(ADC);
		// clear flag
		//LL_ADC_ClearFlag_EOC(ADC1);
		//LL_ADC_REG_StartConversion(ADC1);
		if (log_item.pressure < 100000) // total kludge since it's difficult to figure out if on
										// battery or USB power right now, if pressure is less than
										// 1000 mbar it's pretty likely that system is airborne
			batt_value = __LL_ADC_CALC_DATA_TO_VOLTAGE(3000, adc_value,
					LL_ADC_RESOLUTION_12B);
		else
			batt_value = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, adc_value,
					LL_ADC_RESOLUTION_12B);

		batt_value = (batt_value * 24) / 10;

		// wait end of conversion flag
		while (!LL_ADC_IsActiveFlag_EOC(ADC))
			;

		// read channel1 data in mV
		adc_value = LL_ADC_REG_ReadConversionData12(ADC);

		// clear flag
		LL_ADC_ClearFlag_EOC(ADC);

		if (log_item.pressure < 100000) // total kludge since it's difficult to figure out if on
										// battery or USB power right now, if pressure is less than
										// 1000 mbar it's pretty likely that system is airborne
			isns_value = __LL_ADC_CALC_DATA_TO_VOLTAGE(3000, adc_value,
					LL_ADC_RESOLUTION_12B);
		else
			isns_value = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, adc_value,
					LL_ADC_RESOLUTION_12B);
		// end ADC read

		PIDController_Update(&pid, 435, isns_value); // update PID controller with set point 1000mV
		LL_TIM_OC_SetCompareCH3(TIM1, pid.out); // pid.out
	}
}

/*
 void update_int_count(uint32_t count) {
 lfs_file_open(&lfs, &int_count_save, "int_count_30",
 LFS_O_RDWR | LFS_O_CREAT);
 lfs_file_rewind(&lfs, &int_count_save);
 lfs_file_write(&lfs, &int_count_save, &count, sizeof(count));
 lfs_file_close(&lfs, &int_count_save);
 lfs_unmount(&lfs);
 }
 */

int check_geo_fence() {
	return 0;
}

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

int ipow(int base, int exp) {
	int result = 1;
	for (;;) {
		if (exp & 1)
			result *= base;
		exp >>= 1;
		if (!exp)
			break;
		base *= base;
	}

	return result;
}

void convert_gps_coords(void) {
	// move the decimal point left 2 for coords
	log_item.lat_frac = (((log_item.lat_int % 100)
			* ipow(10, count_revifs(log_item.lat_frac))) + log_item.lat_frac)
			/ 6;
	log_item.lat_int = (log_item.lat_int - (log_item.lat_int % 100)) / 100;

	log_item.lon_frac = (((log_item.lon_int % 100)
			* ipow(10, count_revifs(log_item.lon_frac))) + log_item.lon_frac)
			/ 6;
	log_item.lon_int = (log_item.lon_int - (log_item.lon_int % 100)) / 100;

	if (log_item.lat_dir == 'S')
		log_item.lat_int = log_item.lat_int * -1;

	if (log_item.lon_dir == 'W')
		log_item.lon_int = log_item.lon_int * -1;
}

// this function waits for NMEA messages and parses it into the log item
// struct above
int get_gps_data() {

	static char buf[100] = { 0 };
	static uint32_t len = 100;
	static uint32_t i = 0;

	while (1) { // poll I2C bus until end of line is received

		int timeout = 0; // I2C software timeout counter

		// I2C RX
		LL_I2C_HandleTransfer(I2C1, (0x42 << 1), LL_I2C_ADDRSLAVE_7BIT, 1,
		LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ); // request 1 byte from GPS

		while (LL_I2C_IsActiveFlag_RXNE(I2C1) == 0) { // wait for response
			if (timeout >= 20000) {
				return -1;
			}
			timeout++;
		}

		buf[i] = (char) LL_I2C_ReceiveData8(I2C1);

		if (buf[i] == '$') {
			i = 0;
			buf[i] = '$';
		}

		if (i >= (len - 1) || buf[i] == 10) {
			buf[len - 1] = 0x00;

			if ((i + 1) < len - 1)
				buf[i + 1] = '\0';

			i = 0;

			if (strncmp((buf + 3), "GGA", 3) == 0) { // parse GGA message

				//write_buf_to_fs(&lfs, &cfg, &log_file, "flight_log", buf, strlen(buf));
				sscanf(buf,
						"%*[^,],%lu.%*lu,%ld.%lu,%c,%lu.%lu,%c,%*d,%*d,%*d.%*d,%lu",
						&(log_item.time), &(log_item.lat_int),
						&(log_item.lat_frac), &(log_item.lat_dir),
						&(log_item.lon_int), &(log_item.lon_frac),
						&(log_item.lon_dir), &(log_item.altitude));

				convert_gps_coords();

				break;
			}

			if (strncmp((buf + 3), "RMC", 3) == 0) { // parse RMC message

				/*
				 write_buf_to_fs(&lfs, &cfg, &log_file, "flight_log", buf,
				 strlen(buf));
				 */

				sscanf(buf,
						"%*[^,],%lu.%*lu,%*c,%ld.%lu,%c,%lu.%lu,%c,%*lu.%*lu,%*lu.%*lu,%lu",
						&(log_item.time), &(log_item.lat_int),
						&(log_item.lat_frac), &(log_item.lat_dir),
						&(log_item.lon_int), &(log_item.lon_frac),
						&(log_item.lon_dir), &(log_item.date));

				convert_gps_coords();
			}
		}

		if (buf[i] != 0xFF)
			i++;
	}

	return 0;
}

int set_heater_params(int t_high, int t_low) {

	uint8_t buf[3] = { 0 };

	if (t_high > 127 || t_high < -128)
		return -1;
	if (t_low > 127 || t_low < -128)
		return -1;

	buf[0] = 0x02; // address of t_low register
	buf[1] = (t_low * 256) >> 8;
	buf[2] = (t_low * 256);

	if (i2c_write_blocking(I2C1, TEMP_ADDR, buf, 3, 0) == -1)
		return -1;	// ERROR

	buf[0] = 0x03; // address of t_high register
	buf[1] = (t_high * 256) >> 8;
	buf[2] = (t_high * 256);

	if (i2c_write_blocking(I2C1, TEMP_ADDR, buf, 3, 0) == -1)
		return -1;	// ERROR

	return 0;
}

int read_heater_params(int *t_high, int *t_low) {

	uint8_t buf[3] = { 0 };

	buf[0] = 0x02; // address of t_low register

	if (i2c_write_blocking(I2C1, TEMP_ADDR, buf, 1, 0) == -1)
		return -1;	// ERROR
	HAL_Delay(100);
	i2c_read_blocking(I2C1, TEMP_ADDR, (buf + 1), 2, 0); // read conversion value back
	*t_low = (int) ((buf[1] << 8) | buf[2]) / 256;

	buf[0] = 0x03; // address of t_high register
	buf[1] = 0;
	buf[2] = 0;

	if (i2c_write_blocking(I2C1, TEMP_ADDR, buf, 1, 0) == -1)
		return -1;	// ERROR
	HAL_Delay(100);
	i2c_read_blocking(I2C1, TEMP_ADDR, (buf + 1), 2, 0); // read conversion value back
	*t_high = (int) ((buf[1] << 8) | buf[2]) / 256;

	return 0;

}

int read_heater_temp() {
	uint8_t zero = 0;
	uint8_t buf[] = { 0, 0 };
	int temperature = 0;

	i2c_write_blocking(I2C1, TEMP_ADDR, &zero, 1, 0); // write address of temperature conversion result register
	HAL_Delay(100);
	i2c_read_blocking(I2C1, TEMP_ADDR, buf, 2, 0); // read conversion value back

	temperature = ((int) ((int16_t) (buf[0] << 8)) | (buf[1])) / 256;

	return temperature;
}

// these two I2C functions are not good, the args are mostly unused but writing
// the signature like this made copying RP2040 code faster
int i2c_read_blocking(I2C_TypeDef *i2cx, uint8_t addr, uint8_t *buf,
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

int i2c_write_blocking(I2C_TypeDef *i2cx, uint8_t addr, uint8_t *buf,
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

int log_init() {
	FRESULT res;

	log_item.lat_dir = '0';
	log_item.lon_dir = '0';

	// Link the FatFS driver to SD logical drive
	FATFS_LinkDriver(&USER_Driver, "/SD");

	//-- Mount the filesystem --
	res = f_mount(&fs, "/SD", 1);
	if (res != FR_OK)
		return -1; //ERROR: mount failed

	//-- CSV File Setup --
	// Open/create CSV file for writing
	res = f_open(&csvFile, "data.csv", FA_WRITE | FA_OPEN_APPEND);
	if (res == FR_OK) {
		// CSV column headers
		const char *header = " ,time,lat,long,altitude,temp,press,b_temp\r\n";

		res = f_write(&csvFile, header, strlen(header), &bytes_written);

		if (res != FR_OK) {
			return -1; // ERROR: write to file failed
		}

		// Close file after writing
		f_close(&csvFile);

	} else {
		return -1; // ERROR: opening file failed
	}

	return 0;
}

int log_system_data() {
	FRESULT res;

	res = f_open(&csvFile, "data.csv", FA_WRITE | FA_OPEN_APPEND);
	if (res == FR_OK) {

		snprintf(csv_line, 256, "%lu,%lu,%ld.%lu,%ld.%lu,%ld,%ld,%lu,%ld\r\n",
				log_item.log_count, log_item.time, log_item.lat_int,
				log_item.lat_frac, log_item.lon_int, log_item.lon_frac,
				log_item.altitude, log_item.temperature, log_item.pressure,
				log_item.board_temp);

		res = f_write(&csvFile, csv_line, strlen(csv_line), &bytes_written);

		if (res != FR_OK) {
			return -1; // ERROR: write to file failed
		}

		// Close file after writing
		f_close(&csvFile);

	} else {
		return -1; // ERROR: opening file failed
	}

	return 0;
}

