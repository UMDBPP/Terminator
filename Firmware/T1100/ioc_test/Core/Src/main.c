/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "app_fatfs.h"
#include "i2c.h"
#include "lptim.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "MS5607.h"
#include "PID.h"
#include "stm32_helper.h"
#include "sx1262.h"
#include "t1100_helper.h"
#include "IIRFirstOrder.h"

#include "subghz.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define RF_FREQUENCY                                915000000 /* Hz */
#define TX_OUTPUT_POWER                             0        /* dBm */
#define LORA_BANDWIDTH                              0         /* Hz */
#define LORA_SPREADING_FACTOR                       7
#define LORA_CODINGRATE                             1
#define LORA_PREAMBLE_LENGTH                        8         /* Same for Tx and Rx */
#define LORA_SYMBOL_TIMEOUT                         5         /* Symbols */

#define RADIO_MODE_STANDBY_RC 0x02
#define RADIO_MODE_STANDBY_XOSC 0x03
#define RADIO_MODE_TX 0x06
#define RADIO_COMMAND_TX_DONE 0x06

#define RADIO_MODE_BITFIELD 0x70
#define RADIO_STATUS_BITFIELD 0x0E

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t RadioCmd[3] = { 0x00, 0x00, 0x00 };
uint8_t RadioErr[3] = { 0x00, 0x00, 0x00 };
uint8_t RadioTCXO[4] = { 0x01, 0x01, 0x02, 0x80 };
uint8_t RadioTxParams[2] = { 0x16, 0x04 };
uint8_t RadioResult = 0x00;
uint8_t RadioParam = 0x00;
uint8_t RadioMode = 0x00;
uint8_t RadioStatus = 0x00;
uint8_t RadioPAconfig[4] = { 0x04, 0x07, 0x00, 0x01 };
uint8_t RadioBaseAddr[2] = { 0x00, 0x7F };
uint8_t RadioModParams[4] = { 0x0A, 0x04, 0x04, 0x00 };
uint8_t RadioPacketParams[6] = { 0x00, 0x0F, 0x00, 0x64, 0x01, 0x00 };

uint8_t tx_buf[] = "terminator alive!";

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

sx1262_t radio = { SPI1, (gpio_t ) { GPIOB, LL_GPIO_PIN_0 }, { 0, 0 } };
log_item_t log_item;

static char log_buf[100] = { 0 };

IIRFirstOrder iir;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void setup_radio(void);
void set_rf_switch_rx(void);
void set_rf_switch_tx(void);
void radio_tx(uint8_t *buf, uint16_t bytes);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_FATFS_Init();
	MX_I2C1_Init();
	MX_LPTIM1_Init();
	MX_ADC_Init();
	MX_TIM1_Init();
	MX_LPUART1_UART_Init();
	/* USER CODE BEGIN 2 */

	setup_radio();

	set_heater_params(T_HIGH, T_LOW);

	log_init();

	ms5607_init(I2C1); // init pressure, temperature, and relative humidity sensor

	IIRFirstOrder_Init(&iir, 980);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {

		log_item.log_count++;

		ms5607_get_press_temp(&log_item.pressure, &log_item.temperature);

		log_item.board_temp = read_heater_temp();

		if (get_gps_data() == -1)
			log_timeout();

		log_system_data();

		log_item.ascent_rate = log_item.altitude - log_item.prev_altitude;
		log_item.prev_altitude = log_item.altitude;
		log_item.filtered_ascent_rate = IIRFirstOrder_Update(&iir,
				log_item.ascent_rate);

		if (log_item.log_count % 30 == 0)
			radio_tx(tx_buf, sizeof(tx_buf));

		HAL_Delay(1000);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
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
	LL_RCC_MSI_Enable();

	/* Wait till MSI is ready */
	while (LL_RCC_MSI_IsReady() != 1) {
	}

	LL_RCC_MSI_EnableRangeSelection();
	LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_10);
	LL_RCC_MSI_SetCalibTrimming(0);
	LL_PWR_EnableBkUpAccess();
	LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_LOW);
	LL_RCC_LSE_EnablePropagation();
	LL_RCC_LSE_Enable();

	/* Wait till LSE is ready */
	while (LL_RCC_LSE_IsReady() != 1) {
	}

	LL_RCC_MSI_EnablePLLMode();
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);

	/* Wait till System clock is ready */
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI) {
	}

	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAHB3Prescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
	/* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
	LL_SetSystemCoreClock(32000000);

	/* Update the time base */
	if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void setup_radio(void) {

	MX_SUBGHZ_Init();

	//set standby

	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_SLEEP, &RadioParam, 1)
			!= HAL_OK) {
		Error_Handler();
	}

	RadioParam = 0x00;
	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_STANDBY, &RadioParam, 1)
			!= HAL_OK) {
		Error_Handler();
	}
	RadioParam = 0x00;

	// set tcxo
	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TCXOMODE, RadioTCXO, 4)
			!= HAL_OK) {
		Error_Handler();
	}

	RadioParam = 0x01;
	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_STANDBY, &RadioParam, 1)
			!= HAL_OK) {
		//Error_Handler();
		hsubghz.ErrorCode = 0;

		while (LL_PWR_IsActiveFlag_RFBUSYS() & LL_PWR_IsActiveFlag_RFBUSYMS())
			;
	}
	RadioParam = 0x00;

	RadioParam = 0x30;
	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TXFALLBACKMODE, &RadioParam,
			1) != HAL_OK) {
		Error_Handler();
	}
	RadioParam = 0x00;

	/*
	 RadioParam = 0x7F;
	 if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_CALIBRATE, &RadioParam, 1)
	 != HAL_OK) {
	 //Error_Handler();
	 hsubghz.ErrorCode = 0;

	 while (LL_PWR_IsActiveFlag_RFBUSYS() & LL_PWR_IsActiveFlag_RFBUSYMS())
	 ;
	 }
	 RadioParam = 0x00;
	 */

	// set packet type
	RadioParam = 0x01;
	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_PACKETTYPE, &RadioParam, 1)
			!= HAL_OK) {
		Error_Handler();
	}
	RadioParam = 0x00;

	HAL_SUBGHZ_ReadRegister(&hsubghz, 0x916, &RadioParam);
	HAL_SUBGHZ_WriteRegister(&hsubghz, 0x916, RadioParam | 0x40);
	RadioParam = 0x00;

	// set rf switch
	set_rf_switch_rx();

	// set reg mode
	RadioParam = 0x01;
	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_REGULATORMODE, &RadioParam, 1)
			!= HAL_OK) {
		Error_Handler();
	}
	RadioParam = 0x00;

	// clear errors
	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_CLR_ERROR, &RadioParam, 1)
			!= HAL_OK) {
		Error_Handler();
	}

	// get errors
	if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, &RadioResult, 1)
			!= HAL_OK) {
		Error_Handler();
	}

	if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_ERROR, RadioErr, 3)
			!= HAL_OK) {
		Error_Handler();
	}

	// set rf freq
	const uint32_t frequency = 915000000;
	uint8_t buf[4];
	uint32_t freq = 0;

	freq = (uint32_t) ((double) frequency / (double) FREQ_STEP );
	buf[0] = (uint8_t) ((freq >> 24) & 0xFF);
	buf[1] = (uint8_t) ((freq >> 16) & 0xFF);
	buf[2] = (uint8_t) ((freq >> 8) & 0xFF);
	buf[3] = (uint8_t) (freq & 0xFF);

	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_RFFREQUENCY, buf, 4)
			!= HAL_OK) {
		Error_Handler();
	}

	// set pa config
	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_PACONFIG, RadioPAconfig, 4)
			!= HAL_OK) {
		Error_Handler();
	}

	// set tx params
	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TXPARAMS, RadioTxParams, 2)
			!= HAL_OK) {
		Error_Handler();
	}

	// set buffer base address
	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_BUFFERBASEADDRESS,
			RadioBaseAddr, 2) != HAL_OK) {
		Error_Handler();
	}

	// set modulation params
	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_MODULATIONPARAMS,
			RadioModParams, 4) != HAL_OK) {
		Error_Handler();
	}

	// set packet params
	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_PACKETPARAMS,
			RadioPacketParams, 6) != HAL_OK) {
		Error_Handler();
	}

	// set sync word
	//HAL_SUBGHZ_WriteRegister(&hsubghz, 0x0740, 0x34);
	//HAL_SUBGHZ_WriteRegister(&hsubghz, 0x0741, 0x44);

	// set symbol timeout
	RadioParam = 0x00;
	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_LORASYMBTIMEOUT, &RadioParam,
			1) != HAL_OK) {
		Error_Handler();
	}
	RadioParam = 0x00;

	// get errors
	if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, &RadioResult, 1)
			!= HAL_OK) {
		Error_Handler();
	}

	if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_ERROR, RadioErr, 3)
			!= HAL_OK) {
		Error_Handler();
	}
}

void set_rf_switch_rx(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

void set_rf_switch_tx(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

void radio_tx(uint8_t *buf, uint16_t bytes) {

	HAL_SUBGHZ_WriteBuffer(&hsubghz, 0, buf, bytes);

	set_rf_switch_tx();

	// clear errors
	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_CLR_ERROR, &RadioParam, 1)
			!= HAL_OK) {
		Error_Handler();
	}

	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TX, RadioCmd, 3) != HAL_OK) {
		//Error_Handler();
	}

	hsubghz.ErrorCode = 0;

	while (LL_PWR_IsActiveFlag_RFBUSYS() & LL_PWR_IsActiveFlag_RFBUSYMS())
		;

	/* Reset RadioResult */
	RadioResult = 0x00;

	// get errors
	if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, &RadioResult, 1)
			!= HAL_OK) {
		Error_Handler();
	}

	if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_ERROR, RadioErr, 3)
			!= HAL_OK) {
		Error_Handler();
	}

	/*
	 RadioMode = ((RadioResult & RADIO_MODE_BITFIELD) >> 4);
	 RadioStatus = ((RadioResult & RADIO_STATUS_BITFIELD) >> 1);

	 if (RadioMode == RADIO_MODE_TX) {
	 do {
	 RadioResult = 0x00;

	 if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, &RadioResult,
	 1) != HAL_OK) {
	 Error_Handler();
	 }

	 RadioMode = ((RadioResult & RADIO_MODE_BITFIELD) >> 4);
	 RadioStatus = ((RadioResult & RADIO_STATUS_BITFIELD) >> 1);
	 } while (RadioMode != RADIO_MODE_STANDBY_RC
	 && RadioMode != RADIO_MODE_STANDBY_XOSC);
	 } else if (RadioStatus == RADIO_COMMAND_TX_DONE) { // Check if TX is well done  (SUBGHZ Radio already in Standby mode)
	 // nothing
	 } else {
	 //Error_Handler();
	 }

	 set_rf_switch_rx();

	 // clear errors
	 if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_CLR_ERROR, &RadioParam, 1)
	 != HAL_OK) {
	 Error_Handler();
	 }

	 if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_RX, RadioCmd, 3) != HAL_OK) {
	 //Error_Handler();
	 }

	 hsubghz.ErrorCode = 0;

	 while (LL_PWR_IsActiveFlag_RFBUSYS() & LL_PWR_IsActiveFlag_RFBUSYMS())
	 ;


	 RadioResult = 0x00;

	 // get errors
	 if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, &RadioResult, 1)
	 != HAL_OK) {
	 Error_Handler();
	 }

	 if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_ERROR, RadioErr, 3)
	 != HAL_OK) {
	 Error_Handler();
	 }
	 */

}

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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
