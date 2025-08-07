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
uint8_t RadioResult = 0x00;
uint8_t RadioParam = 0x00;
uint8_t RadioMode = 0x00;
uint8_t RadioStatus = 0x00;

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

	/*** GPIO Configuration (for debugging) ***/
	/* DEBUG_SUBGHZSPI_NSSOUT = PA4
	 * DEBUG_SUBGHZSPI_SCKOUT = PA5
	 * DEBUG_SUBGHZSPI_MISOOUT = PA6
	 * DEBUG_SUBGHZSPI_MOSIOUT = PA7
	 * DEBUG_RF_HSE32RDY = PA10
	 * DEBUG_RF_NRESET = PA11
	 * DEBUG_RF_SMPSRDY = PB2
	 * DEBUG_RF_DTB1 = PB3 <---- Conflicts with RF_IRQ0
	 * DEBUG_RF_LDORDY = PB4
	 * RF_BUSY = PA12
	 * RF_IRQ0 = PB3
	 * RF_IRQ1 = PB5
	 * RF_IRQ2 = PB8
	 */

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

	MX_SUBGHZ_Init();

	set_heater_params(T_HIGH, T_LOW);

	log_init();

	ms5607_init(I2C1); // init pressure, temperature, and relative humidity sensor

	/*## 1 - Wakeup the SUBGHZ Radio ###########################################*/
	/* Set Sleep Mode */
	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_SLEEP, &RadioParam, 1)
			!= HAL_OK) {
		Error_Handler();
	}

	RadioParam = 0x01;

	/* Set Standby Mode */
	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_STANDBY, &RadioParam, 1)
			!= HAL_OK) {
		Error_Handler();
	}

	RadioParam = 0x00;

	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_CLR_ERROR, &RadioParam, 1)
			!= HAL_OK) {
		Error_Handler();
	}

	/* Set Standby Mode */
	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TCXOMODE, RadioTCXO, 4)
			!= HAL_OK) {
		Error_Handler();
	}

	/* Retrieve Status from SUBGHZ Radio */
	if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, &RadioResult, 1)
			!= HAL_OK) {
		Error_Handler();
	} else {
		/* Format Mode and Status receive from SUBGHZ Radio */
		RadioMode = ((RadioResult & RADIO_MODE_BITFIELD) >> 4);

		/* Check if SUBGHZ Radio is in RADIO_MODE_STANDBY_RC mode */
		if (RadioMode != RADIO_MODE_STANDBY_RC) {
			Error_Handler();
		}
	}

	/*## 2 - Set a TX on SUBGHZ Radio side #####################################*/
	/* Set Tx Mode. RadioCmd = 0x00 Timeout deactivated */
	/*if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TX, RadioCmd, 3) != HAL_OK) {
	 Error_Handler();
	 }
	 */

	const uint32_t frequency = 915000000;
	uint8_t buf[4];
	uint32_t freq = 0;

	freq = (uint32_t) ((double) frequency / (double) FREQ_STEP );
	buf[0] = (uint8_t) ((freq >> 24) & 0xFF);
	buf[1] = (uint8_t) ((freq >> 16) & 0xFF);
	buf[2] = (uint8_t) ((freq >> 8) & 0xFF);
	buf[3] = (uint8_t) (freq & 0xFF);

	/* Reset RadioResult */
	RadioResult = 0x00;

	/* Retrieve Status from SUBGHZ Radio */
	if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, &RadioResult, 1)
			!= HAL_OK) {
		Error_Handler();
	}

	if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_ERROR, RadioErr, 3)
			!= HAL_OK) {
		Error_Handler();
	}

	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_RFFREQUENCY, buf, 4)
			!= HAL_OK) {
		Error_Handler();
	}

	if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_ERROR, RadioErr, 3)
			!= HAL_OK) {
		Error_Handler();
	}

	/*
	 if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TXCONTINUOUSWAVE, RadioCmd, 0)
	 != HAL_OK) {
	 Error_Handler();
	 }
	 */

	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TX, RadioCmd, 3) != HAL_OK) {
		//Error_Handler();
	}

	hsubghz.ErrorCode = 0;

	HAL_Delay(1000);

	if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_ERROR, RadioErr, 3)
			!= HAL_OK) {
		Error_Handler();
	}

	/*## 3 - Get TX status from SUBGHZ Radio side ##############################*/
	/* Check that TX is well ongoing (RADIO_MODE_TX), wait end of transfer */

	/* Reset RadioResult */
	RadioResult = 0x00;

	/* Retrieve Status from SUBGHZ Radio */
	if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, &RadioResult, 1)
			!= HAL_OK) {
		Error_Handler();
	}

	/* Format Mode and Status receive from SUBGHZ Radio */
	RadioMode = ((RadioResult & RADIO_MODE_BITFIELD) >> 4);
	RadioStatus = ((RadioResult & RADIO_STATUS_BITFIELD) >> 1);

	if (RadioMode == RADIO_MODE_TX) {
		/* Wait end of transfer. SUBGHZ Radio go in Standby Mode */
		do {
			/* Reset RadioResult */
			RadioResult = 0x00;

			/* Retrieve Status from SUBGHZ Radio */
			if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, &RadioResult,
					1) != HAL_OK) {
				Error_Handler();
			}

			/* Format Mode and Status receive from SUBGHZ Radio */
			RadioMode = ((RadioResult & RADIO_MODE_BITFIELD) >> 4);
			RadioStatus = ((RadioResult & RADIO_STATUS_BITFIELD) >> 1);
		} while (RadioMode != RADIO_MODE_STANDBY_RC);
	} else {
		/* Call Error Handler; LED1 blinking */
		Error_Handler();
	}

	/* Check if TX is well done  (SUBGHZ Radio already in Standby mode) */
	if (RadioStatus == RADIO_COMMAND_TX_DONE) {
		// nothing
	} else {
		/* Call Error Handler; LED1 blinking */
		Error_Handler();
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {

		log_item.log_count++;

		ms5607_get_press_temp(&log_item.pressure, &log_item.temperature);

		log_item.board_temp = read_heater_temp();

		get_gps_data();

		log_system_data();

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
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {
	}

	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
	LL_RCC_MSI_Enable();

	/* Wait till MSI is ready */
	while (LL_RCC_MSI_IsReady() != 1) {
	}

	LL_RCC_MSI_EnableRangeSelection();
	LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_11);
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
	LL_SetSystemCoreClock(48000000);

	/* Update the time base */
	if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK) {
		Error_Handler();
	}
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
