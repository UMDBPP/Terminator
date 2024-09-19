/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>

#include "stm32l412xx.h"

#include "stm32l4xx.h"
#include "stm32l4xx_ll_gpio.h"

LL_GPIO_InitTypeDef gpio_test;

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
#warning \
    "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void) {

    LL_GPIO_StructInit(&gpio_test);

    gpio_test.Pin = LL_GPIO_PIN_13;
    gpio_test.Mode = LL_GPIO_MODE_OUTPUT;

    if (LL_GPIO_Init(GPIOB, &gpio_test) == ERROR)
        while (1);

    while (1) {
    }
}
