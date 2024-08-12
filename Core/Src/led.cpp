/*
 * led.cpp
 *
 *  Created on: 2023/06/24
 *      Author: hashimoto
 */
#include "../../Core/Inc/led.hpp"
#include "../../Core/Inc/main.h"
#include "stm32f4xx_hal.h"

void LedBlink::toggle() {
    HAL_GPIO_WritePin(GPIOB, Debug_LED_Pin, GPIO_PIN_SET); //LEDを点灯
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, Debug_LED_Pin, GPIO_PIN_RESET); //LEDを点灯
    HAL_Delay(100);
}

void LedBlink::stop() {
    HAL_GPIO_WritePin(GPIOB, Debug_LED_Pin, GPIO_PIN_RESET);
}