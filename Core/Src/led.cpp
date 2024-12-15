/*
 * led.cpp
 *
 *  Created on: 2023/06/24
 *      Author: hashimoto
 */
#include "../../Core/Inc/led.hpp"
#include "../../Core/Inc/main.h"
#include "stm32f4xx_hal.h"

extern "C" {
    void ledSensorBlink(GPIO_TypeDef  *GPIOx, uint16_t GPIO_Pinx) {
        HAL_GPIO_TogglePin(GPIOx, GPIO_Pinx);
    }
}

void LedBlink::toggle() {
    HAL_GPIO_WritePin(GPIOB, Debug_LED_Pin, GPIO_PIN_SET); //LEDを点灯
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, Debug_LED_Pin, GPIO_PIN_RESET); //LEDを点灯
    HAL_Delay(100);
}

void LedBlink::stop() {
    HAL_GPIO_WritePin(GPIOB, Debug_LED_Pin, GPIO_PIN_RESET);
}

void LedSensor::blink(){
    HAL_GPIO_WritePin(RR_LED_GPIO_Port, RR_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(250); // 250[μsec]
    HAL_GPIO_WritePin(RR_LED_GPIO_Port, RR_LED_Pin, GPIO_PIN_RESET);
}