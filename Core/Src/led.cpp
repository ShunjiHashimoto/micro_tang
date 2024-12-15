/*
 * led.cpp
 *
 *  Created on: 2023/06/24
 *      Author: hashimoto
 */
#include "../../Core/Inc/led.hpp"
#include "../../Core/Inc/main.h"

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

void LedSensor::blink(GPIO_TypeDef  *GPIOx, uint16_t GPIO_Pinx){
    HAL_GPIO_WritePin(GPIOx, GPIO_Pinx, GPIO_PIN_SET);
}
void LedSensor::stop(GPIO_TypeDef  *GPIOx, uint16_t GPIO_Pinx){
    HAL_GPIO_WritePin(GPIOx, GPIO_Pinx, GPIO_PIN_RESET);
}