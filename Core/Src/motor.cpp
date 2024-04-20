/*
 * motor.cpp
 *
 *  Created on: 2023/06/25
 *      Author: hashimoto
 */

#include "motor.hpp"

void Motor::run(uint16_t direction_channel, GPIO_PinState direction, uint32_t mode_channel, uint32_t duty_channel, int duty) {
    HAL_GPIO_WritePin(GPIOA, mode_channel, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, direction_channel, direction);
    __HAL_TIM_SET_COMPARE(&htim1, duty_channel, duty);
    HAL_TIM_PWM_Start(&htim1, duty_channel);
}
void Motor::toggle(){
    std::cout << "test" << std::endl;
}


