/*
 * led.hpp
 *
 *  Created on: 2023/06/24
 *      Author: hashimoto
 */

#ifndef INC_LED_HPP_
#define INC_LED_HPP_
#include "stm32f4xx_hal.h"

class LedBlink {
public:
    void toggle();
    void stop();
};

class LedSensor {
public:
    void blink(GPIO_TypeDef  *GPIOx, uint16_t GPIO_Pinx);
    void stop(GPIO_TypeDef  *GPIOx, uint16_t GPIO_Pinx);
};


#endif /* INC_LED_HPP_ */
