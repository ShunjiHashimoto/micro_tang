#ifndef INC_PHOTO_TRANS_HPP_
#define INC_PHOTO_TRANS_HPP_

#include "../../Core/Inc/main.h"
#include "adc.h"
#include "tim.h"
#include "stm32f4xx_hal.h"
#include "stm32f405xx.h"
#include "led.hpp"

class PhotoTransSensor {
public:

    GPIO_TypeDef* LED_PORTS[4] = {LL_LED_GPIO_Port, RF_LED_GPIO_Port, LF_LED_GPIO_Port, RR_LED_GPIO_Port};
    uint16_t LED_PINS[4] = {LL_LED_Pin, RF_LED_Pin, LF_LED_Pin, RR_LED_Pin};
    uint32_t adc_value = 0;
    float adc_val[4];
    void getWallSensorData();
    LedSensor led_sensor;
};

#endif /* INC_PHOTO_TRANS_HPP_ */
