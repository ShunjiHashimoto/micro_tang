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
    uint32_t adc_value = 0;
    float adc_val[4];
    void getWallSensorData();
    LedSensor led_sensor;
};

#endif /* INC_PHOTO_TRANS_HPP_ */
