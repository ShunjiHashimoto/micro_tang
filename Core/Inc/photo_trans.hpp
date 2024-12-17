#ifndef INC_PHOTO_TRANS_HPP_
#define INC_PHOTO_TRANS_HPP_


#include "../../Core/Inc/main.h"
#include "adc.h"
#include "tim.h"
#include "stm32f4xx_hal.h"
#include "stm32f405xx.h"
#include "led.hpp"
#include "params.hpp"

class PhotoTransSensor {
public:
    GPIO_TypeDef* LED_PORTS[4] = {LL_LED_GPIO_Port, RF_LED_GPIO_Port, LF_LED_GPIO_Port, RR_LED_GPIO_Port};
    uint16_t LED_PINS[4] = {LL_LED_Pin, RF_LED_Pin, LF_LED_Pin, RR_LED_Pin};
    uint32_t adc_channels[4] = {ADC_CHANNEL_9, ADC_CHANNEL_13, ADC_CHANNEL_15, ADC_CHANNEL_11};
    void configureADCChannel(uint32_t channel);
    uint16_t getCurrentADC(int i) const;
    void updateADC(int i);
    uint16_t adc_val[4] = {0};
};

#endif /* INC_PHOTO_TRANS_HPP_ */
