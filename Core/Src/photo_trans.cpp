#include "../../Core/Inc/photo_trans.hpp"

void PhotoTransSensor::getWallSensorData() {
    uint16_t a[4];
    GPIO_TypeDef* LED_PORTS[4] = {LL_LED_GPIO_Port, RF_LED_GPIO_Port, LF_LED_GPIO_Port, RR_LED_GPIO_Port};
    uint16_t LED_PINS[4] = {LL_LED_Pin, RF_LED_Pin, LF_LED_Pin, RR_LED_Pin};
    for(int i=0; i<4; i++){
        HAL_GPIO_WritePin(LED_PORTS[i], LED_PINS[i], GPIO_PIN_SET);
        for(int i=0; i<100; i++) {}
        if(HAL_ADC_Start(&hadc1) == HAL_OK){
            HAL_ADC_PollForConversion(&hadc1, 1);
            a[i] = HAL_ADC_GetValue(&hadc1);
            HAL_GPIO_WritePin(LED_PORTS[i], LED_PINS[i], GPIO_PIN_RESET);
            HAL_Delay(1);
        }
    }
 
    printf("LL_ADC =%d, RF_ADC =%d, LF_ADC =%d, RR_ADC =%d\n\r", a[0], a[1], a[2], a[3]);
}

// IN15: LF_ADC
// IN13: RF_ADC
// IN11: RR_ADC
// IN9:  LL_ADC