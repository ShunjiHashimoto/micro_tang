#include "../../Core/Inc/photo_trans.hpp"

PhotoTransSensor photo_trans_sensor;

extern "C" {
    void readADC(int i){
        photo_trans_sensor.updateADC(i); 
    }
}

void PhotoTransSensor::updateADC(int i){
    // LED点灯
    HAL_GPIO_WritePin(this->LED_PORTS[i], this->LED_PINS[i], GPIO_PIN_SET);
    for(int j=0; j<200; j++) {}
    this->configureADCChannel(this->adc_channels[i]);
    if (HAL_ADC_Start(&hadc1) == HAL_OK) {
        // 受光データ取得
        HAL_ADC_PollForConversion(&hadc1, 1);
        this->adc_val[i] = HAL_ADC_GetValue(&hadc1); 
        // LED消灯
        HAL_GPIO_WritePin(this->LED_PORTS[i], this->LED_PINS[i], GPIO_PIN_RESET);
    }
}

// ADCチャネルを動的に設定
void PhotoTransSensor::configureADCChannel(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1; // RANKは常に1つだけ設定する
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

uint16_t PhotoTransSensor::getCurrentADC(int i) const {
    return this->adc_val[i];
}

// void PhotoTransSensor::getWallSensorData() {
//     uint16_t a[4];
//     for(int i=0; i<4; i++){
//         HAL_GPIO_WritePin(LED_PORTS[i], LED_PINS[i], GPIO_PIN_SET);
//         for(int i=0; i<100; i++) {}
//         if(HAL_ADC_Start(&hadc1) == HAL_OK){
//             HAL_ADC_PollForConversion(&hadc1, 1);
//             a[i] = HAL_ADC_GetValue(&hadc1);
//             HAL_GPIO_WritePin(LED_PORTS[i], LED_PINS[i], GPIO_PIN_RESET);
//             HAL_Delay(1);
//         }
//     }
 
//     printf("LL_ADC =%d, RF_ADC =%d, LF_ADC =%d, RR_ADC =%d\n\r", a[0], a[1], a[2], a[3]);
// }

// IN15: LF_ADC
// IN13: RF_ADC
// IN11: RR_ADC
// IN9:  LL_ADC