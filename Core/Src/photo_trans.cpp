#include "../../Core/Inc/photo_trans.hpp"

// IN15: LF_ADC
// IN13: RF_ADC
// IN11: RR_ADC
// IN9:  LL_ADC

PhotoTransSensor photo_trans_sensor;

extern "C" {
    void readADC(int i){
        photo_trans_sensor.updateADC(i); 
    }
}

void PhotoTransSensor::updateADC(int i){
    // [2]が左前センサ、[1]が右前センサ, [0]は左センサ、[3]が右センサ
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

int16_t PhotoTransSensor::getDiffADCBothWall() const {
    return int(-adc_val[2] + adc_val[1]) - ADCParam::SENSOR_OFFSET;
}

// 左壁制御（右壁がないとき）
uint16_t PhotoTransSensor::getDiffADCLeftOneWall() const {
    return this->adc_val[2] - ADCParam::SENSOR_OFFSET_L;
}

// 右壁制御（左壁がないとき）
uint16_t PhotoTransSensor::getDiffADCRightOneWall() const {
    return this->adc_val[1] - ADCParam::SENSOR_OFFSET_R;
}