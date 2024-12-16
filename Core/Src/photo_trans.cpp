#include "../../Core/Inc/photo_trans.hpp"

void PhotoTransSensor::getWallSensorData() {
    ADC_ChannelConfTypeDef sConfig = {0};

    // IN9
    sConfig.Rank = 1;
    sConfig.Channel = ADC_CHANNEL_9;
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    HAL_GPIO_WritePin(LL_LED_GPIO_Port, LL_LED_Pin, GPIO_PIN_SET);
    for(int i=0; i<100; i++) {}
    if(HAL_ADC_Start(&hadc1) == HAL_OK){
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 100);
        uint16_t adcValueOn = HAL_ADC_GetValue(&hadc1);

        HAL_GPIO_WritePin(LL_LED_GPIO_Port, LL_LED_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 100);
        uint16_t adcValueOff = HAL_ADC_GetValue(&hadc1);

        adc_value = adcValueOn - adcValueOff;
        printf("adc_val On[%d], Off[%d]: L L_ADC: %d \n\r", adcValueOn, adcValueOff, adc_value);
    }

    // IN13
    sConfig.Rank = 1;
    sConfig.Channel = ADC_CHANNEL_13;
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    HAL_GPIO_WritePin(RF_LED_GPIO_Port, RF_LED_Pin, GPIO_PIN_SET);
    for(int i=0; i<100; i++) {}
    if(HAL_ADC_Start(&hadc1) == HAL_OK){
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 100);
        uint16_t adcValueOn = HAL_ADC_GetValue(&hadc1);

        HAL_GPIO_WritePin(RF_LED_GPIO_Port, RF_LED_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 100);
        uint16_t adcValueOff = HAL_ADC_GetValue(&hadc1);

        adc_value = adcValueOn - adcValueOff;
        printf("adc_val On[%d], Off[%d]: R F_ADC: %d \n\r", adcValueOn, adcValueOff, adc_value);
    }

    // IN15
    sConfig.Rank = 1;
    sConfig.Channel = ADC_CHANNEL_15;
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    HAL_GPIO_WritePin(LF_LED_GPIO_Port, LF_LED_Pin, GPIO_PIN_SET);
    for(int i=0; i<100; i++) {}
    if(HAL_ADC_Start(&hadc1) == HAL_OK){
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 100);
        uint16_t adcValueOn = HAL_ADC_GetValue(&hadc1);

        HAL_GPIO_WritePin(LF_LED_GPIO_Port, LF_LED_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 100);
        uint16_t adcValueOff = HAL_ADC_GetValue(&hadc1);

        adc_value = adcValueOn - adcValueOff;
        printf("adc_val On[%d], Off[%d]: L F_ADC: %d \n\r", adcValueOn, adcValueOff, adc_value);
    }

    // IN11
    sConfig.Rank = 1;
    sConfig.Channel = ADC_CHANNEL_11;
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    HAL_GPIO_WritePin(RR_LED_GPIO_Port, RR_LED_Pin, GPIO_PIN_SET);
    for(int i=0; i<100; i++) {}
    if(HAL_ADC_Start(&hadc1) == HAL_OK){
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 100);
        uint16_t adcValueOn = HAL_ADC_GetValue(&hadc1);

        HAL_GPIO_WritePin(RR_LED_GPIO_Port, RR_LED_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 100);
        uint16_t adcValueOff = HAL_ADC_GetValue(&hadc1);

        adc_value = adcValueOn - adcValueOff;
        printf("adc_val On[%d], Off[%d]: R R_ADC: %d \n\r", adcValueOn, adcValueOff, adc_value);
    }

    printf("Finished!!\n\r");
    HAL_Delay(200);
}

// led_sensor.blink();
// for(int i=0; i<200; i++) {}
// HAL_ADC_Start(&hadc1);
// HAL_ADC_PollForConversion(&hadc1, 10);
// adc_value = HAL_ADC_GetValue(&hadc1);
// led_sensor.stop();
// printf("adc_val : RF_ADC: %d \n\r", adc_value);
// HAL_ADC_Stop(&hadc1);
// HAL_Delay(200);
// IN15: LF_ADC
// IN13: RF_ADC
// IN11: RR_ADC
// IN9:  LL_ADC