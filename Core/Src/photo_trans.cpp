#include "../../Core/Inc/photo_trans.hpp"

void PhotoTransSensor::getWallSensorData() {
    // for(int i=0; i<4; i++){
    //     if(i == 0){
    // ADC_ChannelConfTypeDef sConfig = {0};
    // config
    // sConfig.Channel = LL_ADC1_Pin;
    // sConfig.Rank = 1;
    // sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
    // sConfig.Offset = 0;
    // if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    //     printf("something wrong");
    //     return;
    // }
    HAL_GPIO_WritePin(LL_LED_GPIO_Port, LL_LED_Pin, GPIO_PIN_SET);
    for(int i=0; i<100; i++) {}
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1000);
    uint16_t adcValueOn = HAL_ADC_GetValue(&hadc1);

    HAL_GPIO_WritePin(LL_LED_GPIO_Port, LL_LED_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1000);
    uint16_t adcValueOff = HAL_ADC_GetValue(&hadc1);

    adc_value = adcValueOn - adcValueOff;
    printf("adc_val On[%d], Off[%d]: L L_ADC: %d \n\r", adcValueOn, adcValueOff, adc_value);
    HAL_ADC_Stop(&hadc1);
    HAL_Delay(500);
        // }
        // else if(i == 1){
        //     led_sensor.blink(LF_LED_GPIO_Port, LF_LED_Pin);
        //     for(int i=0; i<200; i++) {}
        //     HAL_ADC_Start(&hadc1);
        //     HAL_ADC_PollForConversion(&hadc1, 1);
        //     adc_value = HAL_ADC_GetValue(&hadc1);
        //     // adc_val[i] = 1000*3.3 * adc_value / 4096; // 値がわかりやすいようにx1000
        //     led_sensor.stop(LF_LED_GPIO_Port, LF_LED_Pin);
        //     HAL_ADC_Stop(&hadc1);
        //     printf("adc_val [%d]: L F_ADC: %d \n\r", i, adc_value);
        // }
        // else if(i == 2){
        //     led_sensor.blink(RF_LED_GPIO_Port, RF_LED_Pin);
        //     for(int i=0; i<200; i++) {}
        //     HAL_ADC_Start(&hadc1);
        //     HAL_ADC_PollForConversion(&hadc1, 1);
        //     adc_value = HAL_ADC_GetValue(&hadc1);
        //     // adc_val[i] = 1000*3.3 * adc_value / 4096; // 値がわかりやすいようにx1000
        //     led_sensor.stop(RF_LED_GPIO_Port, RF_LED_Pin);
        //     HAL_ADC_Stop(&hadc1);
        //     printf("adc_val, [%d]: R F _ADC: %d \n\r",i,  adc_value);
        // }
        // else if(i == 3){
        //     led_sensor.blink(LL_LED_GPIO_Port, LL_LED_Pin);
        //     for(int i=0; i<200; i++) {}
        //     HAL_ADC_Start(&hadc1);
        //     HAL_ADC_PollForConversion(&hadc1, 1);
        //     adc_value = HAL_ADC_GetValue(&hadc1);
        //     // adc_val[i] = 1000*3.3 * adc_value / 4096; // 値がわかりやすいようにx1000
        //     led_sensor.stop(LL_LED_GPIO_Port, LL_LED_Pin);
        //     HAL_ADC_Stop(&hadc1);
        //     printf("adc_val [%d]: L L_ADC: %d \n\r", i, adc_value);
        // }
    // }
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