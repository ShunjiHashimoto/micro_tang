#include "sensors.h"
extern ADC_HandleTypeDef hadc1;

// void phototranCheck(uint32_t channel){
// 	  uint8_t msg[] = "Get Phototransistor Value \n\r";
// 	  char msg_[30];
// 	  uint32_t adc_val;
// 	  int Timeout = 100;
// 	//   HAL_UART_Transmit(&huart3, msg, strlen((char*)msg), Timeout);

// 	  HAL_GPIO_TogglePin(GPIOC, channel);
// 	  for(int i = 0; i<1000; i++){
// 	  }
// 	  HAL_ADC_Start(&hadc1);
// 	  if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK){
// 		  adc_val = HAL_ADC_GetValue(&hadc1);
// //		  if(adc_val > 1)
// //		  {
// 			sprintf(msg_, "Recieved message: %d \n\r", adc_val);
// 			// HAL_UART_Transmit(&huart3, msg_, strlen((char*)msg_), Timeout);
// 			HAL_Delay(100);
// //		  }
// 	  }
// 	  HAL_ADC_Stop(&hadc1);
// }

void read_sensor_r(void){
    // Turn LED on and wait a little
    HAL_GPIO_WritePin(GPIOC, RR_LED_Pin, SET); //LEDを点灯
    HAL_GPIO_WritePin(GPIOC, RR_LED_Pin, RESET); //LEDを消灯
}