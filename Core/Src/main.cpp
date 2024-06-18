/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern "C" {
  #include "encoder.h"
  #include "gyro.h"
}
#include "led.hpp"
#include "motor.hpp"
#include "params.hpp"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern "C" int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 100);
    return ch;
}
void encoder_init(void) {
  HAL_TIM_Base_Start_IT(&htim4); // 割り込み処理開始
  // タイヤ一回転あたりのパルス数：4096*4*(43/13) = 54193.2..
  Encoder_Init(&encoder_l, &htim2, 54193, true);
  Encoder_Init(&encoder_r, &htim3, 54193, false);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  LedBlink ledBlink;
  // Gyro gyro;
  // 参照渡しの場合は、htim1は直接渡す
  Motor motor_l(htim1, Motor_Mode_Pin, GPIO_PIN_SET, MotorL_TIM1_CH1_Pin, TIM_CHANNEL_2, -1);
  Motor motor_r(htim1, Motor_Mode_Pin, GPIO_PIN_SET, MotorR_TIM1_CH3_Pin, TIM_CHANNEL_4, 1);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  setbuf(stdout, NULL); // std::outのバッファリングを無効にし、ログを即出力する
  Gyro_Init(&gyro);
  encoder_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t sw0_state = 1; 
  float adc_bat = 0.0;
  float target_linear_vel = 0.01; // [m/s]
  float target_angular_vel = 0.5;
  float a = 18;
  float torque = (MotorParam::m*a*MotorParam::r)/MotorParam::GEAR_RATIO;
  float vel_pid_error_sum = 0.0;
  float w_pid_error_sum = 0.0;
  while (1){
    // Lチカ
    ledBlink.toggle();
    // IMU
    // gyro.update();
    // sw状態の確認
    if(sw0_state != 0) sw0_state = HAL_GPIO_ReadPin(SW0_GPIO_Port,SW0_Pin);
    // バッテリー電圧の取得
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 1000);
    adc_bat = HAL_ADC_GetValue(&hadc2) * MotorParam::BAT_RATIO;
    if(sw0_state == 0) {
      // 速度制御
      float current_linear_vel     = (MotorParam::r*(encoder_r.rotation_speed) + MotorParam::r*(encoder_l.rotation_speed))/2.0;
      float current_angular_vel    = gyro.ang_vel*0.09*M_PI/180;
      float calculated_linear_vel  = Motor::linearVelocityPIDControl(target_linear_vel, current_linear_vel, vel_pid_error_sum);
      float calculated_angular_vel = Motor::angularVelocityPIDControl(target_angular_vel, current_angular_vel, w_pid_error_sum);
      float rotation_speed_r       = motor_r.calcMotorSpeed(calculated_linear_vel, calculated_angular_vel); // [rpm]
      float rotation_speed_l       = motor_l.calcMotorSpeed(calculated_linear_vel, calculated_angular_vel); // [rpm]
      float duty_r = 100*(MotorParam::R*torque/MotorParam::Kt + MotorParam::Ke*rotation_speed_r)/adc_bat;
      float duty_l = 100*(MotorParam::R*torque/MotorParam::Kt + MotorParam::Ke*rotation_speed_l)/adc_bat;
      motor_r.run(GPIO_PIN_RESET, duty_r);
      motor_l.run(GPIO_PIN_SET, duty_l); 
      // ログ
      // printf("pid_error_sum %lf , w %lf\n\r", vel_pid_error_sum, w_pid_error_sum);
      // printf("Battery Voltage: %lf\n\r", adc_bat); 
      // printf("delta %d, Rotation Num: l: %d r:%d, Rotation Vel: l: %lf r: %lf\n\r",
      //         encoder_r.delta_pulse, Encoder_GetRotationCount(&encoder_l), Encoder_GetRotationCount(&encoder_r),
      //         encoder_l.rotation_speed, encoder_r.rotation_speed);
      // printf("rotation_speed_r: %lf, rotation_speed_l: %lf, torque: %lf, duty: r: %lf, l: %lf\n\r", 
      //         rotation_speed_r, rotation_speed_l, torque, duty_r, duty_l);
      // TODO IMUを使ったPID制御の実装とリファクタリングとバッテリーケーブルの修理
      // メモリスト、C言語とC++で参照渡しが異なる。エンコーダ、IMUのグローバル変数の見直し
      // IMUが-2000~2000だったので、-180~180にするために0.90をかけた
      // PID制御の+pid_error, I成分はtarget-currentで、pid_errorを足し合わせるものではない
      printf("cur_vel %lf tar_vel %lf , cur_w %lf tar_w %lf\n\r", current_linear_vel, calculated_linear_vel, current_angular_vel, calculated_angular_vel);
    }
    // printf("gyro vel: %lf yaw deg: %lf\n\r", gyro.ang_vel*0.09*M_PI/180, gyro.yaw_deg*0.09);
    HAL_Delay(MotorParam::RATE);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
