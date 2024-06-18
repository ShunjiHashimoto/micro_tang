/*
 * gyro.c
 *
 *  Created on: 2024/04/27
 *      Author: hashimoto
 */
#include "gyro.h"

uint8_t Gyro_ReadByte(uint8_t reg)
{
  uint8_t rx_data[2];
  uint8_t tx_data[2];
  // 最上位ビットを1にすることでReadmodeを1にするため、0x80のorを取る
  tx_data[0] = reg | 0x80;
  tx_data[1] = 0x00;  // dummy

  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, tx_data, rx_data, 2, 100);
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  return rx_data[1];
}

void Gyro_WriteByte(uint8_t reg, uint8_t data)
{
  uint8_t rx_data[2];
  uint8_t tx_data[2];

  tx_data[0] = reg & 0x7F;
  tx_data[1] = data;  // write data

  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, tx_data, rx_data, 2, 1);
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

void Gyro_Init(Gyro* gyro_) {
    gyro_->ang_vel = 0.0;
    gyro_->yaw_deg = 0.0;
    uint8_t who_am_i;
  
    HAL_Delay( 100 ); // wait start up
    who_am_i = Gyro_ReadByte( WHO_AM_I ); // 1. read who am i 
    printf( " rn0x%xrn\n\r", who_am_i ); // 2. check who am i value, 0xE0が帰ってくる
    // 初回に失敗するときがあるので、もう一度動かしてみる
    HAL_Delay( 100 );
    who_am_i = Gyro_ReadByte( WHO_AM_I );  
    printf( " rn0x%xrn\n\r", who_am_i );

    // 2. error check
    if ( who_am_i != 0xE0 ){
      printf( " gyro_errorr\n\r");
      HAL_Delay(100);
    }
    printf("gyro startup");

    HAL_Delay( 50 ); // wait
    // sleep 状態から起こす、clearだから0
    // デフォルトでは省電力モード、今回は20Mhzで動かす、0で設定する
    // device resetも1で設定する、右側がかいビット、左が上位ビット
    Gyro_WriteByte(PWR_MGMT_1, 0x80); // 1000 0000 
    HAL_Delay(50);
    Gyro_WriteByte(PWR_MGMT_1, 0x01); // 0000 0001
    HAL_Delay(50);
    // GYRO_FS_SEL: 設定する必要あり、どこの速度まで角度取るかの設定、11（フルスケール）、旋回速度が遅いものは下げる必要
    Gyro_WriteByte(GYRO_CONFIG_1, 0x06); // 0000 0110
    HAL_Delay(100);
}

void Gyro_Update(Gyro* gyro_) {
    // HAL_Delay(100);
    uint8_t zout_h, zout_l;
    zout_h = Gyro_ReadByte(0x37);
    zout_l = Gyro_ReadByte(0x38);
    int16_t gyro_raw = (((uint16_t)zout_h<<8)&0xff00)|zout_l;
    gyro_->ang_vel = GYRO_OFFSET + (float)gyro_raw / SENSITIVITY_SCALE_FACTOR;
    gyro_->yaw_deg += gyro_->ang_vel * 0.001;
}

