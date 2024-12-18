#ifndef PARAMS_H
#define PARAMS_H

#define WHO_AM_I 0x00
#define PWR_MGMT_1 0x06
#define GYRO_CONFIG_1 0x01

#define SENSITIVITY_SCALE_FACTOR 16.4 
#define GYRO_OFFSET 2.558
#define GYRO_GAIN_R 0.12 // IMUが-1600 ~ 1600だったので、-180~180にするために0.12をかけた
#define GYRO_GAIN_L 0.13 // IMU値が左右方向で差があったためゲインを少し変更

#endif
