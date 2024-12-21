#ifndef PARAMS_H
#define PARAMS_H

#define WHO_AM_I 0x00
#define PWR_MGMT_1 0x06
#define GYRO_CONFIG_1 0x01

#define SENSITIVITY_SCALE_FACTOR 16.4 
#define GYRO_OFFSET 2.558
#define GYRO_GAIN_R 0.125 // IMUが-1600 ~ 1600だったので、-180~180にするために0.12をかけた
#define GYRO_GAIN_L 0.125 // IMU値が左右方向で差があったためゲインを少し変更
// 左回転だと足りず、右回転だと行き過ぎ
// 左回転のログ
// 0.115: 5.5回転
// 0.12: 5.25回転
// 0.125: 5.05回転
// 右回転のログ
// 0.12: 5.25
// 0.125: 4.65

#endif
