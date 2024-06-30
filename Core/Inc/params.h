#ifndef PARAMS_H
#define PARAMS_H

#define WHO_AM_I 0x00
#define PWR_MGMT_1 0x06
#define GYRO_CONFIG_1 0x01

#define SENSITIVITY_SCALE_FACTOR 16.4 
#define GYRO_OFFSET 2.558
#define GYRO_GAIN 0.09 // IMUが-2000~2000だったので、-180~180にするために0.90をかけた

#endif
