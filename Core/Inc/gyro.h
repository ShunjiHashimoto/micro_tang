#ifndef GYRO_H
#define GYRO_H

#include "../../Core/Inc/main.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <spi.h>
#include "usart.h"
#include "params.h"

typedef struct {
    float angular_vel; // [rad/s]
    float yaw_deg; // [rad]
} Gyro;

uint8_t Gyro_ReadByte(uint8_t reg);
void Gyro_WriteByte(uint8_t reg, uint8_t data);
void gyroInit(Gyro* gyro_);
void updateGyro(Gyro* gyro_);
extern Gyro gyro;

#endif /* GYRO_H */
