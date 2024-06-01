/*
 * gyro.hpp
 *
 *  Created on: 2024/04/27
 *      Author: hashimoto
 */

#ifndef INC_GYRO_HPP_
#define INC_GYRO_HPP_

#include "../../Core/Inc/main.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <spi.h>
#include "usart.h"

#define WHO_AM_I 0x00
#define PWR_MGMT_1 0x06
#define GYRO_CONFIG_1 0x01

class Gyro {
public:
    uint8_t readByte(uint8_t reg);
    void writeByte(uint8_t reg, uint8_t data);
    void init(void);
    void update(void);
};

#endif /* INC_GYRO_HPP_ */
