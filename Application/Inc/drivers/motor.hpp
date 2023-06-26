/*
 * motor.hpp
 *
 *  Created on: 2023/06/25
 *      Author: hashimoto
 */

#ifndef INC_DRIVERS_MOTOR_HPP_
#define INC_DRIVERS_MOTOR_HPP_
#include "../../Core/Inc/main.h"
#include "../pins/timers.hpp"

class Motor {
public:
    void run(uint16_t direction_channel, GPIO_PinState direction, uint32_t mode_channel, uint32_t duty_channel, int duty);
    void toggle();
};

#endif /* INC_DRIVERS_MOTOR_HPP_ */
