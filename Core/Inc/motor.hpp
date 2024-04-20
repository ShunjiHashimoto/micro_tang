/*
 * motor.hpp
 *
 *  Created on: 2023/06/25
 *      Author: hashimoto
 */

#ifndef INC_DRIVERS_MOTOR_HPP_
#define INC_DRIVERS_MOTOR_HPP_
#include "main.h"
#include "timers.hpp"
#include "tim.h"

class Motor {
public:
    /**
     * @brief モーター制御関数
     * @param direction_channel 回転方向のPin指定 {MotorR_TIM1_CH3_Pin, MotorL_TIM1_CH1_Pin}
     * @param direction 回転方向（0は時計回り、1は反時計回り）
     * @param mode_channel モード選択（1でPhase/Enable Mode）
     * @param duty_channel デューティ比のPin指定
     * @param duty デューティ比 {右：TIM_CHANNEL_4, 左：TIM_CHANNEL_2}
    */
    void run(uint16_t direction_channel, GPIO_PinState direction, uint32_t mode_channel, uint32_t duty_channel, int duty);
    void toggle();
};

#endif /* INC_DRIVERS_MOTOR_HPP_ */
