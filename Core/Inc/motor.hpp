/*
 * motor.hpp
 *
 *  Created on: 2023/06/25
 *      Author: hashimoto
 */

#ifndef INC_DRIVERS_MOTOR_HPP_
#define INC_DRIVERS_MOTOR_HPP_
#include "main.h"
#include "params.hpp"
#include "timers.hpp"
#include "tim.h"

class Motor {
public:
    /** 
     * @param htim_x タイマーの指定
     * @param mode_channel Motor Mode Pin
     * @param mode モード選択（1でPhase/Enable Mode）
     * @param direction_channel 回転方向のPin指定 {MotorR_TIM1_CH3_Pin, MotorL_TIM1_CH1_Pin}
     * @param duty_channel デューティ比のPin指定
    */
    Motor(TIM_HandleTypeDef &htim_x, uint16_t mode_channel, GPIO_PinState mode, uint16_t direction_channel, uint16_t duty_channel, int16_t left_or_right);

    float calcMotorSpeed(float linear_vel, float angular_vel);

    /**
     * @brief モーター制御関数
     * @param direction 回転方向（0は時計回り、1は反時計回り）
     * @param duty デューティ比 {右：TIM_CHANNEL_4, 左：TIM_CHANNEL_2}
    */
    void run(GPIO_PinState direction, int duty);

private:
    TIM_HandleTypeDef &htim_x;
    GPIO_PinState mode;
    uint16_t mode_channel;
    uint16_t direction_channel;
    uint16_t duty_channel;
    int16_t left_or_right;
};

#endif /* INC_DRIVERS_MOTOR_HPP_ */
