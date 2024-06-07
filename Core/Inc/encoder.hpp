/*
 * encoder.hpp/*
 * encoder.hpp
 *
 *  Created on: 2024/06/04
 *      Author: hashimoto
 */

#include "main.h"
#include "timers.hpp"
#include "tim.h"

class Encoder {
public:
    /**
     * @brief Encoder コンストラクタ
     * @param htim_x 使用するタイマー
     * @param one_rotation_pulse １回転あたりのパルス数 
     * @param cw 回転方向（）
    */
    Encoder(TIM_HandleTypeDef &htim_x, int32_t  one_rotation_pulse, bool cw);
    void update(const uint32_t pulse_count);
    int32_t getDeltaPulse() const { return _delta_pulse; }
    int32_t getTotalPulse() const { return _total_pulse; }
    int32_t getRotationCount() const { return _total_pulse / _one_rotation_pulse; }

private:
    int32_t _delta_pulse;
    int32_t _total_pulse;
    TIM_HandleTypeDef& _htim_x;
    const bool _forward_wise;
    const int32_t _one_rotation_pulse;
    uint32_t _initial_pulse_count;
};