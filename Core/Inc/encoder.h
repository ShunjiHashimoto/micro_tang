#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include "tim.h"
#include "math.h"

typedef struct {
    int32_t delta_pulse;
    int32_t total_pulse;
    TIM_HandleTypeDef* htim_x;
    int32_t one_rotation_pulse;
    uint32_t initial_pulse_count;
    float rad_per_rotation;
    float rotation_speed;
    bool forward_wise;
} Encoder;

// 初期化関数
void initEncoder(Encoder* encoder, TIM_HandleTypeDef* htim_x, float one_rotation_pulse, bool cw);

// 更新関数
void updateEncoder(Encoder* encoder, uint32_t pulse_count);

void resetEncoder(Encoder* encoder);

// delta_pulseを取得する関数
int32_t getDeltaPulse(const Encoder* encoder);

// total_pulseを取得する関数
int32_t getTotalPulse(const Encoder* encoder);

// 回転数を取得する関数
int32_t getRotationCount(const Encoder* encoder);

// グローバル変数として宣言
extern Encoder encoder_r;
extern Encoder encoder_l;

#endif // ENCODER_H