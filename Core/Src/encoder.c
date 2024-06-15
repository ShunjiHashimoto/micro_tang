#include "encoder.h"

void Encoder_Init(Encoder* encoder, TIM_HandleTypeDef* htim_x, int32_t one_rotation_pulse, bool cw) {
    encoder->delta_pulse = 0;
    encoder->total_pulse = 0;
    encoder->htim_x = htim_x;
    encoder->forward_wise = cw;
    encoder->one_rotation_pulse = one_rotation_pulse;
    encoder->rad_per_rotation = 2*M_PI / one_rotation_pulse;
    encoder->motor_vel = 0.0;
    encoder->initial_pulse_count = (__HAL_TIM_GET_AUTORELOAD(htim_x)+1)/2; // 必要に応じて初期値を設定
    __HAL_TIM_SET_COUNTER(encoder->htim_x, encoder->initial_pulse_count); // タイマをリセット
    HAL_TIM_Encoder_Start_IT(encoder->htim_x, TIM_CHANNEL_ALL); // 左車輪のエンコーダのカウントスタート
}

void Encoder_Update(Encoder* encoder, uint32_t cur_pulse) {
    encoder->delta_pulse = -encoder->initial_pulse_count + cur_pulse;
    encoder->total_pulse += (encoder->forward_wise ? -encoder->delta_pulse : encoder->delta_pulse);
    encoder->motor_vel = (encoder->forward_wise ? 
    (-encoder->delta_pulse * encoder->rad_per_rotation)/0.001: (encoder->delta_pulse * encoder->rad_per_rotation)/0.001);
}

int32_t Encoder_GetDeltaPulse(const Encoder* encoder) {
    return encoder->delta_pulse;
}

int32_t Encoder_GetTotalPulse(const Encoder* encoder) {
    return encoder->total_pulse;
}

int32_t Encoder_GetRotationCount(const Encoder* encoder) {
    return encoder->total_pulse / encoder->one_rotation_pulse;
}