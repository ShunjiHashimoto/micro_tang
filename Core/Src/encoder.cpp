#include <encoder.hpp>

Encoder::Encoder(TIM_HandleTypeDef &htim_x, int32_t one_rotation_pulse, bool cw)
  : _delta_pulse(0)
  , _total_pulse(0)
  , _htim_x(htim_x)
  // タイマーのオートリロード値の半分から1を引いた値を初期値として設定することで、正逆両方向のパルスを均等に処理できる
  , _initial_pulse_count(__HAL_TIM_GET_AUTORELOAD(&_htim_x) / 2 - 1) 
  , _one_rotation_pulse(one_rotation_pulse)  
  , _forward_wise(cw){
}

void Encoder::update(const uint32_t pulse_count) {
    // タイマーのカウンタ値を基準点にリセットする
    __HAL_TIM_SET_COUNTER(&_htim_x, _initial_pulse_count);
    _delta_pulse = static_cast<int32_t>(pulse_count - _initial_pulse_count);
    // _forward_wise が true の時にカウントアップとする
    if (!_forward_wise) _delta_pulse *= -1;
    // _total_pulse を更新
    _total_pulse += _delta_pulse;
}
