#ifndef INC_PARAMS_HPP
#define INC_PARAMS_HPP
#include "main.h"

namespace MotorParam {
    const float Ke = 0.207/1000.0; // 逆起電圧定数[V/min^-1]
    const float Kt = 1.98/1000.0;   // トルク定数[Nm/A]
    const float R = 1.07;         // 巻線抵抗[Ω]
    const float GEAR_RATIO = 43.0/13.0;        // ギア比
    const float m = 90.0/1000.0;
    const float r = 12.0*0.001;           // タイヤ半径[m]
    const int bit = 4096; //
    const float stm32_vat = 3.3;
    const float BAT_RATIO = (25.0/10.0)*(stm32_vat/bit); // 電圧倍率(分圧)*（12bit=4096/3.3V)
    const float TREAD_WIDTH = 6.5/1000;
    const uint8_t RATE = 1;
    const float PULSE_PER_TIRE_ONEROTATION = 54193.2; // タイヤ一回転あたりのパルス数：4096*4*(43/13) = 54193.2..
}

namespace LinearVelocityPID {
    extern float target_a; // 加速度
    extern float vel_pid_error_sum;
    extern float target_linear_vel_mm;
    extern float current_linear_vel_mm;
    extern float calculated_linear_vel_mm;
    extern float current_distance;
    const float Kp = 0.1;
    const float Ki = 0.0;
    const float Kd = 0.0; // 使われていない
    const uint16_t MAX_PID_ERROR_SUM = 10;
    const int16_t MIN_PID_ERROR_SUM = 0;
}

namespace AngularVelocityPID {
    extern float target_alpha;  // 角加速度
    extern float w_pid_error_sum;
    extern float target_angular_vel;
    extern float current_angular_vel;
    extern float calculated_angular_vel;
    extern float current_angle;
    const float Kp = 1.0;
    const float Ki = 0.0;
    const float Kd = 0.0;
    // TODO: PIDのパラメータ調整
    const uint16_t MAX_PID_ERROR_SUM = 60;
    const int16_t MIN_PID_ERROR_SUM = 0;
}

namespace Battery {
    extern float adc_bat;
}

namespace RobotControllerParam {
    const float MAX_SPEED = 200; // [mm/s]
    const float ACCEL = 100; // [mm/s^2]
    const float DECEL = 100; // [mm/s^2]
    const float TARGET_DISTANCE = 0.3;
}

namespace Mode {
    enum class ModeType {
        WAIT,
        RUN,
        LOG
    };
}
#endif // INC_PARAMS_HPP
