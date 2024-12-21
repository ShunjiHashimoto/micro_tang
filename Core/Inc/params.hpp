#ifndef INC_PARAMS_HPP
#define INC_PARAMS_HPP
#include "main.h"

namespace MotorParam {
    // const float Ke = (2*3.1415926/60)*(0.207/1000.0); // 逆起電圧定数[V*s/rad], 0.00002168
    const float Ke = (2*3.1415926/60)*0.207; // 逆起電圧定数[V*s/rad], 0.00002168
    // const float Kt = 1.98/1000.0;   // トルク定数[Nm/A]
    const float Kt = 1.98;   // トルク定数[Nm/A]
    const float R = 1.07;         // 巻線抵抗[Ω]
    const float GEAR_RATIO = 43.0/13.0;        // ギア比
    // const float m = 90.0/1000.0;
    const float m = 90.0; // [g]
    const float r = (12.0 + 0.6)*0.001;  // タイヤ半径[m], 両面テープの厚さ分0.1*2
    const int bit = 4096; //
    const float stm32_vat = 3.3;
    const float BAT_RATIO = (25.0/10.0)*(stm32_vat/bit); // 電圧倍率(分圧)*（12bit=4096/3.3V)
    const float TREAD_WIDTH = 6.5/1000;
    const uint8_t RATE = 1;
    const float PULSE_PER_TIRE_ONEROTATION = 54193.2; // タイヤ一回転あたりのパルス数：4096*4*(43/13) = 54193.2..
    const float DUTY_GAIN = 1.1; // 右モータのゲイン
    // 左にどんどん擦れている: 1.23
    // もっとずれた: 1.25
    // マシになった： 1.20
    // 変わらず： 1.15
    // 変わらず: 1.1
}

namespace LinearVelocityPID {
    extern float target_a; // 加速度
    extern float vel_pid_error_sum;
    extern float target_linear_vel;
    extern float current_linear_vel;
    extern float calculated_linear_vel;
    extern float current_distance;
    const float Kp = 10.0;
    const float Ki = 0.3;
    const float Kd = 0.0;
    const uint16_t MAX_PID_ERROR_SUM = 10;
    const int16_t MIN_PID_ERROR_SUM = 0;
    const float MAX_SPEED = 1000.0; // [mm/s]
    const float MIN_SPEED = 100.0; // [mm/s]
}

namespace AngularVelocityPID {
    extern float target_alpha;  // 角加速度
    extern float w_pid_error_sum;
    extern float target_angular_vel;
    extern float current_angular_vel;
    extern float calculated_angular_vel;
    extern float current_angle;
    const float Kp = 100.0;
    const float Ki = 0.8;
    const float Kd = 0.0;
    // TODO: PIDのパラメータ調整
    const uint16_t MAX_PID_ERROR_SUM = 60;
    const int16_t MIN_PID_ERROR_SUM = 0;
}

namespace Battery {
    extern float adc_bat;
}

namespace RobotControllerParam {
    const float MAX_SPEED = 300.0; // [mm/s]
    const float MAX_OMEGA = 200.0; // [rad/s]
    // const float ACCEL = 50; // [mm/s^2]
    // const float DECEL = 50; // [mm/s^2]
    const float TARGET_DISTANCE = 0.3;
    const int MIN_DISTANCE_TO_RUN = 5;
}

namespace Mode {
    enum class ModeType {
        WAIT,
        RUN,
        LOG
    };
}

namespace ADCParam {
    const int SENSOR_COUNT = 4;
}
#endif // INC_PARAMS_HPP
