// motor_params.h
#ifndef MOTOR_PARAMS_H
#define MOTOR_PARAMS_H

namespace MotorParam {
    const float Ke = 0.207/1000.0; // 逆起電圧定数[V/min^-1]
    const float Kt = 1.98/1000.0;   // トルク定数[Nm/A]
    const float R = 1.07;         // 巻線抵抗[Ω]
    const float GEAR_RATIO = 43.0/13.0;        // ギア比
    const float m = 90.0/1000.0;
    const float r = 11.5*0.001/2;           // タイヤ半径[m]
    const int bit = 4096; //
    const float stm32_vat = 3.3;
    const float BAT_RATIO = (25.0/10.0)*(stm32_vat/bit); // 電圧倍率(分圧)*（12bit=4096/3.3V)
    const float TREAD_WIDTH = 6.5/1000;
    const uint16_t RATE = 10;
}

namespace LinearVelocityPID {
    const float Kp = 0.1;
    const float Ki = 0.1;
    const float Kd = 0.0;
    const uint16_t MAX_PID_ERROR_SUM = 10;
    const int16_t MIN_PID_ERROR_SUM = 0;
}

namespace AngularVelocityPID {
    const float Kp = 0.1;
    const float Ki = 0.01;
    const float Kd = 0.0;
    const uint16_t MAX_PID_ERROR_SUM = 10;
    const int16_t MIN_PID_ERROR_SUM = 0;
}
#endif // MOTOR_PARAMS_H