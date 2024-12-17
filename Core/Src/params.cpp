#include "params.hpp"

namespace LinearVelocityPID {
    float target_a = 200.0;
    float vel_pid_error_sum = 0.0;
    float target_linear_vel = 0.0;
    float current_linear_vel = 0.0;
    float calculated_linear_vel = 0.0;
    float current_distance = 0.0;
}

namespace AngularVelocityPID {
    float target_angular_vel = 0.0;
    float w_pid_error_sum = 0.0;
    float current_angular_vel = 0.0;
    float calculated_angular_vel = 0.0;
    float current_angle = 0.0;
}

namespace Battery {
    float adc_bat = 0.0;
}

namespace Mode {
    uint8_t current_mode = 0;
}

namespace ADCParam {
    uint16_t adc_val[4] = {0};
}