#include "log.hpp"

Log vel_log;
extern Encoder encoder_r;
extern Encoder encoder_l;

extern "C" {
    float roundToTwoDecimalPlaces(float value) {
        return std::round(value * 1000.0f) / 1000.0f;
    }

    void updateLog() {
        // std::vector<float> log = {LinearVelocityPID::target_linear_vel, LinearVelocityPID::current_linear_vel};
        std::vector<float> log = {LinearVelocityPID::current_distance, LinearVelocityPID::target_linear_vel, LinearVelocityPID::current_linear_vel, encoder_r.rotation_speed, encoder_l.rotation_speed};
        // 四捨五入を適用してlogに再度格納
        for (auto& vel : log) {
            vel = roundToTwoDecimalPlaces(vel);
        }
        vel_log.saveLog(log);
    }
}

void Log::saveLog(const std::vector<float>& new_vals) {
    logMatrix_.push_back(new_vals);
    return;
}

void Log::printLog() const {
    for (const auto& row : logMatrix_) {
            for(int i=0; i < (int)row.size(); ++i) {
                printf("%lf", row[i]);
                if(i < (int)row.size() - 1) {
                    printf(", ");
                }
        }
        printf("\n\r");
    }
    return;
}
