#include "log.hpp"

Log vel_log;

extern "C" {
    float roundToTwoDecimalPlaces(float value) {
        return std::round(value * 100.0f) / 100.0f;
    }

    void updateLog() {
        // std::vector<float> log = {LinearVelocityPID::target_linear_vel, LinearVelocityPID::current_linear_vel};
        std::vector<float> log = {AngularVelocityPID::target_angular_vel, AngularVelocityPID::current_angular_vel};
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
