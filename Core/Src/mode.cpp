#include "mode.hpp"
#include <iostream>

ModeManager mode_manager;
Log vel_log;

extern "C" {
    float roundToTwoDecimalPlaces(float value) {
        return std::round(value * 100.0f) / 100.0f;
    }

    void updateModeManager(){
        mode_manager.updateCurrentMode();
    }

    void saveLog() {
        // std::vector<float> log_linear_vel = {LinearVelocityPID::target_linear_vel, LinearVelocityPID::current_linear_vel};
        std::vector<float> log_linear_vel = {AngularVelocityPID::target_angular_vel, AngularVelocityPID::current_angular_vel};
        // 四捨五入を適用してlog_linear_velに再度格納
        for (auto& vel : log_linear_vel) {
            vel = roundToTwoDecimalPlaces(vel);
        }
        vel_log.saveLog(log_linear_vel);
    }
}


ModeManager::ModeManager() : current_mode(Mode::ModeType::WAIT) {}


void ModeManager::initializeSwitches() {
    std::cout << "Switches initialized." << std::endl;
}

void ModeManager::updateCurrentMode() {
    uint8_t sw0_state = HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin);
    uint8_t sw1_state = HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin);
    uint8_t sw2_state = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);

    if (sw0_state == 0) {
        printf("Wait Mode\r\n");
        current_mode = Mode::ModeType::WAIT;
    } 
    else if (sw1_state == 0) {
        printf("Run Mode\r\n");
        current_mode = Mode::ModeType::RUN;
    } 
    else if (sw2_state == 0) {
        printf("Log Mode\r\n");
        current_mode = Mode::ModeType::LOG;
    }
    else {
        current_mode = prev_mode;
    }
    prev_mode = current_mode;
}

Mode::ModeType ModeManager::getCurrentMode() const {
    return current_mode;
}
