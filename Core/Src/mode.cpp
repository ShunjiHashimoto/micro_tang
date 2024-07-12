#include "mode.hpp"
#include <iostream>

ModeManager mode_manager;

extern "C" {
    void updateModeManager(){
        mode_manager.updateCurrentMode();
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
        printf("Run Mode\r\n");
        current_mode = Mode::ModeType::RUN;
    } 
    else if (sw1_state == 0) {
        printf("Log Mode\r\n");
        current_mode = Mode::ModeType::LOG;
    } 
    // 一度logを吐き出せばwaitモードに移行する
    else if (sw2_state == 0 or prev_mode == Mode::ModeType::LOG) {
        current_mode = Mode::ModeType::WAIT;
        printf("Wait Mode\r\n");
    }
    else {
        current_mode = prev_mode;
    }
    prev_mode = current_mode;
}

Mode::ModeType ModeManager::getCurrentMode() const {
    return current_mode;
}
