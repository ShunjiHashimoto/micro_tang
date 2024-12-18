#ifndef INC_ROBOT_CONTROLLER_HPP
#define INC_ROBOT_CONTROLLER_HPP

#include "main.h"
#include "params.hpp"
#include "mode.hpp"
#include "motor.hpp"
#include "log.hpp"
#include "adc.h"
#include "photo_trans.hpp"
#include "utils.hpp"


// TODO:
// pwmcontrolをここに追加する？

class RobotController {
    public:
        RobotController();
        void mainControl();
        void motorControl(float target_linear_vel, float target_angular_vel);
        void allMotorStop();
        void straight(float target_distance);
        void turn_right(uint16_t target_deg);
        void turn_left(uint16_t target_deg);
        float calculateDeltaTime(unsigned long current_count, unsigned long& prev_count, unsigned long timer_max_count);
        float getCalculatedLinearVel();
        float getCalculatedAngularVel();
        float getCurrentLinearVel();
        float getCurrentAngularVel();
};


#endif /* INC_ROBOT_CONTROLLER_HPP_ */