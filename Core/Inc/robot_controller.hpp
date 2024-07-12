#ifndef INC_ROBOT_CONTROLLER_HPP
#define INC_ROBOT_CONTROLLER_HPP

#include "main.h"
#include "params.hpp"
#include "mode.hpp"
#include "motor.hpp"
#include "log.hpp"
#include "adc.h"


// TODO:
// pwmcontrolをここに追加する？

class RobotController {
    public:
        RobotController();
        void mainControl();
        void motorControl(float target_linear_vel, float target_angular_vel);
        float getCalculatedLinearVel();
        float getCalculatedAngularVel();
        float getCurrentLinearVel();
        float getCurrentAngularVel();
};


#endif /* INC_ROBOT_CONTROLLER_HPP_ */