#include "robot_controller.hpp"

extern ModeManager mode_manager;
extern Motor motor_l;
extern Motor motor_r;
extern Log vel_log;

RobotController::RobotController(){
};

void RobotController::motorControl(float target_linear_vel, float target_angular_vel) {
    LinearVelocityPID::target_linear_vel = target_linear_vel;
    AngularVelocityPID::target_angular_vel = target_angular_vel;
    return;
}

void RobotController::linearRun(float distance) {
    // 指定した距離を走行する
}

float RobotController::getCalculatedLinearVel() {
    return LinearVelocityPID::calculated_linear_vel;
}

float RobotController::getCalculatedAngularVel() {
    return AngularVelocityPID::calculated_angular_vel;
}

float RobotController::getCurrentLinearVel() {
    return LinearVelocityPID::current_linear_vel;
}

float RobotController::getCurrentAngularVel() {
    return AngularVelocityPID::current_angular_vel;
}

void RobotController::mainControl(){
    Mode::ModeType current_mode = mode_manager.getCurrentMode();

    if(current_mode == Mode::ModeType::RUN) {
    //   this->motorControl(0.1, 0.0);
      printf("cur_vel %lf tar_vel %lf\n\r", LinearVelocityPID::current_linear_vel, LinearVelocityPID::target_linear_vel);
    //   printf("tar_vel %lf cur_vel %lf\n\r", AngularVelocityPID::target_angular_vel, this->getCurrentAngularVel());
    //   printf("duty_r %d, duty_l %d\n\r", motor_r.duty, motor_l.duty);
    //   printf("current_distance: %lf angle: %lf\n\r", LinearVelocityPID::current_distance, AngularVelocityPID::current_angle);
      return;
    }

    else if(current_mode == Mode::ModeType::LOG) {
      // TODO: logは繰り返し出す必要はない
      vel_log.printLog();
      HAL_Delay(2000);
      return;
    }
    
}