/*
 * motor.cpp
 *
 *  Created on: 2023/06/25
 *      Author: hashimoto
 */

#include "motor.hpp"

extern Encoder encoder_r;
extern Encoder encoder_l;
extern Gyro gyro;
extern ModeManager mode_manager;

// 参照渡しの場合は、htim1は直接渡す
Motor motor_l(htim1, Motor_Mode_Pin, GPIO_PIN_SET, MotorL_TIM1_CH1_Pin, TIM_CHANNEL_2, -1);
Motor motor_r(htim1, Motor_Mode_Pin, GPIO_PIN_SET, MotorR_TIM1_CH3_Pin, TIM_CHANNEL_4, 1);

extern "C" {
    void pwmControl(){
        Mode::ModeType current_mode = mode_manager.getCurrentMode();
        if(current_mode == Mode::ModeType::WAIT || current_mode == Mode::ModeType::LOG) {
            CommonMotorControl::resetTargetVelocity();
            motor_r.Stop();
            motor_l.Stop();
            return;
        }
        float torque = CommonMotorControl::calcTorque(LinearVelocityPID::target_a);
        LinearVelocityPID::current_linear_vel = CommonMotorControl::calcCurrentLinearVel(encoder_r.rotation_speed, encoder_l.rotation_speed);
        AngularVelocityPID::current_angular_vel  = CommonMotorControl::calcCurrentAngularVel(gyro.angular_vel);
        LinearVelocityPID::calculated_linear_vel  = Motor::linearVelocityPIDControl(LinearVelocityPID::target_linear_vel, LinearVelocityPID::current_linear_vel, LinearVelocityPID::vel_pid_error_sum);
        AngularVelocityPID::calculated_angular_vel = Motor::angularVelocityPIDControl(AngularVelocityPID::target_angular_vel, AngularVelocityPID::current_angular_vel, AngularVelocityPID::w_pid_error_sum);
        motor_r.rotation_speed = motor_r.calcMotorSpeed(LinearVelocityPID::calculated_linear_vel, AngularVelocityPID::calculated_angular_vel); // [rpm]
        motor_l.rotation_speed = motor_l.calcMotorSpeed(LinearVelocityPID::calculated_linear_vel, AngularVelocityPID::calculated_angular_vel); // [rpm]
        motor_r.duty = motor_r.calcDuty(torque);
        motor_l.duty = motor_l.calcDuty(torque);
        motor_r.Run(GPIO_PIN_RESET);
        motor_l.Run(GPIO_PIN_SET); 
    }
}

float CommonMotorControl::calcTorque(float target_a) {
    return (MotorParam::m*target_a*MotorParam::r)/MotorParam::GEAR_RATIO;
}

float CommonMotorControl::calcCurrentLinearVel(float rotation_speed_r, float rotation_speed_l) {
    return ((MotorParam::r*rotation_speed_r) + (MotorParam::r*rotation_speed_l))/2.0;
}

float CommonMotorControl::calcCurrentAngularVel(float angular_vel) {
    return angular_vel*GYRO_GAIN*M_PI/180;
}

void CommonMotorControl::resetTargetVelocity() {
    LinearVelocityPID::target_linear_vel = 0.0;
    AngularVelocityPID::target_angular_vel = 0.0;
    // TODO: encoder値やpwm制御の値をリセット
    return;
}

Motor::Motor(TIM_HandleTypeDef &htim_x, uint16_t mode_channel, GPIO_PinState mode, uint16_t direction_channel, uint16_t duty_channel, int16_t left_or_right)
    : htim_x(htim_x), mode_channel(mode_channel), mode(mode), direction_channel(direction_channel), duty_channel(duty_channel), left_or_right(left_or_right) {
}

float Motor::calcMotorSpeed(float calculated_linear_vel, float calculated_angular_vel){
    float vel = calculated_linear_vel + left_or_right*(MotorParam::TREAD_WIDTH/2)*calculated_angular_vel;
    float rotation_speed = (vel/MotorParam::r)*MotorParam::GEAR_RATIO*60/(2*M_PI);
    return rotation_speed;
}

float Motor::linearVelocityPIDControl(float target_linear_vel, float current_linear_vel, float &pid_error_sum){
    float pid_error = LinearVelocityPID::Kp*(target_linear_vel - current_linear_vel)+ LinearVelocityPID::Ki*pid_error_sum;
    pid_error_sum += target_linear_vel - current_linear_vel;
    if(pid_error_sum > LinearVelocityPID::MAX_PID_ERROR_SUM) {
        pid_error_sum = LinearVelocityPID::MAX_PID_ERROR_SUM;
    }
    return target_linear_vel + pid_error;
}

float Motor::angularVelocityPIDControl(float target_angular_vel, float current_angular_vel, float &pid_error_sum){
    float pid_error = AngularVelocityPID::Kp*(target_angular_vel - current_angular_vel)+ AngularVelocityPID::Ki*pid_error_sum;
    pid_error_sum += target_angular_vel - current_angular_vel;
    if(pid_error_sum > AngularVelocityPID::MAX_PID_ERROR_SUM) {
        pid_error_sum = AngularVelocityPID::MAX_PID_ERROR_SUM;
    }
    return target_angular_vel + pid_error;
}

int Motor::calcDuty(float torque) {
    return 100*(MotorParam::R*torque/MotorParam::Kt + MotorParam::Ke*this->rotation_speed)/Battery::adc_bat;
}

void Motor::Run(GPIO_PinState direction) {
    if(this->duty == 0) {
        this->Stop();
        return;
    }
    HAL_GPIO_WritePin(GPIOA, this->mode_channel, this->mode);
    HAL_GPIO_WritePin(GPIOA, this->direction_channel, direction);
    //  duty デューティ比 {右：TIM_CHANNEL_4, 左：TIM_CHANNEL_2}
    __HAL_TIM_SET_COMPARE(&htim_x, this->duty_channel, this->duty);
    HAL_TIM_PWM_Start(&htim_x, this->duty_channel);
}

void Motor::Stop() {
    HAL_TIM_PWM_Stop(&htim_x, this->duty_channel);
}
