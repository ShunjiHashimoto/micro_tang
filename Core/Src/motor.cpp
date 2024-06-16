/*
 * motor.cpp
 *
 *  Created on: 2023/06/25
 *      Author: hashimoto
 */

#include "motor.hpp"
#include "math.h"

Motor::Motor(TIM_HandleTypeDef &htim_x, uint16_t mode_channel, GPIO_PinState mode, uint16_t direction_channel, uint16_t duty_channel, int16_t left_or_right)
    : htim_x(htim_x), mode_channel(mode_channel), mode(mode), direction_channel(direction_channel), duty_channel(duty_channel), left_or_right(left_or_right) {
}

float Motor::calcMotorSpeed(float linear_vel, float angular_vel){
    float vel = linear_vel + left_or_right*(MotorParam::TREAD_WIDTH/2)*angular_vel;
    float w = (vel/MotorParam::r)*MotorParam::GEAR_RATIO*60/(2*M_PI);
    return w;
}

float Motor::linearVelocityPIDControl(float target_v, float current_v, float &pid_error_sum){
    // float pid_error = LinearVelocityPID::Kp*(target_v - current_v) + LinearVelocityPID::Ki*pid_error_sum + LinearVelocityPID::Kd*(target_v-current_v)/(MotorParam::RATE/1000);
    float pid_error = LinearVelocityPID::Kp*(target_v - current_v)+ LinearVelocityPID::Ki*pid_error_sum;
    // if(pid_error_sum < LinearVelocityPID::MAX_PID_ERROR_SUM) {
    pid_error_sum += pid_error;
    // }
    return target_v - pid_error;
}

float Motor::angularVelocityPIDControl(float target_w, float current_w, float &pid_error_sum){
    // float pid_error = AngularVelocityPID::Kp*(target_w - current_w) + AngularVelocityPID::Ki*pid_error_sum + AngularVelocityPID::Kd*(target_w - current_w)/(MotorParam::RATE/1000);
    float pid_error = AngularVelocityPID::Kp*(target_w - current_w)+ AngularVelocityPID::Ki*pid_error_sum;
    // if(pid_error_sum < AngularVelocityPID::MAX_PID_ERROR_SUM) {
    pid_error_sum += pid_error;
    // }
    return target_w - pid_error;
}

void Motor::run(GPIO_PinState direction, int duty) {
    HAL_GPIO_WritePin(GPIOA, mode_channel, mode);
    HAL_GPIO_WritePin(GPIOA, direction_channel, direction);
    __HAL_TIM_SET_COMPARE(&htim_x, duty_channel, duty);
    HAL_TIM_PWM_Start(&htim_x, duty_channel);
}