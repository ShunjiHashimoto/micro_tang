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
        if(current_mode == Mode::ModeType::WAIT) {
            motor_r.run(GPIO_PIN_RESET, 0);
            motor_l.run(GPIO_PIN_RESET, 0);
            return;
        }
        float torque = (MotorParam::m*LinearVelocityPID::target_a*MotorParam::r)/MotorParam::GEAR_RATIO;
        LinearVelocityPID::current_linear_vel = (MotorParam::r*(encoder_r.rotation_speed) + MotorParam::r*(encoder_l.rotation_speed))/2.0;
        AngularVelocityPID::current_angular_vel  = gyro.angular_vel*GYRO_GAIN*M_PI/180;
        float calculated_linear_vel  = Motor::linearVelocityPIDControl(LinearVelocityPID::target_linear_vel, LinearVelocityPID::current_linear_vel, LinearVelocityPID::vel_pid_error_sum);
        float calculated_angular_vel = Motor::angularVelocityPIDControl(AngularVelocityPID::target_angular_vel, AngularVelocityPID::current_angular_vel, AngularVelocityPID::w_pid_error_sum);
        float rotation_speed_r       = motor_r.calcMotorSpeed(calculated_linear_vel, calculated_angular_vel); // [rpm]
        float rotation_speed_l       = motor_l.calcMotorSpeed(calculated_linear_vel, calculated_angular_vel); // [rpm]
        float duty_r = 100*(MotorParam::R*torque/MotorParam::Kt + MotorParam::Ke*rotation_speed_r)/Battery::adc_bat;
        float duty_l = 100*(MotorParam::R*torque/MotorParam::Kt + MotorParam::Ke*rotation_speed_l)/Battery::adc_bat;
        motor_r.run(GPIO_PIN_RESET, duty_r);
        motor_l.run(GPIO_PIN_SET, duty_l); 
    }
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

void Motor::run(GPIO_PinState direction, int duty) {
    HAL_GPIO_WritePin(GPIOA, mode_channel, mode);
    HAL_GPIO_WritePin(GPIOA, direction_channel, direction);
    __HAL_TIM_SET_COMPARE(&htim_x, duty_channel, duty);
    HAL_TIM_PWM_Start(&htim_x, duty_channel);
}
