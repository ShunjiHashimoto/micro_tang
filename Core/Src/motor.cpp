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
LedBlink ledBlink;

extern "C" {
    void pwmControl() {
        Mode::ModeType current_mode = mode_manager.getCurrentMode();
        if(current_mode == Mode::ModeType::WAIT || current_mode == Mode::ModeType::LOG) {
            // CommonMotorControl::resetTargetVelocity();
            motor_r.Stop();
            motor_l.Stop();
            return;
        }
        float torque = CommonMotorControl::calcTorque(LinearVelocityPID::target_a);
        LinearVelocityPID::current_linear_vel = CommonMotorControl::calcCurrentLinearVel(encoder_r.rotation_speed, encoder_l.rotation_speed);
        AngularVelocityPID::current_angular_vel  = CommonMotorControl::calcCurrentAngularVel(gyro.angular_vel);
        LinearVelocityPID::current_distance += LinearVelocityPID::current_linear_vel; // 0.001*1000, [mm]
        AngularVelocityPID::current_angle  += AngularVelocityPID::current_angle * 0.001;

        LinearVelocityPID::calculated_linear_vel  = Motor::linearVelocityPIDControl(LinearVelocityPID::target_linear_vel, LinearVelocityPID::current_linear_vel, LinearVelocityPID::vel_pid_error_sum);
        AngularVelocityPID::calculated_angular_vel = Motor::angularVelocityPIDControl(AngularVelocityPID::target_angular_vel, AngularVelocityPID::current_angular_vel, AngularVelocityPID::w_pid_error_sum);
        motor_r.rotation_speed = motor_r.calcMotorSpeed(LinearVelocityPID::calculated_linear_vel, AngularVelocityPID::calculated_angular_vel); // [rpm]
        motor_l.rotation_speed = motor_l.calcMotorSpeed(LinearVelocityPID::calculated_linear_vel, AngularVelocityPID::calculated_angular_vel); // [rpm]
        int duty_r = motor_r.calcDuty(torque);
        int duty_l = motor_l.calcDuty(torque);
        if(duty_r < 0) duty_r = 0;
        if(duty_l < 0) duty_l = 0;
        motor_r.duty = duty_r;
        motor_l.duty = duty_l;
        motor_r.Run(GPIO_PIN_RESET);
        motor_l.Run(GPIO_PIN_SET); 
    }
    // void updateTargetVelocity(float target_distance) {
        // 制御周期：1[mm/sec]
    // }
    // void calcTrapezoidalProfile(float target_distance) {
    //     Mode::ModeType current_mode = mode_manager.getCurrentMode();
    //     if(current_mode == Mode::ModeType::WAIT || current_mode == Mode::ModeType::LOG) {
    //         return;
    //     }
    //     if(target_distance = 0.0) return;
    //     float v_max = RobotControllerParam::MAX_SPEED;
    //     float a = RobotControllerParam::ACCEL;
    //     // 加速・減速距離の計算
    //     float t_acc = v_max/a;
    //     float d_acc = 0.5*a*t_acc*t_acc;
    //     // 定速距離の計算
    //     float d_const = target_distance - 2*d_acc;
    //     // 定速距離が負の場合は、定速距離をなくし、加速・減速距離のみとする
    //     if(d_const < 0) {
    //         d_const = 0;
    //         d_acc = target_distance/2;
    //         t_acc = sqrt(2*d_acc/a);
    //     }
    //     // 定速距離を最大速度で走行する場合の走行時間
    //     float t_const = d_const/v_max;
    // }
}

float CommonMotorControl::calcTorque(float target_a) {
    return (MotorParam::m*target_a*MotorParam::r)/MotorParam::GEAR_RATIO;
}

// 車体の線形速度vを計算, rotation_speedはモータの角速度w
float CommonMotorControl::calcCurrentLinearVel(float rotation_speed_r, float rotation_speed_l) {
    return ((MotorParam::r*rotation_speed_r) + (MotorParam::r*rotation_speed_l))/2.0;
}

float CommonMotorControl::calcCurrentAngularVel(float angular_vel) {
    return angular_vel*GYRO_GAIN*M_PI/180;
}

void CommonMotorControl::resetTargetVelocity() {
    LinearVelocityPID::target_linear_vel = 0.0;
    AngularVelocityPID::target_angular_vel = 0.0;
    LinearVelocityPID::current_distance = 0.0;
    AngularVelocityPID::current_angle = 0.0;
    // TODO: encoder値やpwm制御の値をリセット
    resetEncoder(&encoder_l);
    resetEncoder(&encoder_r);
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
