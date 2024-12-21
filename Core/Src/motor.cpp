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
        LinearVelocityPID::current_linear_vel = CommonMotorControl::calcCurrentLinearVel(encoder_r.rotation_speed, encoder_l.rotation_speed); //[mm]
        LinearVelocityPID::current_distance += LinearVelocityPID::current_linear_vel * 0.001;; // [mm/sec]*0.001[sec]
        AngularVelocityPID::current_angular_vel  = gyro.angular_vel;
        AngularVelocityPID::current_angle = gyro.yaw_deg;

        LinearVelocityPID::calculated_linear_vel  = Motor::linearVelocityPIDControl(LinearVelocityPID::target_linear_vel, LinearVelocityPID::current_linear_vel, LinearVelocityPID::vel_pid_error_sum);
        AngularVelocityPID::calculated_angular_vel = Motor::angularVelocityPIDControl(AngularVelocityPID::target_angular_vel, AngularVelocityPID::current_angular_vel, AngularVelocityPID::w_pid_error_sum);
        // タイヤの回転速度は[rpm]ではなく、[rad/s], 300[rad/s]くらいが定格, 1500が最大
        motor_r.rotation_speed = motor_r.calcMotorSpeed(LinearVelocityPID::calculated_linear_vel, AngularVelocityPID::calculated_angular_vel); // [rpm]
        motor_l.rotation_speed = motor_l.calcMotorSpeed(LinearVelocityPID::calculated_linear_vel, AngularVelocityPID::calculated_angular_vel); // [rpm]
        int duty_r = motor_r.calcDuty(torque);
        int duty_l = motor_l.calcDuty(torque);
        motor_r.duty = abs(duty_r)*MotorParam::DUTY_GAIN;
        motor_l.duty = abs(duty_l);
        motor_r.Run(duty_r < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        motor_l.Run(duty_l < 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

float CommonMotorControl::calcTorque(float target_a) {
    // shishikawaさんのmotor.jsより(r*m*a)/(2*n)
    // https://github.com/meganetaaan/M5Mouse/blob/master/mouse/motor.js
    return (MotorParam::m*target_a*0.001*MotorParam::r)/(2*MotorParam::GEAR_RATIO);
}

// 車体の線形速度v[mm/s]を計算, rotation_speedはモータ（車輪ではなく）の角速度w[rad/s]
float CommonMotorControl::calcCurrentLinearVel(float rotation_speed_r, float rotation_speed_l) {
    // エンコーダから計算されたrotation_speedは車輪の回転速度
    float wheel_speed_r = rotation_speed_r;
    float wheel_speed_l = rotation_speed_l;
    return 1000*((MotorParam::r*wheel_speed_r) + (MotorParam::r*wheel_speed_l))/2.0;
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
    // w_R = v/r + (tread/2r)*w       
    // v: 線形速度[m/s]、r:タイヤ半径[m]、tread: トレッド幅[m], 2:車体の角速度[rad/s]
    // float vel = calculated_linear_vel*0.001 + left_or_right*(MotorParam::TREAD_WIDTH/2)*calculated_angular_vel; // [mm/s]
    // float rotation_speed = (vel/MotorParam::r)*MotorParam::GEAR_RATIO*60/(2*M_PI);
    float rotation_speed = (calculated_linear_vel*0.001/MotorParam::r) + left_or_right*(MotorParam::TREAD_WIDTH/(2*MotorParam::r))*calculated_angular_vel;
    // ギア比を考慮
    return rotation_speed*MotorParam::GEAR_RATIO;
}

float Motor::linearVelocityPIDControl(float target_linear_vel, float current_linear_vel, float &pid_error_sum){
    float pid_error = LinearVelocityPID::Kp*(target_linear_vel - current_linear_vel)+ LinearVelocityPID::Ki*pid_error_sum;
    pid_error_sum += target_linear_vel - current_linear_vel;
    if(pid_error_sum > LinearVelocityPID::MAX_PID_ERROR_SUM) {
        pid_error_sum = LinearVelocityPID::MAX_PID_ERROR_SUM;
    }
    if(LinearVelocityPID::MAX_SPEED < abs(target_linear_vel + pid_error)){
        return (target_linear_vel+pid_error) > 0 ? RobotControllerParam::MAX_SPEED : 0.0;
    }
    if(target_linear_vel + pid_error <0) return 0.0;
    return target_linear_vel + pid_error;
}

float Motor::angularVelocityPIDControl(float target_angular_vel, float current_angular_vel, float &pid_error_sum){
    float pid_error = AngularVelocityPID::Kp*(target_angular_vel - current_angular_vel)+ AngularVelocityPID::Ki*pid_error_sum;
    pid_error_sum += target_angular_vel - current_angular_vel;
    if(abs(pid_error_sum) > AngularVelocityPID::MAX_PID_ERROR_SUM) {
        pid_error_sum = (pid_error_sum > 0 ? AngularVelocityPID::MAX_PID_ERROR_SUM : -AngularVelocityPID::MAX_PID_ERROR_SUM);
    }
    if(RobotControllerParam::MAX_OMEGA < abs(target_angular_vel + pid_error)){
        return target_angular_vel > 0 ? RobotControllerParam::MAX_OMEGA : -RobotControllerParam::MAX_OMEGA;
    }
    return target_angular_vel + pid_error;
}

int Motor::calcDuty(float torque) {
    // shishikawaさんのmotor.jsより((R*tau)/Kt + Ke*omega)/Vbat
    // https://github.com/meganetaaan/M5Mouse/blob/master/mouse/motor.js
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
