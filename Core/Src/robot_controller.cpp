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

void RobotController::straight(float target_distance) { // [mm]
   float diff = target_distance - LinearVelocityPID::current_distance;
    // float v_0 = 0.0; // [mm/sec]
    if(diff < RobotControllerParam::MIN_DISTANCE_TO_RUN) return;
    LinearVelocityPID::current_distance = 0.0;
    LinearVelocityPID::current_linear_vel = 0.0;

    float v_0 = LinearVelocityPID::current_linear_vel; // [mm/sec]
    float v_max = RobotControllerParam::MAX_SPEED; // [mm/sec]
    float accel = LinearVelocityPID::target_a; // [mm/sec^2]
    float decel = LinearVelocityPID::target_a; // [mm/sec^2]
    float t_acc = (v_max - v_0)/accel;
    float AL = 0.5*accel*t_acc*t_acc; // 加速距離[mm]
    float CL = target_distance - AL*2; // 定速距離[mm]
    if(CL <= 0) {
      CL = 0.0;
      AL = target_distance/2;
    }
    float DL = target_distance - AL - CL; // 減速距離[mm]

    // 1カウントあたり20μs（TIM9）, 200msごとにオーバーフロー 
    unsigned long prev_count = 0; 
    float total_time = 0.0;
    LinearVelocityPID::target_linear_vel = 0.0;
    AngularVelocityPID::target_angular_vel = 0.0;
    motor_r.Stop();
    motor_l.Stop();

    while (true) {
      unsigned long current_count= __HAL_TIM_GET_COUNTER(&htim9);
      unsigned long delta_count;
      if (current_count >= prev_count) {
        delta_count = current_count - prev_count;
      } else {
        delta_count = (10000 - prev_count) + current_count; // オーバーフロー対応、10000でオーバーフロー
      }
      float delta_time = delta_count * 10e-6; // マイクロ秒 → 秒, 0.0125[sec]程度
      total_time += delta_time;
      prev_count = current_count;
      // printf("cur_time: %lf, last_update_time: %lf, delta_time: %lf\n\r", current_time, last_update_time, delta_time);
      float diff = target_distance - LinearVelocityPID::current_distance;
      if(diff < RobotControllerParam::MIN_DISTANCE_TO_RUN) break;
      // 加速区間
      if(LinearVelocityPID::current_distance < AL){
        LinearVelocityPID::target_linear_vel += accel*delta_time;
        printf("accel, diff: %lf, target_vel: %lf, delta_time: %lf\n\r", diff, LinearVelocityPID::target_linear_vel, delta_time);
      }
      // 定速区間
      else if (LinearVelocityPID::current_distance >= AL && LinearVelocityPID::current_distance <= (AL+CL)) {
        LinearVelocityPID::target_linear_vel = v_max;
        printf("const, diff: %lf, target_vel: %lf, delta_time: %lf\n\r", diff, v_max, delta_time);
      }
      // 減速区間
      else if (LinearVelocityPID::current_distance > (AL+CL) && LinearVelocityPID::current_distance <= (AL+CL+DL)) {
        LinearVelocityPID::target_linear_vel -= decel*delta_time;
        printf("decel, diff: %lf, target_vel: %lf, delta_time: %lf\n\r", diff, LinearVelocityPID::target_linear_vel, delta_time);
      }
      if(LinearVelocityPID::target_linear_vel > v_max) LinearVelocityPID::target_linear_vel = v_max;
      if(LinearVelocityPID::target_linear_vel < 0) LinearVelocityPID::target_linear_vel = 0.0;
    }
    printf("stop\n\r");
    LinearVelocityPID::target_linear_vel = 0.0;
    AngularVelocityPID::target_angular_vel = 0.0;
    motor_r.Stop();
    motor_l.Stop();
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
      // printf("tar_vel %lf cur_vel %lf\n\r", AngularVelocityPID::target_angular_vel, this->getCurrentAngularVel());
      // printf("duty_r %d, duty_l %d\n\r", motor_r.duty, motor_l.duty);
      printf("cur_LinearVelocityPIDvel %lf tar_vel %lf\n\r", LinearVelocityPID::current_linear_vel, LinearVelocityPID::target_linear_vel);
      printf("current_distance: %lf angle: %lf\n\r", LinearVelocityPID::current_distance, AngularVelocityPID::current_angle);
      this->straight(540);
      // タイヤ系を調整する必要がある

      // 目標速度のみ与える
      // float target_distance = 1000;
      // float diff = target_distance - LinearVelocityPID::current_distance;
      // if (diff < RobotControllerParam::MIN_DISTANCE_TO_RUN) {
      //   LinearVelocityPID::target_linear_vel = 0.0;
      //   motor_r.Stop();
      //   motor_l.Stop();
      //   return;
      // }
      // モード更新し、終了
      current_mode = mode_manager.getCurrentMode();
      return;
    }

    else if(current_mode == Mode::ModeType::LOG) {
      // TODO: logは繰り返し出す必要はない
      vel_log.printLog();
      HAL_Delay(2000);
      return;
    }
    
}