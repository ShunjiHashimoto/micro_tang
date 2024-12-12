#include "robot_controller.hpp"

extern ModeManager mode_manager;
extern Motor motor_l;
extern Motor motor_r;
extern Log vel_log;

RobotController::RobotController(){
};

void RobotController::motorControl(float target_linear_vel_mm, float target_angular_vel) {
    LinearVelocityPID::target_linear_vel_mm = target_linear_vel_mm;
    AngularVelocityPID::target_angular_vel = target_angular_vel;
    return;
}

void RobotController::straight(float target_distance) { // [mm]
    float diff = target_distance - LinearVelocityPID::current_distance;
    if(diff < 10) return;
    // float v_0 = 0.0; // [mm/sec]
    LinearVelocityPID::current_distance = 0.0;
    LinearVelocityPID::current_linear_vel_mm = 0.0;
    float v_0 = LinearVelocityPID::current_linear_vel_mm; // [mm/sec]
    float v_max = RobotControllerParam::MAX_SPEED; // [mm/sec]
    float accel = RobotControllerParam::ACCEL; // [mm/sec^2]
    float decel = RobotControllerParam::DECEL; // [mm/sec^2]
    const float t_acc = (v_max - v_0)/accel; // 200/100 = 2[sec]
    float AL = 0.5*accel*t_acc*t_acc; // 0.5*100*4 = 200, 加速距離[mm], 2000mmの場合は,加速距離は200mm
    float CL = target_distance - AL*2; // 定速距離[mm], 1600mm
    if(CL <= 0) {
      CL = 0.0;
      AL = target_distance/2;
    }
    float DL = target_distance - AL - CL; // 減速距離[mm]
    printf("v_0: %lf, accel: %lf, t_acc: %lf, AL: %lf, CL: %lf, DL: %lf\n\r", v_0, accel, t_acc, AL, CL, DL);

    // 1カウントあたり20μs（TIM9）, 200msごとにオーバーフロー 
    unsigned long prev_count = 0; 
    float total_time = 0.0;
    LinearVelocityPID::target_linear_vel_mm = 0.0;
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
      // printf("cur_count: %ld, prev_count: %ld, delta_time: %lf\n\r", current_count, prev_count, delta_time);
      prev_count = current_count;
      float diff = target_distance - LinearVelocityPID::current_distance;
      if(diff < 10) break;
      // 加速区間, delta_time = 0.022480[sec], v=a*t=100[mm/sec^2]*0.022480[sec]=0.22480[mm/sec]
      if(LinearVelocityPID::current_distance < AL){
        LinearVelocityPID::target_linear_vel_mm += accel*delta_time;
        printf("AL, %lf, current_distance, %lf ,accel, diff, %lf, target_vel, %lf, cur_vel, %lf, calculated_vel, %lf, delta_time, %lf\n\r", AL , LinearVelocityPID::current_distance, diff, LinearVelocityPID::target_linear_vel_mm, LinearVelocityPID::current_linear_vel_mm, LinearVelocityPID::calculated_linear_vel_mm, delta_time);
      }
      // 定速区間
      else if (LinearVelocityPID::current_distance >= AL && LinearVelocityPID::current_distance <= (AL+CL)) {
        LinearVelocityPID::target_linear_vel_mm = v_max;
        printf("CL, %lf, current_distance: %lf ,const, diff: %lf, target_vel: %lf,  cur_vel, %lf, delta_time: %lf\n\r", CL, LinearVelocityPID::current_distance, diff, LinearVelocityPID::target_linear_vel_mm, LinearVelocityPID::current_linear_vel_mm, delta_time);
      }
      // 減速区間
      else if (LinearVelocityPID::current_distance > (AL+CL) && LinearVelocityPID::current_distance <= target_distance) {
        LinearVelocityPID::target_linear_vel_mm -= decel*delta_time;
        printf("current_distance: %lf ,decel, diff: %lf, target_vel: %lf,  cur_vel, %lf, delta_time: %lf\n\r", LinearVelocityPID::current_distance, diff, LinearVelocityPID::target_linear_vel_mm, LinearVelocityPID::current_linear_vel_mm, delta_time);
      }
      printf("timestamp, %lf, target_vel, %lf, cur_vel, %lf, distance, %lf\n\r", total_time, LinearVelocityPID::target_linear_vel_mm, LinearVelocityPID::current_linear_vel_mm, LinearVelocityPID::current_distance);
      if(LinearVelocityPID::target_linear_vel_mm > v_max) LinearVelocityPID::target_linear_vel_mm = v_max;
      if(LinearVelocityPID::target_linear_vel_mm < 0) LinearVelocityPID::target_linear_vel_mm = 0.0;
    }
    printf("stop\n\r");
    LinearVelocityPID::target_linear_vel_mm = 0.0;
    AngularVelocityPID::target_angular_vel = 0.0;
    motor_r.Stop();
    motor_l.Stop();
    return;
}

void RobotController::linearRun(float distance) {
    // 指定した距離を走行する
}

float RobotController::getCalculatedLinearVel() {
    return LinearVelocityPID::calculated_linear_vel_mm;
}

float RobotController::getCalculatedAngularVel() {
    return AngularVelocityPID::calculated_angular_vel;
}

float RobotController::getCurrentLinearVel() {
    return LinearVelocityPID::current_linear_vel_mm;
}

float RobotController::getCurrentAngularVel() {
    return AngularVelocityPID::current_angular_vel;
}

void RobotController::mainControl(){
    Mode::ModeType current_mode = mode_manager.getCurrentMode();

    if(current_mode == Mode::ModeType::RUN) {
      // printf("cur_LinearVelocityPIDvel %lf tar_vel %lf\n\r", LinearVelocityPID::current_linear_vel_mm, LinearVelocityPID::target_linear_vel_mm);
      // printf("tar_vel %lf cur_vel %lf\n\r", AngularVelocityPID::target_angular_vel, this->getCurrentAngularVel());
      // printf("duty_r %d, duty_l %d\n\r", motor_r.duty, motor_l.duty);
      // printf("current_distance: %lf angle: %lf\n\r", LinearVelocityPID::current_distance, AngularVelocityPID::current_angle);
      this->straight(2000);
      // モード更新し、終了
      // HAL_Delay(200);
      // LinearVelocityPID::target_linear_vel_mm = 200;
      // if(LinearVelocityPID::current_distance > 1000) {
      //   LinearVelocityPID::target_linear_vel_mm = 0.0;
      //   AngularVelocityPID::target_angular_vel = 0.0;
      //   motor_r.Stop();
      //   motor_l.Stop();
      //   current_mode = Mode::ModeType::WAIT;
      // }
      return;
    }

    else if(current_mode == Mode::ModeType::LOG) {
      // TODO: logは繰り返し出す必要はない
      vel_log.printLog();
      HAL_Delay(180);
      current_mode = mode_manager.getCurrentMode();
      return;
    }
    
}