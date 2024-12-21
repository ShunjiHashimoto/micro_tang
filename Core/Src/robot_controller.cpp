#include "robot_controller.hpp"

extern ModeManager mode_manager;
extern PhotoTransSensor photo_trans_sensor;
extern Motor motor_l;
extern Motor motor_r;
extern Log vel_log;

RobotController::RobotController(){
  this->is_running = true;
};

void RobotController::motorControl(float target_linear_vel, float target_angular_vel) {
    LinearVelocityPID::target_linear_vel = target_linear_vel;
    AngularVelocityPID::target_angular_vel = target_angular_vel;
    return;
}

void RobotController::allMotorStop() {
    LinearVelocityPID::target_linear_vel = 0.0;
    AngularVelocityPID::target_angular_vel = 0.0;
    motor_r.Stop();
    motor_l.Stop();
    this->is_running = false;
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
    this->allMotorStop();
    this->is_running = true;

    while (true) {
      unsigned long current_count = __HAL_TIM_GET_COUNTER(&htim9);
      float delta_time = calculateDeltaTime(current_count, prev_count, 10000); // タイマーの最大カウント値を10000と仮定
      total_time += delta_time;
      prev_count = current_count;
      float diff = target_distance - LinearVelocityPID::current_distance;
      AngularVelocityPID::w_pid_error_sum = 0.0;

      // 両壁あり
      int16_t sensor_diff = 0;
      if(photo_trans_sensor.getCurrentADC(2) > 100 && photo_trans_sensor.getCurrentADC(1) > 100){
          sensor_diff = photo_trans_sensor.getDiffADCBothWall();
          AngularVelocityPID::target_angular_vel = sensor_diff*ADCParam::SENSOR_GAIN;
          printf("both sensor diff %d\n\r", sensor_diff);
      }
      else if(photo_trans_sensor.getCurrentADC(2) < 100 && photo_trans_sensor.getCurrentADC(1) < 100) {
          AngularVelocityPID::target_angular_vel = 0.0;
          printf("others 1: %d, 2: %d, \n\r", photo_trans_sensor.getCurrentADC(1), photo_trans_sensor.getCurrentADC(2));
      }
      // 左壁無し、右壁制御
      else if(photo_trans_sensor.getCurrentADC(2) < 100){
          sensor_diff = photo_trans_sensor.getDiffADCRightOneWall();
          AngularVelocityPID::target_angular_vel = 2*sensor_diff*ADCParam::SENSOR_GAIN_R;
          printf("right sensor diff %d\n\r", sensor_diff);
      }
      // 右壁無し、左壁制御
      else if(photo_trans_sensor.getCurrentADC(1) < 100){
          sensor_diff = photo_trans_sensor.getDiffADCLeftOneWall();
          AngularVelocityPID::target_angular_vel = -sensor_diff*ADCParam::SENSOR_GAIN_L;
          printf("left sensor diff %d\n\r", sensor_diff);
      }
      if(diff < RobotControllerParam::MIN_DISTANCE_TO_RUN) break;
      // 加速区間
      if(LinearVelocityPID::current_distance < AL){
        LinearVelocityPID::target_linear_vel += accel*delta_time;
        // printf("accel, diff: %lf, target_vel: %lf, delta_time: %lf\n\r", diff, LinearVelocityPID::target_linear_vel, delta_time);
      }
      // 定速区間
      else if (LinearVelocityPID::current_distance >= AL && LinearVelocityPID::current_distance <= (AL+CL)) {
        LinearVelocityPID::target_linear_vel = v_max;
        // printf("const, diff: %lf, target_vel: %lf, delta_time: %lf\n\r", diff, v_max, delta_time);
      }
      // 減速区間
      else if (LinearVelocityPID::current_distance > (AL+CL) && LinearVelocityPID::current_distance <= (AL+CL+DL)) {
        LinearVelocityPID::target_linear_vel -= decel*delta_time;
        if(LinearVelocityPID::target_linear_vel < LinearVelocityPID::MIN_SPEED) {
          LinearVelocityPID::target_linear_vel = LinearVelocityPID::MIN_SPEED;
        }
        // printf("decel, diff: %lf, target_vel: %lf, delta_time: %lf\n\r", diff, LinearVelocityPID::target_linear_vel, delta_time);
      }
      printf("photo_diff: %d, adc_bat: %lf, calculated_linear_vel: %lf, calculated_angular_vel: %lf, motor_r.rotation_speed: %lf, motor_l.rotation_speed: %lf, motor_r.duty: %d, motor_l.duty: %d, target_angular_vel: %lf, current_angular_vel: %lf, current_deg: %d, diff: %lf\n\r", photo_trans_sensor.getDiffADCBothWall() + ADCParam::SENSOR_OFFSET, Battery::adc_bat, LinearVelocityPID::calculated_linear_vel, AngularVelocityPID::calculated_angular_vel, motor_r.rotation_speed, motor_l.rotation_speed, motor_r.duty, motor_l.duty, AngularVelocityPID::target_angular_vel, AngularVelocityPID::current_angular_vel, radToDeg(AngularVelocityPID::current_angle), diff);
      if(LinearVelocityPID::target_linear_vel > v_max) LinearVelocityPID::target_linear_vel = v_max;
      if(LinearVelocityPID::target_linear_vel < 0) LinearVelocityPID::target_linear_vel = 0.0;
    }
    printf("stop\n\r");
    this->allMotorStop();
    return;
}

void RobotController::turn_right(uint16_t target_deg) {
    AngularVelocityPID::target_angular_vel = degToRad(-150); // [rad/s]
    // LinearVelocityPID::target_linear_vel = 10; //[mm/s]
    float target_rad = degToRad(target_deg);
    float diff = 9999.0;
    while(diff > 0) {
      diff = target_rad - abs(AngularVelocityPID::current_angle);
      printf("adc_bat: %lf, calculated_linear_vel: %lf, calculated_angular_vel: %lf, motor_r.rotation_speed: %lf, motor_l.rotation_speed: %lf, motor_r.duty: %d, motor_l.duty: %d, target_deg: %d, target_angular_vel: %lf, current_angular_vel: %lf, current_deg: %d, diff: %lf\n\r", Battery::adc_bat, LinearVelocityPID::calculated_linear_vel, AngularVelocityPID::calculated_angular_vel, motor_r.rotation_speed, motor_l.rotation_speed, motor_r.duty, motor_l.duty, target_deg, AngularVelocityPID::target_angular_vel, AngularVelocityPID::current_angular_vel, radToDeg(AngularVelocityPID::current_angle), diff);
    }
    printf("Finished: %lf\n\r", diff);
    this->allMotorStop();
    return;
}

void RobotController::turn_left(uint16_t target_deg) {
    AngularVelocityPID::target_angular_vel = degToRad(150); // [rad/s]
    // LinearVelocityPID::target_linear_vel = 10; //[mm/s]
    float target_rad = degToRad(target_deg);
    float diff = 9999.0;
    while(diff > 0) {
      diff = target_rad - AngularVelocityPID::current_angle;
      printf("adc_bat: %lf, calculated_linear_vel: %lf, calculated_angular_vel: %lf, motor_r.rotation_speed: %lf, motor_l.rotation_speed: %lf, motor_r.duty: %d, motor_l.duty: %d, target_deg: %d, target_angular_vel: %lf, current_angular_vel: %lf, current_deg: %d, diff: %lf\n\r", Battery::adc_bat, LinearVelocityPID::calculated_linear_vel, AngularVelocityPID::calculated_angular_vel, motor_r.rotation_speed, motor_l.rotation_speed, motor_r.duty, motor_l.duty, target_deg, AngularVelocityPID::target_angular_vel, AngularVelocityPID::current_angular_vel, radToDeg(AngularVelocityPID::current_angle), diff);
    }
    printf("Finished: %lf\n\r", diff);
    this->allMotorStop();
    return;
}

// タイマーのカウント値を基にデルタ時間を計算する関数
float RobotController::calculateDeltaTime(unsigned long current_count, unsigned long& prev_count, unsigned long timer_max_count) {
    unsigned long delta_count;
    if (current_count >= prev_count) {
        delta_count = current_count - prev_count;
    } else {
        delta_count = (timer_max_count - prev_count) + current_count; // オーバーフロー対応
    }
    prev_count = current_count;
    return delta_count * 10e-6; // マイクロ秒 -> 秒
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
      // printf("tar_vel %lf cur_vel %lf, cur_angle %lf\n\r", AngularVelocityPID::target_angular_vel, this->getCurrentAngularVel(), AngularVelocityPID::current_angle);
      // printf("duty_r %d, duty_l %d\n\r", motor_r.duty, motor_l.duty);
      // printf("cur_LinearVelocityPIDvel %lf tar_vel %lf\n\r", LinearVelocityPID::current_linear_vel, LinearVelocityPID::target_linear_vel);
      // printf("current_distance: %lf angle: %lf\n\r", LinearVelocityPID::current_distance, AngularVelocityPID::current_angle);
      if(this-is_running) this->straight(180*2);
      // if(this->is_running) this->turn_left(360*5);
      // if(this->is_running) this->turn_right(360*5);
      this->is_running = false;

      // 目標速度のみ与える
      // float target_distance = 1000;
      // float diff = target_distance - LinearVelocityPID::current_distance;
      // if (diff < RobotControllerParam::MIN_DISTANCE_TO_RUN) {
      //   LinearVelocityPID::target_linear_vel = 0.0;
      //   motor_r.Stop();
      //   motor_l.Stop();
      //   return;
      // }
      for(int i=0; i<4; i++){
        auto adc_val = photo_trans_sensor.getCurrentADC(i);
        printf("adc_val[%d]: %d\n\r", i, adc_val);
      }
      // printf("Finished 1 loop\n\r");
      HAL_Delay(200);
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