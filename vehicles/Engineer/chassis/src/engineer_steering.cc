 #include "engineer_steering.h"

template <typename T>
bool equal(T value, T number) {
  const T diff = value-number;
  return abs(diff) < 0.5 ? true : false;
}
    
namespace control{

    EngineerSteeringChassis::EngineerSteeringChassis(engineer_steering_chassis_t* _chassis) {
  if (_chassis == nullptr) {
    RM_ASSERT_TRUE(false, "Chassis pointer is null\r\n");
  }

  if (_chassis->fl_steer_motor == nullptr || _chassis->fr_steer_motor == nullptr ||
      _chassis->bl_steer_motor == nullptr || _chassis->br_steer_motor == nullptr ||
      _chassis->fl_wheel_motor == nullptr || _chassis->fr_wheel_motor == nullptr ||
      _chassis->bl_wheel_motor == nullptr || _chassis->br_wheel_motor == nullptr) {
    RM_ASSERT_TRUE(false, "At least one motor pointer is null\r\n");
  }

  // Init Steer Motors
  fl_steer_motor = _chassis->fl_steer_motor;
  fr_steer_motor = _chassis->fr_steer_motor;
  bl_steer_motor = _chassis->bl_steer_motor;
  br_steer_motor = _chassis->br_steer_motor;

  fl_steer_motor_raw = _chassis->fl_steer_motor_raw;
  fr_steer_motor_raw = _chassis->fr_steer_motor_raw;
  bl_steer_motor_raw = _chassis->bl_steer_motor_raw;
  br_steer_motor_raw = _chassis->br_steer_motor_raw;

  // Init Wheel Motors
  fl_wheel_motor = _chassis->fl_wheel_motor;
  fr_wheel_motor = _chassis->fr_wheel_motor;
  bl_wheel_motor = _chassis->bl_wheel_motor;
  br_wheel_motor = _chassis->br_wheel_motor;

  fl_steer_motor->CalcOutput();
  fr_steer_motor->CalcOutput();
  bl_steer_motor->CalcOutput();
  br_steer_motor->CalcOutput();

  fl_wheel_motor->SetOutput(0.0);
  fr_wheel_motor->SetOutput(0.0);
  bl_wheel_motor->SetOutput(0.0);
  br_wheel_motor->SetOutput(0.0);
  // safety lock ends

  // Init private variables starts
  vx = 0.0;
  vy = 0.0;
  vw = 0.0;

  v_fl_ = 0.0;
  v_fr_ = 0.0;
  v_bl_ = 0.0;
  v_br_ = 0.0;

  in_position = false;
  in_position_detector = new BoolEdgeDetector(false);


  theta_fl_ = wrap<double>(0,0,2*PI);
  theta_fr_ = wrap<double>(0,0,2*PI);
  theta_bl_ = wrap<double>(0,0,2*PI);
  theta_br_ = wrap<double>(0,0,2*PI);


  // Init private variables ends
//   SteerThetaReset();
  // just for wheel speed pid (not for steer motor)

  float* pid_params = new float[3]{200, 0, 0};
  float motor_max_iout = 20000;
  float motor_max_out = 20000;
  for (int i = 0; i < MOTOR_NUM; i++) {
    pids[i].Reinit(pid_params, motor_max_iout, motor_max_out);
  }

  power_limit = new PowerLimit(MOTOR_NUM);
}

EngineerSteeringChassis::~EngineerSteeringChassis() {
  fl_steer_motor = nullptr;
  fr_steer_motor = nullptr;
  bl_steer_motor = nullptr;
  br_steer_motor = nullptr;
  fl_wheel_motor = nullptr;
  fr_wheel_motor = nullptr;
  bl_wheel_motor = nullptr;
  br_wheel_motor = nullptr;
}


  // front -> positive, back -> negative
  void EngineerSteeringChassis::SetXSpeed(float _vx){ vx = _vx; }

  // left -> positive, right -> negative
  void EngineerSteeringChassis::SetYSpeed(float _vy){ vy = _vy; }

  // counterclockwise -> positive
  void EngineerSteeringChassis::SetWSpeed(float _vw){ vw = _vw; }

  int loop_count = 0;

  void EngineerSteeringChassis::Update(float _power_limit, float _chassis_power, float _chassis_power_buffer){
    // Update Wheels
    float PID_output[MOTOR_NUM];
    float output[MOTOR_NUM];

    // compute PID output
    


    PID_output[0] = pids[0].ComputeOutput(fl_wheel_motor->GetOmegaDelta(v_fl_ *M3508P19_RATIO / 0.06)); // m/s to rad/s
    PID_output[1] = pids[1].ComputeOutput(fr_wheel_motor->GetOmegaDelta(v_fr_ *M3508P19_RATIO / 0.06));
    PID_output[2] = pids[2].ComputeOutput(bl_wheel_motor->GetOmegaDelta(v_bl_ *M3508P19_RATIO / 0.06));
    PID_output[3] = pids[3].ComputeOutput(br_wheel_motor->GetOmegaDelta(v_br_ *M3508P19_RATIO / 0.06));

    // compute power limit
    power_limit_info.power_limit = _power_limit;
    power_limit_info.WARNING_power = _power_limit * 0.9;
    power_limit_info.WARNING_power_buff = 50;
    power_limit_info.buffer_total_current_limit = 3500 * MOTOR_NUM;
    power_limit_info.power_total_current_limit = 5000 * MOTOR_NUM / 80.0 * _power_limit;
    power_limit->Output(true, power_limit_info, _chassis_power, _chassis_power_buffer, PID_output,
                        output);

    // set final output
    fl_wheel_motor->SetOutput(PID_output[0]);
    fr_wheel_motor->SetOutput(PID_output[1]);
    bl_wheel_motor->SetOutput(PID_output[2]);
    br_wheel_motor->SetOutput(PID_output[3]);

    // if(loop_count == 100){
    //   set_cursor(0, 0);
    //   clear_screen();
    //   print("fl_wheel target %5.2f error %5.2f m/s\r\n", v_fl_, v_fl_ - fl_wheel_motor->GetOmega()/M3508P19_RATIO*0.06); //0.06 = wheel radius
    //   print("pid out %5.2f \r\n", PID_output[0]);
    //   print("fr_wheel target %5.2f error %5.2f m/s\r\n", v_fr_, v_fr_ - fr_wheel_motor->GetOmega()/M3508P19_RATIO*0.06);
    //   print("pid out %5.2f \r\n", PID_output[1]);
    //   print("bl_wheel target %5.2f error %5.2f m/s\r\n", v_bl_, v_bl_ - bl_wheel_motor->GetOmega()/M3508P19_RATIO*0.06);
    //   print("pid out %5.2f \r\n", PID_output[2]);
    //   print("br_wheel target %5.2f error %5.2f m/s\r\n", v_br_, v_br_ - br_wheel_motor->GetOmega()/M3508P19_RATIO*0.06);
    //   print("pid out %5.2f \r\n", PID_output[3]);

    //   loop_count = 0;
    // }
    // loop_count ++;
  }

  void EngineerSteeringChassis::SetSpeed(const float _vx, const float _vy, const float _vw){
    vx = _vx;
    vy = _vy;
    vw = _vw;
  }

  void EngineerSteeringChassis::SetWheelSpeed(float _v_fl, float _v_fr, float _v_bl, float _v_br){
    v_fl_ = _v_fl;
    v_fr_ = _v_fr;
    v_bl_ = _v_bl;
    v_br_ = _v_br;
  } 

  bool EngineerSteeringChassis::Calibrate(){
    fl_steer_motor->SetTarget(0,true);
    fr_steer_motor->SetTarget(0,true);
    bl_steer_motor->SetTarget(0,true);
    br_steer_motor->SetTarget(0,true);

    bool calibrate_done = SteerInPosition();
    if(calibrate_done){
      in_position = true;
      in_position_detector->input(in_position);
    }

    return calibrate_done;

  }

  void EngineerSteeringChassis::SteerCalcOutput(){
    fl_steer_motor->CalcOutput();
    fr_steer_motor->CalcOutput();
    bl_steer_motor->CalcOutput();
    br_steer_motor->CalcOutput();
  }

  void EngineerSteeringChassis::SteerUpdateTarget(){
    if (vx != 0 || vy != 0 || vw != 0) {
    
    double _theta_fl = atan2(vy + vw * cos(PI / 4), vx - vw * sin(PI / 4));
    double _theta_fr = atan2(vy + vw * cos(PI / 4), vx + vw * sin(PI / 4));
    double _theta_bl = atan2(vy - vw * cos(PI / 4), vx - vw * sin(PI / 4));
    double _theta_br = atan2(vy - vw * cos(PI / 4), vx + vw * sin(PI / 4));

    theta_fl_ = _theta_fl;
    theta_fr_ = _theta_fr;
    theta_bl_ = _theta_bl;
    theta_br_ = _theta_br;


    double _theta_fl_alt = wrap<double>(_theta_fl + PI, -PI, PI);
    double _theta_fr_alt = wrap<double>(_theta_fr + PI, -PI, PI);
    double _theta_bl_alt = wrap<double>(_theta_bl + PI, -PI, PI);
    double _theta_br_alt = wrap<double>(_theta_br + PI, -PI, PI);

    bool ret_fl = false;
    bool ret_fr = false;
    bool ret_bl = false;
    bool ret_br = false;

    wheel_dir_fl_ = 1.0;
    wheel_dir_fr_ = 1.0;
    wheel_dir_bl_ = 1.0;
    wheel_dir_br_ = 1.0;

    // Compute 2 position proposals, theta and theta + PI.
    // Turn to which one is closer

    float _current_theta_bl = bl_steer_motor->GetTheta();
    float _current_theta_br = br_steer_motor->GetTheta();
    float _current_theta_fl = fl_steer_motor->GetTheta();
    float _current_theta_fr = fr_steer_motor->GetTheta();


    // Back Left Motor
    if (abs(wrap<double>(_theta_bl - _current_theta_bl, -PI, PI)) <
        abs(wrap<double>(_theta_bl_alt - _current_theta_bl, -PI, PI))) {
      wheel_dir_bl_ = 1.0;
      steer_target_bl_  = wrap<double>(_theta_bl, -PI, PI);
    } else {
      wheel_dir_bl_ = -1.0;
      steer_target_bl_  = wrap<double>(_theta_bl_alt, -PI, PI);
    }
    ret_bl = bl_steer_motor->SetTarget(steer_target_bl_) == 0 ? false : true;

    //Back Right Motor
    if (abs(wrap<double>(_theta_br - _current_theta_br, -PI, PI)) <
        abs(wrap<double>(_theta_br_alt - _current_theta_br, -PI, PI))) {
      wheel_dir_br_ = 1.0;
      steer_target_br_  = wrap<double>(_theta_br, -PI, PI);
    } else {
      wheel_dir_br_ = -1.0;
      steer_target_br_  = wrap<double>(_theta_br_alt, -PI, PI);
    }
    ret_br = br_steer_motor->SetTarget(steer_target_br_) == 0 ? false : true;

    // Front Right Motor
    if (abs(wrap<double>(_theta_fr - _current_theta_fr , -PI, PI)) <
        abs(wrap<double>(_theta_fr_alt - _current_theta_fr , -PI, PI))) {
      wheel_dir_fr_ = 1.0;
      steer_target_fr_  = wrap<double>(_theta_fr, -PI, PI);
    } else {
      wheel_dir_fr_ = -1.0;
      steer_target_fr_  = wrap<double>(_theta_fr_alt, -PI, PI);
    }
    ret_fr = fr_steer_motor->SetTarget(steer_target_fr_) == 0 ? false : true;


    if (abs(wrap<double>(_theta_fl - _current_theta_fl, -PI, PI)) <
        abs(wrap<double>(_theta_fl_alt - _current_theta_fl, -PI, PI))) {
      wheel_dir_fl_ = 1.0;
      steer_target_fl_ = wrap<double>(_theta_fl, -PI, PI);
    } else {
      wheel_dir_fl_ = -1.0;
      steer_target_fl_ = wrap<double>(_theta_fl_alt, -PI, PI);
    }
    ret_fl = fl_steer_motor->SetTarget(steer_target_fl_) == 0 ? false : true;


    // if(fl_steer_motor->inPosition()){
    //   theta_fl_ = fl_steer_motor->GetTheta();
    // }
    // if(fr_steer_motor->inPosition()){
    //   theta_fr_ = fr_steer_motor->GetTheta();
    // }
    // if(bl_steer_motor->inPosition()){
    //   theta_bl_ = bl_steer_motor->GetTheta();
    // }
    // if(br_steer_motor->inPosition()){
    //   theta_br_ = br_steer_motor->GetTheta();
    // }

    //in_position logic might not working here.
    in_position = ret_fl && ret_fr && ret_bl && ret_br;
    in_position_detector->input(in_position);

  }
}

  void EngineerSteeringChassis::WheelUpdateSpeed(){
     // Stay at current position when no command is given
    if (vx == 0 && vy == 0 && vw == 0) {
        SetWheelSpeed(0,0,0,0);
    // TODO: Potential bug, wheel will rotate during turning.
    // } else if (in_position_detector->posEdge()) {
    } else {


        // Wheels move only when all SteeringMotors are in position
        v_fl_ = wheel_dir_fl_ * sqrt(pow(vy + vw * cos(PI / 4), 2.0) + pow(vx - vw * sin(PI / 4), 2.0));
        v_fr_ = wheel_dir_fr_ * sqrt(pow(vy + vw * cos(PI / 4), 2.0) + pow(vx + vw * sin(PI / 4), 2.0));
        v_bl_ = wheel_dir_bl_ * sqrt(pow(vy - vw * cos(PI / 4), 2.0) + pow(vx - vw * sin(PI / 4), 2.0));
        v_br_ = wheel_dir_br_ * sqrt(pow(vy - vw * cos(PI / 4), 2.0) + pow(vx + vw * sin(PI / 4), 2.0));

        // check if the speed of each wheel is greater than limit of m3508
        // if yes, scale down the speed of each wheel

        // Another option is ensure translation first, then rotation, but need to discuss which is better
        const float M3508_MAX_SPEED = M3508_MAX_OMEGA * WHEEL_RADIUS;

        if(v_fl_ > M3508_MAX_SPEED || v_fr_ > M3508_MAX_SPEED || v_bl_ > M3508_MAX_SPEED || v_br_ > M3508_MAX_SPEED){
            float max_speed = max(v_fl_, max(v_fr_, max(v_bl_, v_br_)));
            v_fl_ = v_fl_ / max_speed * M3508_MAX_SPEED;
            v_fr_ = v_fr_ / max_speed * M3508_MAX_SPEED;
            v_bl_ = v_bl_ / max_speed * M3508_MAX_SPEED;
            v_br_ = v_br_ / max_speed * M3508_MAX_SPEED;
        }
    }
  }

  void EngineerSteeringChassis::SteerSetMaxSpeed(const float max_speed){
    fl_steer_motor->SetMaxSpeed(max_speed);
    fr_steer_motor->SetMaxSpeed(max_speed);
    bl_steer_motor->SetMaxSpeed(max_speed);
    br_steer_motor->SetMaxSpeed(max_speed);
  }

  void EngineerSteeringChassis::PrintData(){
    clear_screen();
    set_cursor(0, 0);

    //print steer angle from chassis's reference
    if(true){
      print("fl_steer_motor: \r\n");
      print("target: %f, current: %f \r\n", steer_target_fl_, fl_steer_motor->GetTheta());
      osDelay(10);
      print("fr_steer_motor: \r\n");
      print("target: %f, current: %f \r\n", steer_target_fr_, fr_steer_motor->GetTheta());
      osDelay(10);
      print("bl_steer_motor: \r\n");
      print("target: %f, current: %f \r\n", steer_target_bl_, bl_steer_motor->GetTheta());
      osDelay(10);
      print("br_steer_motor: \r\n");
      print("target: %f, current: %f \r\n", steer_target_br_, br_steer_motor->GetTheta());
 
    }

    if(false){
      print("fl_wheel_motor: \r\n");
      print("target: %f, current: %f \r\n", v_fl_, fl_wheel_motor->GetTheta()/M3508P19_RATIO*0.06);
      osDelay(10);
      print("fr_wheel_motor: \r\n");
      print("target: %f, current: %f \r\n", v_fr_, fr_wheel_motor->GetTheta()/M3508P19_RATIO*0.06);
      osDelay(10);
      print("bl_wheel_motor: \r\n");
      print("target: %f, current: %f \r\n", v_bl_, bl_wheel_motor->GetTheta()/M3508P19_RATIO*0.06);
      osDelay(10);
      print("br_wheel_motor: \r\n");
      print("target: %f, current: %f \r\n", v_br_, br_wheel_motor->GetTheta()/M3508P19_RATIO*0.06);
      
    }
    
    //print motor data
    if(false){
      print("fl_steer_motor: \r\n");
      fl_steer_motor->PrintData();
      osDelay(10);
      print("fr_steer_motor: \r\n");
      fr_steer_motor->PrintData();
      osDelay(10);
      print("bl_steer_motor: \r\n");
      bl_steer_motor->PrintData();
      osDelay(10);
      print("br_steer_motor: \r\n");
      br_steer_motor->PrintData();
    }
  }

  bool EngineerSteeringChassis::SteerInPosition(){
    bool result = fl_steer_motor->inPosition() && 
            fr_steer_motor->inPosition() &&
            bl_steer_motor->inPosition() &&
            br_steer_motor->inPosition();
    in_position = result;
    in_position_detector->input(in_position);
    return in_position;
  }





}