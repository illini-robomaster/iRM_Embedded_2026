/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2024 RoboMaster.                                          *
 *  Illini RoboMaster @ University of Illinois at Urbana-Champaign          *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                          *
 ****************************************************************************/


// #define INV_KINEMATICS

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "sbus.h"
#include "main.h"
#include "motor.h"
#include "utils.h"
#include "bsp_can_bridge.h"
#include "moving_average.h"

#include "arm.h"
#include "arm_config.h"
#include "encoder.h"
#include "bsp_relay.h"

#include "A1/A1_motor_drive.h"
#include "trapezoid_profile.h"
#include "geometry/geometry.h"
#include <vector>
#include <referee_task.h>

// static bsp::CAN* can1 = nullptr;
static control::BRTEncoder* encoder0= nullptr;
static control::BRTEncoder* encoder1= nullptr;
static bsp::Relay* pump = nullptr;
static bsp::GPIO* power_5v = nullptr;


// Motor 4310
static control::Motor4310* forearm_rotate_motor_4 = nullptr;
static control::Motor4310* wrist_rotate_motor_5 = nullptr;
static control::Motor4310* hand_rotate_motor_6 = nullptr;

static const int ARM_TASK_DELAY = 3;

static float A1_id2_offset; // encoder1 + encoder0 - A1_id2 start angle
static float A1_id1_offset; // encoder0 - A1_id1 start angle
static float A1_id0_offset; // -A1_id0 start angle

static const float encoder1_val_when_arm_upright = 1.8776;
static const float encoder0_val_when_arm_upright = -1.6;

static bool killed = false;

static joint_state_t last_angular_velocities = {0,0,0,0,0,0,0};
static joint_state_t last_targets = {0,0,0,0,0,0,0};
static joint_state_t target_mechanical_angle = {0,0,0,0,0,0,0};
static joint_state_t current_mechanical_angle = {0,0,0,0,0,0,0};

static joint_state_t MECHANICAL_MAX_POS = {0, PI, 0.3, 1.0, PI, 2*PI/3, 2*PI/3};
static joint_state_t MECHANICAL_MIN_POS = {0, -PI, -0.7, -1.627, -PI, -2*PI/3, -2*PI/3};

static joint_state_t MAX_SPEED_RADIANS = {0, 1.0, 0.5, 0.5, 10.0, 10.0, 10.0};

static float base_rotate_offset = 0.0;


#ifdef INV_KINEMATICS
  static Vector3d last_target_position(0.46,0.05,0.36); // 3+3 joint
  // static Vector3d last_position(0.46,0.05,0.36); // 6 joint

#endif
const float a = 0.05;
const float b = 0.36; // big arm length
const float c = 0.30; // forearm length
const float wrist_length = 0.16;

extern bsp::CanBridge* send;


/*
zero position:
(elbow) -----
        |     
        |      
        |
      -----
      |   | (base)
      -----
*/

/**
 * forearm_rotate_motor     RX=0x08 TX=0x07
 * wrist_rotate_motor       RX=0x04 TX=0x03
 * hand_rotete_motor        RX=0x06 TX=0x05
 * 
 * elbow_rotate_motor      ID=2
 * upper_arm_motor          ID=1
 * base_motor               ID=0
*/

// entire arm

void init_arm_A1() {
  bsp::SetHighresClockTimer(&htim5);
  encoder1 = new control::BRTEncoder(can1,0x01, false);
  encoder0 = new control::BRTEncoder(can1,0x0A, true);
  pump = new bsp::Relay(GPIOC,GPIO_PIN_6);
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_DeInit(GPIOC,GPIO_PIN_6);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  pump->Off();
  power_5v = new bsp::GPIO(GPIOC, GPIO_PIN_14);
  power_5v->High();


  // motor 4310
  forearm_rotate_motor_4 = new control::Motor4310(can1, 0x08, 0x07, control::MIT);
  wrist_rotate_motor_5 = new control::Motor4310(can1, 0x04, 0x03, control::MIT);
  hand_rotate_motor_6 = new control::Motor4310(can1, 0x06, 0x05, control::MIT);

  /* rx_id = Master id
   * tx_id = CAN id
   * see example/m4310_mit.cc
   */

}

// CAUTION: these values needs to be updated if encoder/J4 4310 motor/elbow lever is reinstalled, especially encoder1
/** A legal arm position satisfies all of the following constraints:
 * encoder1 (elbow pitch) 2.9 to 3.9      -2.6875 -1.2272
 * encoder0 (base pitch) -2.1 to -3.5     1.2701 2.7612
 * encoder1+encoder0 0.1 to 1.8           -1.3929 1.0922
 * @return return false if joint targets are not possible to reach mechanically. 
 */

// -0.869177, -1.255376
bool isJointTargetsLegal(joint_state_t &mechanical_angle) {
  // base pitch joint
  if (mechanical_angle.base_pitch_rotate_2 < -2.3 -encoder0_val_when_arm_upright){
    return false;
  }

  if (mechanical_angle.base_pitch_rotate_2 > -1.3-encoder0_val_when_arm_upright){
    return false;
  }


  // elbow pitch joint
  if (mechanical_angle.forearm_pitch_3 < -1.35){
    return false;
  } 
  
  if (mechanical_angle.forearm_pitch_3 > 1.0) {
    return false;
  }

  // not possible to determine which one to 
  
  // angle between two carbon tubes
  if (mechanical_angle.forearm_pitch_3-mechanical_angle.base_pitch_rotate_2 > 2.65 - encoder1_val_when_arm_upright|| mechanical_angle.forearm_pitch_3-mechanical_angle.base_pitch_rotate_2 < 1.27 - encoder1_val_when_arm_upright ) {
    return false;
  }

  if (mechanical_angle.wrist_5 < MECHANICAL_MIN_POS.wrist_5 || mechanical_angle.wrist_5 > MECHANICAL_MAX_POS.wrist_5) {
    return false;
  }


  return true;
}

void waitUntil(bool (*function)()){
  while(!function()){
    osDelay(100);
  }
  return;
}

void checkAllMotorsConnected(){
  while(true){
    // because A1 will only return motor states if a command is sent to it, so we send a stop command
    modify_stop_cmd(&MotorA1_send, 0);
    unitreeA1_rxtx(huart1);
    modify_stop_cmd(&MotorA1_send, 1);
    unitreeA1_rxtx(huart1);
    modify_stop_cmd(&MotorA1_send, 2);
    unitreeA1_rxtx(huart1);


    bool all_connected = true;

    // because if A1 send back data, temperature will very unlikely be 0
    if(MotorA1_recv_id00.Temp == 0){
      print("A1 0 not connected\r\n");
      all_connected = false;
    }
    if(MotorA1_recv_id01.Temp == 0){
      print("A1 1 not connected\r\n");
      all_connected = false;
    }
    if(MotorA1_recv_id02.Temp == 0){
      print("A1 2 not connected\r\n");
      all_connected = false;
    }

    if(!forearm_rotate_motor_4->connection_flag_){
      print("forearm motor not connected\r\n");
      all_connected = false;
    }

    if(!wrist_rotate_motor_5->connection_flag_){
      print("wrist motor not connected\r\n");
      all_connected = false;
    }

    if(!hand_rotate_motor_6->connection_flag_){
      print("hand motor not connected\r\n");
      all_connected = false;
    }

    if(all_connected){
      print("all motors connected\r\n");
      return;
    }

    osDelay(100);
  }
}



joint_state_t inverse_kinematics(Vector3d position, Rotation3d orientation, joint_state_t joint_angles){
  
  float l = sqrt(position._x*position._x + position._y*position._y - a*a);
  float alpha1 = atan(l/a);
  float theta1 = atan2(position._y, position._x) + alpha1 - PI/2;

  float d = sqrt(l*l+position._z*position._z);
  float alpha2 = atan2(position._z, l);
  float beta1 = acos((b*b+d*d-c*c)/(2*b*d));
  float beta2 = acos((b*b+c*c-d*d)/(2*b*c));
  float theta2 = PI/2 - alpha2 - beta1;
  float theta3 = PI - alpha2 - beta1 - beta2;

  if(isnan(theta1)){
    theta1 = 0;
  }

  if(isnan(theta2)){
    theta2 = 0;
  }

  if(isnan(theta3)){
    theta3 = 0;
  }

     
  Rotation3d j3_rotation(0,1,0, Angle2d(joint_angles.forearm_pitch_3));
  Rotation3d j1_rotation(0,0,1, Angle2d(joint_angles.base_yaw_rotate_1));

  Rotation3d j1j2j3_rotation = j1_rotation*j3_rotation;
  // UNUSED(j1j2j3_rotation);
  // std::cout << "j3j2j1 rotation" << j1j2j3_rotation.getRoll() << " " << j1j2j3_rotation.getPitch() << " " << j1j2j3_rotation.getYaw() << std::endl;

  Rotation3d target_rotation(0,0,0);
  Rotation3d wrist_to_forearm_rotation = j1j2j3_rotation.inverse() * orientation;
  // Rotation3d wrist_to_forearm_rotation = orientation;

  Vector3d wrist_facing = Vector3d(1,0,0).rotateBy(wrist_to_forearm_rotation);
  // assumes forearm is pointing forward (camera is on forearm)
  // TODO: calculate angle relative to field instead of relative to forewarm
  // DEBUG: Roll axis chaneg target make position change as well.
  // 1. calculate the 3d rotation of the forearm (rotation cause by j3j2j1)
  // 2. calculate the 3d rotation of the wrist (j4j5j6) relative to the 3d rotation of the forearm (j3j2j1)


  float theta4 = atan2(wrist_facing._z, wrist_facing._y); // the angle of the wrist pointing direction projected in camera plane
  float theta5 = wrist_facing.angleBetween(Vector3d(1,0,0)).getRadians();  // angle between pointing forward and desired rotation as a vector
  
  // optimize 4310 angles
  // J4 angle should not turn over 90 degrees, and should reverse theta5
  Angle2d j4_delta(theta4-joint_angles.forearm_roll_4);
  if(abs(j4_delta.getRadiansNegPItoPI())>M_PI/2){
      theta4 = Angle2d(theta4-M_PI).getRadians();
      theta5 *= -1.0;
  }

  Rotation3d j4_rotation(1,0,0, Angle2d(theta4)); // axis-angle representation of a 3d rotation
  Rotation3d j5_rotation(0,0,1, Angle2d(theta5));

  // calculate wrist rotation only due to j4 and j5
  Rotation3d j4j5_rotation = j4_rotation * j5_rotation;
  
  // angle between desired rotation and rotation achieved by J4 and J5 only, which is j6 angle
  float theta6 = j4j5_rotation.angleBetween(wrist_to_forearm_rotation).getRadians(); 

  // however, there two direction of j6, so we need to do forward kinematics to see if it's this theta6 or its opposite.
  Rotation3d j6_rotation(1,0,0, Angle2d(theta6));
  Rotation3d j4j5j6_rotation = j4_rotation*j5_rotation*j6_rotation; // rotation by joint 6 then joint 5 then joint 4

  // std::cout << "j4j5j6 rotation" << j4j5j6_rotation.getRoll() << " " << j4j5j6_rotation.getPitch() << " " << j4j5j6_rotation.getYaw() << std::endl;
  // compare j4j5j6_rotation to wrist_to_forearm, if difference is big, then reverse j6 angle;
  float diff = j4j5j6_rotation.minus(wrist_to_forearm_rotation).getAxisAngle().angle.getRadians();
  // std::cout << "diff" << diff << std::endl;

  if(abs(diff) > 0.01){
      theta6 *= -1;
      // theta6 -= diff;
  }

  j6_rotation = Rotation3d(1,0,0, Angle2d(theta6));

  // rotation by joint 6 then joint 5 then joint 4
  j4j5j6_rotation = j4_rotation*j5_rotation*j6_rotation; 
  Vector3d j4j5j6_facing = Vector3d(1,0,0).rotateBy(j4j5j6_rotation);
  // std::cout << "j4j5j6 facing" << j4j5j6_facing._x << " " << j4j5j6_facing._y << " " << j4j5j6_facing._z << std::endl;
  // std::cout << "j4j5j6 rotation corrected: " << j4j5j6_rotation.getRoll() << " " << j4j5j6_rotation.getPitch() << " " << j4j5j6_rotation.getYaw() << std::endl;
  diff = j4j5j6_rotation.minus(wrist_to_forearm_rotation).getAxisAngle().angle.getRadians();
  // std::cout << "diff corrected" << diff << std::endl;
  // std::cout << theta4 << " " << theta5 << " " << theta6 << std::endl;

  if(diff > 0.01){
    print("diff is too big, %f\r\n", diff);
    print("%f %f %f \n", wrist_to_forearm_rotation.getRoll(), wrist_to_forearm_rotation.getPitch(), wrist_to_forearm_rotation.getYaw());
    osDelay(10);
  }

  // optimize j4 angle when arm is pointing forward, keeps the most recent angle, don't return to 0.
  if(wrist_facing == Vector3d(1,0,0)){
    theta4 = joint_angles.forearm_roll_4;
  }

  return {0,theta1, theta2, theta3, theta4, theta5, theta6};
}


void checkEncodersConnected(){
  while(!encoder0->is_connected() || !encoder1->is_connected()){
    if(!encoder0->is_connected()){
      print("encoder 0 not connected\r\n");
    }
    osDelay(100);
    if (!encoder1->is_connected()){
      print("encoder 1 not connected\r\n");
    }
    print("encoder 0: %d encoder 1: %d\r\n", encoder0->getData(), encoder1->getData());
    osDelay(100);
  }
}

void armTask(void* args) {
  UNUSED(args);
  
  
  // Mechanical Angle refers to the angle reference defined manually for easy kinematics/inverse kinematics and/or other purposes
  // for joint 2, mechanical angle is the reading of encoder0 - encoder0_angle_when_upright
  // for joint 3, mechanical angle is the reading of encoder1-encoder0 - encoder1_angle_when_upright - encoder0_angle_when_upright
  // mechanical angle range

  const float SBUS_CHANNEL_MAX = 660;

  joint_state_t current_motor_angles = {0,0,0,0,0,0,0};

  //Add 1s Time for 4310 start up.
  print("START\r\n");
  osDelay(2000);

  // check if encoders are connected
  checkEncodersConnected();

  // manually enable motors
  
  print("waiting for sbus channel 10 to be greater than 100, now %d\r\n",sbus->ch[12]);

  waitUntil([]()->bool {return sbus->ch[12]>100;});



  // enable 4310

  // uncomment SetZeroPos() if remounted 4310

  forearm_rotate_motor_4->SetZeroPos();
  osDelay(10);
  wrist_rotate_motor_5->SetZeroPos();
  osDelay(10);
  hand_rotate_motor_6->SetZeroPos();  
  osDelay(10);



  forearm_rotate_motor_4->MotorEnable();
  wrist_rotate_motor_5->MotorEnable();
  hand_rotate_motor_6->MotorEnable();

  // check if all motors connected, this has to be after 4310 motors enabled
  print("checking all motors connected\r\n");
  checkAllMotorsConnected();

  // load current motor angles, need to be after checkAllMotorsConnected, or else A1 will not return data
  ArmLoadInput(current_motor_angles);

  // set 4310 initial position
  forearm_rotate_motor_4->SetOutput(current_motor_angles.forearm_roll_4, 0, 10, 0.5, 0);
  wrist_rotate_motor_5->SetOutput(current_motor_angles.wrist_5, 0, 10, 0.5, 0);
  hand_rotate_motor_6->SetOutput(current_motor_angles.end_6, 0, 10, 0.5, 0);

  control::Motor4310* forearm_motors[3] = {forearm_rotate_motor_4, wrist_rotate_motor_5, hand_rotate_motor_6};
  control::Motor4310::TransmitOutput(forearm_motors, 3); 

  print("4310 motors enabled, but should not move\r\n");

  
  // read A1 offsets, this has to be after checkAllMotorsConnected
  // encoder1 - encoder0 = A1_id2 angle
  // encoder0 = A1_id1 angle
  A1_id2_offset = encoder1->getData() + encoder0->getData() - MotorA1_recv_id02.Pos;
  A1_id1_offset = encoder0->getData() - MotorA1_recv_id01.Pos;
  A1_id0_offset = -MotorA1_recv_id00.Pos;
  print("A1 offsets read\r\n");

  // initialize target angles
  last_angular_velocities = {0,0,0,0,0,0,0};  
  last_targets = current_motor_angles; // target angles initially match current motor angles

  MovingAverage moving_average[7]; // filter for 7 "joint"s

  // Motion Profile for each joint
  TrapezoidProfile joint_1_profile(1.0, MAX_SPEED_RADIANS.base_yaw_rotate_1); // acceleration, velocity
  TrapezoidProfile joint_2_profile(1.0, MAX_SPEED_RADIANS.base_pitch_rotate_2);
  TrapezoidProfile joint_3_profile(1.0, MAX_SPEED_RADIANS.forearm_pitch_3);
  TrapezoidProfile joint_4_profile(4.0, MAX_SPEED_RADIANS.forearm_roll_4);
  TrapezoidProfile joint_5_profile(4.0, MAX_SPEED_RADIANS.wrist_5);
  TrapezoidProfile joint_6_profile(4.0, MAX_SPEED_RADIANS.end_6);

#ifdef INV_KINEMATICS
  // xyz profile for arm
  TrapezoidProfile x_profile(0.2, 0.1);
  TrapezoidProfile y_profile(0.2, 0.1);
  TrapezoidProfile z_profile(0.2, 0.1);
  Vector3d last_setpoint_velocity(0,0,0);
  Vector3d last_setpoint_position = last_target_position;
#endif

  int loop_cnt = 0;
  float last_100loop_time = HAL_GetTick();
  UNUSED(last_100loop_time);
  while (true) {
    
    // if killed do nothing
    if(killed){
      osDelay(100);
      continue;
    }
  
    float filtered_sbus[16];

    // read and filter sbus

    // manage input from sbus
    moving_average[1].AddSample(sbus->ch[0]); // base_yaw
    moving_average[2].AddSample(sbus->ch[1]); // base_pitch
    moving_average[3].AddSample(sbus->ch[2]); // elbow_pitch
    moving_average[4].AddSample(sbus->ch[3]); // forearm_roll
    moving_average[5].AddSample(sbus->ch[4]); // wrist
    moving_average[6].AddSample(sbus->ch[5]); // end


    if(sbus->ch[10] > 100){ // exchange ore
      base_rotate_offset = M_PI/2;
    }else if(sbus->ch[10] < -100){ // get ore
      base_rotate_offset = -M_PI/2;
    }else{
      base_rotate_offset = 0;    
    }

    if(sbus->ch[13] > 0){
      pump->On();
     }else{
      pump->Off();
    }

    // value range from -1 to 1
    filtered_sbus[1] = clip<float>(moving_average[1].GetAverage(), -SBUS_CHANNEL_MAX, SBUS_CHANNEL_MAX)/SBUS_CHANNEL_MAX;
    filtered_sbus[2] = clip<float>(moving_average[2].GetAverage(), -SBUS_CHANNEL_MAX, SBUS_CHANNEL_MAX)/SBUS_CHANNEL_MAX;
    filtered_sbus[3] = clip<float>(moving_average[3].GetAverage(), -SBUS_CHANNEL_MAX, SBUS_CHANNEL_MAX)/SBUS_CHANNEL_MAX;
    filtered_sbus[4] = clip<float>(moving_average[4].GetAverage(), -SBUS_CHANNEL_MAX, SBUS_CHANNEL_MAX)/SBUS_CHANNEL_MAX;
    filtered_sbus[5] = clip<float>(moving_average[5].GetAverage(), -SBUS_CHANNEL_MAX, SBUS_CHANNEL_MAX)/SBUS_CHANNEL_MAX;
    filtered_sbus[6] = clip<float>(moving_average[6].GetAverage(), -SBUS_CHANNEL_MAX, SBUS_CHANNEL_MAX)/SBUS_CHANNEL_MAX;


    // ch 0~5 are arm joint angles
    // ch 6 & 7 are chassis vx and vy
    // ch 8 is chassis rotate 
    // ch 9 is base translate 3508 motor
    // ch 10 is  90 degree offset switch
    // ch 11 is kill switch
    // ch 12 is confirm start switch
    // ch 13 is pump

#ifdef INV_KINEMATICS

    // x is forward, y is left, z is up
    Vector3d target_position = last_target_position;
    target_position._x += 0.2*-filtered_sbus[3]*0.006;
    target_position._y += 0.2*filtered_sbus[1]*0.006;
    target_position._z += 0.2*-filtered_sbus[2]*0.006;

    // make sure target position is in range
    target_position._z = clip<float>(target_position._z, 0.1, 0.65);
    target_position._x = clip<float>(target_position._x, 0.1, 0.65);
    target_position._y = clip<float>(target_position._y, -0.65, 0.65);

    kinematics_state x_state = x_profile.calculate(target_position._x, 6, {last_setpoint_position._x, last_setpoint_velocity._x});
    kinematics_state y_state = y_profile.calculate(target_position._y, 6, {last_setpoint_position._y, last_setpoint_velocity._y});
    kinematics_state z_state = z_profile.calculate(target_position._z, 6, {last_setpoint_position._z, last_setpoint_velocity._z});

    Vector3d current_setpoint_velocity(x_state.velocity, y_state.velocity, z_state.velocity);
    Vector3d current_setpoint_position(x_state.position, y_state.position, z_state.position);


    // calculate ik based on current_setpoint (position) and orientation
  
    Rotation3d orientation(filtered_sbus[4]*M_PI, filtered_sbus[5]*M_PI/2, filtered_sbus[6]*M_PI);
    Vector3d wrist_translation = Vector3d(wrist_length,0,0).rotateBy(orientation);
    Vector3d forearm_setpoint_position = current_setpoint_position - wrist_translation;
    Vector3d forearm_goal_position = target_position - wrist_translation;
    // current_position *= -1;
    
    joint_state_t mechanical_angle_setpoint = inverse_kinematics(forearm_setpoint_position, orientation, current_mechanical_angle);
    joint_state_t goal_mechanical_angle = inverse_kinematics(forearm_goal_position, orientation, current_mechanical_angle);
    UNUSED(goal_mechanical_angle);
    last_target_position = target_position; // xyz
    if(isJointTargetsLegal(mechanical_angle_setpoint)){
      target_mechanical_angle = mechanical_angle_setpoint;


      last_setpoint_position = current_setpoint_position;
      last_setpoint_velocity = current_setpoint_velocity;


    // TODO: if we have this line, arm rotate motor will calculate to a very large angle
    // }else if(isJointTargetsLegal(goal_mechanical_angle)){ // if goal is legal but next setpoint is not, fallback to joint movement
      // target_mechanical_angle = goal_mechanical_angle;
    }else{
      last_setpoint_velocity = {0,0,0};
      if(loop_cnt % 100 == 0)
        print("ik result not legal %f %f %f %f %f %f\n", mechanical_angle_setpoint.base_yaw_rotate_1, mechanical_angle_setpoint.base_pitch_rotate_2, mechanical_angle_setpoint.forearm_pitch_3, mechanical_angle_setpoint.forearm_roll_4, mechanical_angle_setpoint.wrist_5, mechanical_angle_setpoint.end_6);
    }
    // otherwise, A1 targets don't change
#else
    // map sbus input to mechanical angle range
    target_mechanical_angle.base_yaw_rotate_1   = map<float>(filtered_sbus[1], MECHANICAL_MIN_POS.base_yaw_rotate_1,   MECHANICAL_MAX_POS.base_yaw_rotate_1);
    target_mechanical_angle.base_pitch_rotate_2 = map<float>(filtered_sbus[2], MECHANICAL_MIN_POS.base_pitch_rotate_2, MECHANICAL_MAX_POS.base_pitch_rotate_2);
    target_mechanical_angle.forearm_pitch_3     = map<float>(filtered_sbus[3], MECHANICAL_MIN_POS.forearm_pitch_3,     MECHANICAL_MAX_POS.forearm_pitch_3);
    target_mechanical_angle.forearm_roll_4      = map<float>(filtered_sbus[4], MECHANICAL_MIN_POS.forearm_roll_4,      MECHANICAL_MAX_POS.forearm_roll_4);
    target_mechanical_angle.wrist_5             = map<float>(filtered_sbus[5], MECHANICAL_MIN_POS.wrist_5,             MECHANICAL_MAX_POS.wrist_5);
    target_mechanical_angle.end_6               = map<float>(filtered_sbus[6], MECHANICAL_MIN_POS.end_6,               MECHANICAL_MAX_POS.end_6);
#endif 

   
    // joint_state_t joint_targets = mechanical_angle_to_motor_angle(target_mechanical_angle);
    // mechanical angle to encoder angle 
    joint_state_t encoder_angle = target_mechanical_angle; 
    encoder_angle.base_pitch_rotate_2 = target_mechanical_angle.base_pitch_rotate_2 + encoder0_val_when_arm_upright;
    encoder_angle.forearm_pitch_3 = target_mechanical_angle.forearm_pitch_3 + encoder0_val_when_arm_upright + encoder1_val_when_arm_upright;

    joint_state_t joint_targets = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  
    // Convert mechanical angle to motor command angle.
    joint_targets.base_yaw_rotate_1 = encoder_angle.base_yaw_rotate_1 - A1_id0_offset + base_rotate_offset; // no abs encoder
    joint_targets.base_pitch_rotate_2 = encoder_angle.base_pitch_rotate_2 - A1_id1_offset; // has abs encoder
    joint_targets.forearm_pitch_3 = encoder_angle.forearm_pitch_3 - A1_id2_offset; // has abs encoder

    joint_targets.forearm_roll_4 = wrap<float>(encoder_angle.forearm_roll_4, current_motor_angles.forearm_roll_4-PI, current_motor_angles.forearm_roll_4+PI); // wrap to the closest rotation
    if(joint_targets.forearm_roll_4 < MECHANICAL_MIN_POS.forearm_roll_4){
      joint_targets.forearm_roll_4 += 2*PI;
    }else if(joint_targets.forearm_roll_4 > MECHANICAL_MAX_POS.forearm_roll_4){
      joint_targets.forearm_roll_4 -= 2*PI;
    }
    // joint_targets.forearm_roll_4 = encoder_angle.forearm_roll_4;

    joint_targets.wrist_5 = -encoder_angle.wrist_5;

    // joint_targets.end_6 = encoder_angle.end_6;
    joint_targets.end_6 = wrap<float>(encoder_angle.end_6, current_motor_angles.end_6-PI, current_motor_angles.end_6+PI); // wrap to the closest rotation
    if(joint_targets.end_6 < MECHANICAL_MIN_POS.end_6){
      joint_targets.end_6 += 2*PI;
    }else if(joint_targets.end_6 > MECHANICAL_MAX_POS.end_6){
      joint_targets.end_6 -= 2*PI;
    }

    // UNUSED(encoder_angle);
    // joint_targets = {0, -0.022778, -0.119204, -0.607556, -1.644363, 0.794053, 1.627852};

    // apply kinematics constraints    
    kinematics_state joint_1_state = joint_1_profile.calculate(joint_targets.base_yaw_rotate_1,   6, {last_targets.base_yaw_rotate_1,   last_angular_velocities.base_yaw_rotate_1});
    kinematics_state joint_2_state = joint_2_profile.calculate(joint_targets.base_pitch_rotate_2, 6, {last_targets.base_pitch_rotate_2, last_angular_velocities.base_pitch_rotate_2});
    kinematics_state joint_3_state = joint_3_profile.calculate(joint_targets.forearm_pitch_3,     6, {last_targets.forearm_pitch_3,     last_angular_velocities.forearm_pitch_3});
    kinematics_state joint_4_state = joint_4_profile.calculate(joint_targets.forearm_roll_4,      6, {last_targets.forearm_roll_4,      last_angular_velocities.forearm_roll_4});
    kinematics_state joint_5_state = joint_5_profile.calculate(joint_targets.wrist_5,             6, {last_targets.wrist_5,             last_angular_velocities.wrist_5});
    kinematics_state joint_6_state = joint_6_profile.calculate(joint_targets.end_6,               6, {last_targets.end_6,               last_angular_velocities.end_6});

    // prevent sudden movement
    // joint_1_state.position = clip<float>(joint_1_state.position, current_motor_angles.base_yaw_rotate_1  -MAX_SPEED_RADIANS.base_yaw_rotate_1*0.006,    current_motor_angles.base_yaw_rotate_1  +MAX_SPEED_RADIANS.base_yaw_rotate_1*0.006);
    // joint_2_state.position = clip<float>(joint_2_state.position, current_motor_angles.base_pitch_rotate_2-MAX_SPEED_RADIANS.base_pitch_rotate_2*0.006,  current_motor_angles.base_pitch_rotate_2+MAX_SPEED_RADIANS.base_pitch_rotate_2*0.006);
    // joint_3_state.position = clip<float>(joint_3_state.position, current_motor_angles.forearm_pitch_3    -MAX_SPEED_RADIANS.forearm_pitch_3*0.006,      current_motor_angles.forearm_pitch_3    +MAX_SPEED_RADIANS.forearm_pitch_3*0.006);
    // joint_4_state.position = clip<float>(joint_4_state.position, current_motor_angles.forearm_roll_4     -MAX_SPEED_RADIANS.forearm_roll_4*0.006,       current_motor_angles.forearm_roll_4     +MAX_SPEED_RADIANS.forearm_roll_4*0.006);
    // joint_5_state.position = clip<float>(joint_5_state.position, current_motor_angles.wrist_5            -MAX_SPEED_RADIANS.wrist_5*0.006,              current_motor_angles.wrist_5            +MAX_SPEED_RADIANS.wrist_5*0.006);
    // joint_6_state.position = clip<float>(joint_6_state.position, current_motor_angles.end_6              -MAX_SPEED_RADIANS.end_6*0.006,                current_motor_angles.end_6              +MAX_SPEED_RADIANS.end_6*0.006);

    // modify target angles
    if(isJointTargetsLegal(target_mechanical_angle)){
      last_targets =    {0, joint_1_state.position, joint_2_state.position, joint_3_state.position, joint_4_state.position, joint_5_state.position, joint_6_state.position};
    }else{
      if(loop_cnt % 100 == 0){
        print("mechanical angle illegal\r\n");
      }
      last_targets = {0, joint_1_state.position, last_targets.base_pitch_rotate_2, last_targets.forearm_pitch_3, joint_4_state.position, joint_5_state.position, joint_6_state.position};
    }

    last_angular_velocities = {0, joint_1_state.velocity, joint_2_state.velocity, joint_3_state.velocity, joint_4_state.velocity, joint_5_state.velocity, joint_6_state.velocity};

    // print("start setting angles %f %f %f %f %f %f\n", last_targets.base_yaw_rotate_1, last_targets.base_pitch_rotate_2, last_targets.forearm_pitch_3, last_targets.forearm_roll_4, last_targets.wrist_5, last_targets.end_6);
    // osDelay(10);
    ArmSetTargetAngles(last_targets);
    // print("finish set angles\n");

    // print every 100 loops
    if(loop_cnt % 100 == 0){
      // print("encoder0 %f, encoder1 %f\n", encoder0->getData(), encoder1->getData());
      // print("A1_offset %f, %f\n", A1_id1_offset, A1_id2_offset);
      // print("A1 current: %f. %f \n", MotorA1_recv_id00.current);
#ifdef INV_KINEMATICS
      print("position: %f, %f, %f \n", last_target_position._x, last_target_position._y, last_target_position._z);
      print("orientation: %f, %f, %f \n", orientation.getRoll(), orientation.getPitch(), orientation.getYaw());
#endif
      // print("mechanical: %f, %f, %f, %f, %f, %f \n", mechanical_angle.base_yaw_rotate_1, mechanical_angle.base_pitch_rotate_2, mechanical_angle.forearm_pitch_3, mechanical_angle.forearm_roll_4, mechanical_angle.wrist_5, mechanical_angle.end_6);
      // print("current: %f, %f, %f, %f, %f, %f \n", current_motor_angles.base_yaw_rotate_1, current_motor_angles.base_pitch_rotate_2, current_motor_angles.forearm_pitch_3, current_motor_angles.forearm_roll_4, current_motor_angles.wrist_5, current_motor_angles.end_6);
      // print("current mechanical angles: %f, %f, %f, %f, %f, %f \n", current_mechanical_angle.base_yaw_rotate_1, current_mechanical_angle.base_pitch_rotate_2, current_mechanical_angle.forearm_pitch_3, current_mechanical_angle.forearm_roll_4, current_mechanical_angle.wrist_5, current_mechanical_angle.end_6);

      if(!isJointTargetsLegal(current_mechanical_angle)){
        print("current mechanical angle illegal: \n");

      }
      // print("targets: %f, %f, %f, %f, %f, %f \n",last_targets.base_yaw_rotate_1, last_targets.base_pitch_rotate_2, last_targets.forearm_pitch_3, last_targets.forearm_roll_4, last_targets.wrist_5, last_targets.end_6);
      // print("loop time %fms \r\n", (HAL_GetTick()-last_100loop_time)/100.0);
      // print("d: %f, %f \r\n", hand_rotate_motor_6->GetTheta(), last_targets.end_6);
      last_100loop_time = HAL_GetTick();

    }

    // manually confirm on the first loop to prevent accidental movement
    if(loop_cnt == 0){
      print("waiting for sbus channel 10 to be smaller than -100\r\n");
      waitUntil([]()->bool{return sbus->ch[12] < -100;});
      last_100loop_time = HAL_GetTick();
    }


    ArmLoadInput(current_motor_angles);
    // current_mechanical_angle = motor_command_to_mechanical_angle(current_motor_angles);

    // first convert motor angles to encoder angles
    current_mechanical_angle = current_motor_angles;
    current_mechanical_angle.base_yaw_rotate_1 += A1_id0_offset - base_rotate_offset;
    current_mechanical_angle.base_pitch_rotate_2 += A1_id1_offset;
    current_mechanical_angle.forearm_pitch_3 += A1_id2_offset;

    // joint_state_t mechanical_angle = encoder_angle;
    current_mechanical_angle.base_pitch_rotate_2 += - encoder0_val_when_arm_upright;
    current_mechanical_angle.forearm_pitch_3 += - encoder0_val_when_arm_upright - encoder1_val_when_arm_upright;
    current_mechanical_angle.wrist_5 *= -1;
    // current_mechanical_angle.forearm_roll_4 -= PI/2;
    osDelay(ARM_TASK_DELAY);
    loop_cnt++;
  }
}

static const float A1_Kp = 0.03, A1_Kw = 2.0;
static const float m4310_Kp= 3, m4310_Kd = 1;

/**
 * @brief set the target joint angles for the arm
*/
int ArmSetTargetAngles(joint_state_t target) {
  // A1 
  modify_pos_cmd(&MotorA1_send, 0, target.base_yaw_rotate_1, 0.06, 8.0);
  // modify_stop_cmd(&MotorA1_send, 0);
  unitreeA1_rxtx(huart1);

  modify_pos_cmd(&MotorA1_send, 1, target.base_pitch_rotate_2, 0.06, 8.0);
  // modify_stop_cmd(&MotorA1_send, 1);
  unitreeA1_rxtx(huart1);

  modify_pos_cmd(&MotorA1_send, 2, target.forearm_pitch_3, 0.06, 8.0);
  // modify_stop_cmd(&MotorA1_send, 2);
  unitreeA1_rxtx(huart1);


  // 4310
  forearm_rotate_motor_4->SetOutput(target.forearm_roll_4, 0, 10, 0.5, 0);
  wrist_rotate_motor_5->SetOutput(target.wrist_5, 0, 10, 0.5, 0);
  hand_rotate_motor_6->SetOutput(target.end_6, 0, 10, 0.5, 0);
  control::Motor4310* forearm_motors[3] = {forearm_rotate_motor_4, wrist_rotate_motor_5, hand_rotate_motor_6};
  UNUSED(forearm_motors);
  control::Motor4310::TransmitOutput(forearm_motors, 3);
  // control::Motor4310::TransmitOutput(&hand_rotate_motor_6,1);

  return 0;
}


void ArmLoadInput(joint_state_t &current_joint_positions) {
  current_joint_positions.base_yaw_rotate_1 = MotorA1_recv_id00.Pos;
  current_joint_positions.base_pitch_rotate_2 = MotorA1_recv_id01.Pos;
  current_joint_positions.forearm_pitch_3 = MotorA1_recv_id02.Pos;
  current_joint_positions.forearm_roll_4 = forearm_rotate_motor_4->GetTheta(); 
  current_joint_positions.wrist_5 = wrist_rotate_motor_5->GetTheta();
  current_joint_positions.end_6 = hand_rotate_motor_6->GetTheta();
}

void kill_arm() {
  killed = true;

  modify_stop_cmd(&MotorA1_send, 0);
  unitreeA1_rxtx(huart1);

  modify_stop_cmd(&MotorA1_send, 1);  
  unitreeA1_rxtx(huart1);

  modify_stop_cmd(&MotorA1_send, 2);
  unitreeA1_rxtx(huart1);

  forearm_rotate_motor_4->MotorDisable();
  wrist_rotate_motor_5->MotorDisable();
  hand_rotate_motor_6->MotorDisable();
}

void revive_arm(){
  killed = false;

  last_angular_velocities = {0,0,0,0,0,0,0};  
  last_targets = {0,0,0,0,0,0,0}; // target angles initially match current motor angles

  forearm_rotate_motor_4->MotorEnable();
  wrist_rotate_motor_5->MotorEnable();
  hand_rotate_motor_6->MotorEnable();
  ArmLoadInput(last_targets);

  
}
