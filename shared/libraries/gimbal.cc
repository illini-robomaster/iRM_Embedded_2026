#include "gimbal.h"

#include "motor.h"
#include "utils.h"

namespace control {

Gimbal::Gimbal(gimbal_t gimbal)
    : pitch_detector_(BoolEdgeDetector(false)), yaw_detector_(BoolEdgeDetector(false)) {
  // acquired from user
  pitch_motor_4310_ = gimbal.pitch_motor_4310;
  yaw_motor_4310_ = gimbal.yaw_motor_4310;
  pitch_motor_ = gimbal.pitch_motor;
  yaw_motor_ = gimbal.yaw_motor;
  model_ = gimbal.model;

  float pitch_theta_max_iout = 0;
  float pitch_theta_max_out = 10;
  float pitch_omega_max_iout = 10000;
  float pitch_omega_max_out = 30000;
  float yaw_theta_max_iout = 0;
  float yaw_theta_max_out = 10;
  float yaw_omega_max_iout = 25000;
  float yaw_omega_max_out = 30000;

  // data initialization using acquired model
  switch (gimbal.model) {
    case GIMBAL_FORTRESS:
      data_.pitch_offset_ = 5.205564f;
      data_.yaw_offset_ = 1.8132f;
      data_.pitch_max_ = 0.5080f;
      data_.yaw_max_ = PI;
      {
        pitch_theta_max_iout = 0;
        pitch_theta_max_out = 10;
        pitch_omega_max_iout = 10000;
        pitch_omega_max_out = 30000;
        yaw_theta_max_iout = 0;
        yaw_theta_max_out = 10;
        yaw_omega_max_iout = 25000;  // 10000
        yaw_omega_max_out = 30000;
        pitch_theta_pid_param_ = new float[3]{15, 0, 0};
        pitch_omega_pid_param_ = new float[3]{2900, 60, 0};
        yaw_theta_pid_param_ = new float[3]{26, 0, 0.3};
        yaw_omega_pid_param_ = new float[3]{3600, 20, 0};
      }
      break;
    case GIMBAL_SENTRY:
      /* ICRA gimbal setting */
      data_.yaw_offset_ = 0.2f;
      data_.pitch_offset_ = 1.5f;
      data_.pitch_max_ = 0.3;
      data_.yaw_max_ = 1.5;
      {
        pitch_theta_max_iout = 0;
        pitch_theta_max_out = 10;
        pitch_omega_max_iout = 10000;
        pitch_omega_max_out = 30000;
        yaw_theta_max_iout = 0;
        yaw_theta_max_out = 10;
        yaw_omega_max_iout = 10000;
        yaw_omega_max_out = 30000;
        pitch_theta_pid_param_ = new float[3]{17, 0, 1};
        pitch_omega_pid_param_ = new float[3]{2000, 15, 5};
        yaw_theta_pid_param_ = new float[3]{18, 0, 5};
        yaw_omega_pid_param_ = new float[3]{2100, 10, 10};
      }
      break;
    case GIMBAL_STEERING:
      data_.pitch_offset_ = 5.25f;
      data_.yaw_offset_ = 4.52;
      data_.pitch_max_ = 0.5080f;
      data_.yaw_max_ = PI;
      {
        pitch_theta_max_iout = 0;
        pitch_theta_max_out = 10;
        pitch_omega_max_iout = 10000;
        pitch_omega_max_out = 30000;
        yaw_theta_max_iout = 0;
        yaw_theta_max_out = 10;
        yaw_omega_max_iout = 10000;  // 10000
        yaw_omega_max_out = 30000;
        pitch_theta_pid_param_ = new float[3]{20, 0, 0};
        pitch_omega_pid_param_ = new float[3]{2900, 60, 0};
        yaw_theta_pid_param_ = new float[3]{30, 0, 0.3};
        yaw_omega_pid_param_ = new float[3]{3600, 20, 0};
      }
      break;
    case GIMBAL_STEERING_4310:
      //      data_.yaw_offset_ = 0.3451;
      data_.yaw_offset_ = 4.52;
      data_.yaw_max_ = PI;
      {
        yaw_theta_max_iout = 0;
        yaw_theta_max_out = 10;
        yaw_omega_max_iout = 10000;  // 10000
        yaw_omega_max_out = 30000;

        yaw_theta_pid_param_ = new float[3]{30, 0, 0.3};
        yaw_omega_pid_param_ = new float[3]{3600, 20, 0};
        yaw_theta_pid_ =
            new ConstrainedPID(yaw_theta_pid_param_, yaw_theta_max_iout, yaw_theta_max_out);
        yaw_omega_pid_ =
            new ConstrainedPID(yaw_omega_pid_param_, yaw_omega_max_iout, yaw_omega_max_out);
      }
      break;
    default:
      RM_ASSERT_TRUE(false, "Not Supported Gimbal Mode\r\n");
  }

  // nullptr guard
  switch (gimbal.model) {
    case GIMBAL_STEERING_4310:
      if (yaw_theta_pid_param_ == nullptr || yaw_omega_pid_param_ == nullptr) {
        RM_ASSERT_TRUE(false, "Not Supported Gimbal Mode\r\n");
      }
      break;
    default:
      if (pitch_theta_pid_param_ == nullptr || pitch_omega_pid_param_ == nullptr ||
          yaw_theta_pid_param_ == nullptr || yaw_omega_pid_param_ == nullptr) {
        RM_ASSERT_TRUE(false, "Not Supported Gimbal Mode\r\n");
      }
  }

  pitch_theta_pid_ =
      new ConstrainedPID(pitch_theta_pid_param_, pitch_theta_max_iout, pitch_theta_max_out);
  pitch_omega_pid_ =
      new ConstrainedPID(pitch_omega_pid_param_, pitch_omega_max_iout, pitch_omega_max_out);
  yaw_theta_pid_ =
      new ConstrainedPID(yaw_theta_pid_param_, yaw_theta_max_iout, yaw_theta_max_out);
  yaw_omega_pid_ =
      new ConstrainedPID(yaw_omega_pid_param_, yaw_omega_max_iout, yaw_omega_max_out);

  pitch_angle_ = data_.pitch_offset_;
  yaw_angle_ = data_.yaw_offset_;

  pitch_lower_limit_ = wrap<float>(data_.pitch_offset_ - data_.pitch_max_, 0, 2 * PI);
  pitch_upper_limit_ = wrap<float>(data_.pitch_offset_ + data_.pitch_max_, 0, 2 * PI);
  yaw_lower_limit_ = wrap<float>(data_.yaw_offset_ - data_.yaw_max_, 0, 2 * PI);
  yaw_upper_limit_ = wrap<float>(data_.yaw_offset_ + data_.yaw_max_, 0, 2 * PI);
}

Gimbal::~Gimbal() {
  switch (model_) {
    case GIMBAL_STEERING_4310: {
      delete yaw_theta_pid_param_;
      yaw_theta_pid_param_ = nullptr;
      delete yaw_omega_pid_param_;
      yaw_omega_pid_param_ = nullptr;
      delete yaw_theta_pid_;
      yaw_theta_pid_ = nullptr;
      delete yaw_omega_pid_;
      yaw_omega_pid_ = nullptr;
      break;
    }
    default:
      delete pitch_theta_pid_param_;
      pitch_theta_pid_param_ = nullptr;
      delete pitch_omega_pid_param_;
      pitch_omega_pid_param_ = nullptr;
      delete yaw_theta_pid_param_;
      yaw_theta_pid_param_ = nullptr;
      delete yaw_omega_pid_param_;
      yaw_omega_pid_param_ = nullptr;
      delete pitch_theta_pid_;
      pitch_theta_pid_ = nullptr;
      delete pitch_omega_pid_;
      pitch_omega_pid_ = nullptr;
      delete yaw_theta_pid_;
      yaw_theta_pid_ = nullptr;
      delete yaw_omega_pid_;
      yaw_omega_pid_ = nullptr;
  }
}

gimbal_data_t* Gimbal::GetData() { return &data_; }

void Gimbal::Update() {
  switch (model_) {
    case GIMBAL_STEERING_4310: {
      float yt_diff = yaw_motor_->GetThetaDelta(yaw_angle_);
      float yt_out = yaw_theta_pid_->ComputeOutput(yt_diff);
      float yt_in = yaw_motor_->GetOmegaDelta(yt_out);
      float yo_out = yaw_omega_pid_->ComputeConstrainedOutput(yt_in);

      yaw_motor_->SetOutput(yo_out);
      break;
    }
    default:
      float pt_diff = (pitch_motor_4310_ == nullptr)
                          ? pitch_motor_->GetThetaDelta(pitch_angle_)
                          : pitch_motor_4310_->GetThetaDelta(pitch_angle_);
      float pt_out = pitch_theta_pid_->ComputeOutput(pt_diff);
      float po_in = (pitch_motor_4310_ == nullptr)
                        ? pitch_motor_->GetOmegaDelta(pt_out)
                        : pitch_motor_4310_->GetOmegaDelta(pt_out);
      float po_out = pitch_omega_pid_->ComputeConstrainedOutput(po_in);

      float yt_diff = (yaw_motor_4310_ == nullptr)
                          ? yaw_motor_->GetThetaDelta(yaw_angle_)
                          : yaw_motor_4310_->GetThetaDelta(yaw_angle_);
      float yt_out = yaw_theta_pid_->ComputeOutput(yt_diff);
      float yo_in = (yaw_motor_4310_ == nullptr)
                        ? yaw_motor_->GetOmegaDelta(yt_out)
                        : yaw_motor_4310_->GetOmegaDelta(yt_out);
      float yo_out = yaw_omega_pid_->ComputeConstrainedOutput(yo_in);

      pitch_motor_->SetOutput(po_out);
      yaw_motor_->SetOutput(yo_out);
  }
}

void Gimbal::TargetAbsWOffset(float abs_pitch, float abs_yaw) {
  switch (model_) {
    case GIMBAL_STEERING_4310: {
      float clipped_yaw = clip<float>(abs_yaw, -data_.yaw_max_, data_.yaw_max_);
      yaw_angle_ = wrap<float>(clipped_yaw + data_.yaw_offset_, 0, 2 * PI);
      break;
    }
    default:
      float clipped_pitch = clip<float>(abs_pitch, -data_.pitch_max_, data_.pitch_max_);
      float clipped_yaw = clip<float>(abs_yaw, -data_.yaw_max_, data_.yaw_max_);
      pitch_angle_ = wrapping_clip<float>(clipped_pitch + data_.pitch_offset_, pitch_lower_limit_,
                                          pitch_upper_limit_, 0, 2 * PI);
      yaw_angle_ = wrapping_clip<float>(clipped_yaw + data_.yaw_offset_, yaw_lower_limit_, yaw_upper_limit_, 0, 2 * PI);
  }
}

void Gimbal::TargetAbsNoOffset(float abs_pitch, float abs_yaw) {
  pitch_angle_ = wrapping_clip<float>(abs_pitch, pitch_lower_limit_, pitch_upper_limit_, 0, 2 * PI);
  yaw_angle_ = wrapping_clip<float>(abs_yaw, yaw_lower_limit_, yaw_upper_limit_, 0, 2 * PI);
}

void Gimbal::TargetRel(float rel_pitch, float rel_yaw) {
  switch (model_) {
    case GIMBAL_STEERING_4310: {
      yaw_angle_ = yaw_motor_->GetTheta() + rel_yaw;
      break;
    }
    default:
      pitch_angle_ = ComputePitchRel(rel_pitch, pitch_angle_);
      yaw_angle_ = ComputeYawRel(rel_yaw, yaw_angle_);
  }
}

float Gimbal::ComputePitchRel(float rel_pitch, float ref_pitch) {
  rel_pitch = clip<float>(rel_pitch, -2 * PI, 2 * PI);
  return wrapping_clip<float>(ref_pitch + rel_pitch, pitch_lower_limit_, pitch_upper_limit_, 0, 2 * PI);
}

float Gimbal::ComputeYawRel(float rel_yaw, float ref_yaw) {
  rel_yaw = clip<float>(rel_yaw, -2 * PI, 2 * PI);
  return wrapping_clip<float>(ref_yaw + rel_yaw, yaw_lower_limit_, yaw_upper_limit_, 0, 2 * PI);
}

float Gimbal::GetTargetPitchAngle() { return pitch_angle_; }

float Gimbal::GetTargetYawAngle() { return yaw_angle_; }

}  // namespace control
