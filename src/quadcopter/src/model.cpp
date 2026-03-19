// BSD 3-Clause License
//
// Copyright (c) 2026, Dinay Kingkiller
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "quadcopter/model.h"

#include <cmath>
#include <random>
#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose.h"

#include "quadcopter/dynamics.h"
#include "quadcopter/types.h"
#include "quadcopter/conversions.h"

#include "quad_msgs/Motor.h"
#include "quad_msgs/Sensor.h"
#include "quad_msgs/ICM20948.h"

namespace quadcopter
{
Model::Model(ros::NodeHandle node)
: node_(node)
{
  Model::reset_params();
  Model::zero();
  rand_gen_ = std::mt19937(std::random_device{}());
  pose_pub_ = node.advertise<geometry_msgs::Pose>("pose", 1000);
  sensor_pub_ = node.advertise<quad_msgs::ICM20948>("ICM20948", 1000);
}
/*
void Model::sense(const ros::TimerEvent& e)
{
  quad_msgs::Sensor sensor_actual;
  quad_msgs::ICM20948 sensor_raw;

  sensor_actual.accelerometer = accel_;
  sensor_actual.gyroscope = gyro_;

  // Add sensor noise
  accel_.x += accel_noise_(rand_gen_);
  accel_.y += accel_noise_(rand_gen_);
  accel_.z += accel_noise_(rand_gen_);
  gyro_.x += gyro_noise_(rand_gen_);
  gyro_.y += gyro_noise_(rand_gen_);
  gyro_.z += gyro_noise_(rand_gen_);

  // Convert to g's
  int16_t a_x = static_cast<int16_t>(std::round(accel_.x*accel_scale_/k_gravity_));
  int16_t a_y = static_cast<int16_t>(std::round(accel_.y*accel_scale_/k_gravity_));
  int16_t a_z = static_cast<int16_t>(std::round(accel_.z*accel_scale_/k_gravity_));
  int16_t g_x = static_cast<int16_t>(std::round(gyro_.x*gyro_scale_*180./M_PI));
  int16_t g_y = static_cast<int16_t>(std::round(gyro_.y*gyro_scale_*180./M_PI));
  int16_t g_z = static_cast<int16_t>(std::round(gyro_.z*gyro_scale_*180./M_PI));

  // Convert to Bytes
  sensor_raw.ACCEL_XOUT_H = static_cast<uint8_t>((a_x >> 8) & 0xFF);
  sensor_raw.ACCEL_XOUT_L = static_cast<uint8_t>(a_x & 0xFF);
  sensor_raw.ACCEL_YOUT_H = static_cast<uint8_t>((a_y >> 8) & 0xFF);
  sensor_raw.ACCEL_YOUT_L = static_cast<uint8_t>(a_y & 0xFF);
  sensor_raw.ACCEL_ZOUT_H = static_cast<uint8_t>((a_z >> 8) & 0xFF);
  sensor_raw.ACCEL_ZOUT_L = static_cast<uint8_t>(a_z & 0xFF);
  sensor_raw.GYRO_XOUT_H = static_cast<uint8_t>((g_x >> 8) & 0xFF);
  sensor_raw.GYRO_XOUT_L = static_cast<uint8_t>(g_x & 0xFF);
  sensor_raw.GYRO_YOUT_H = static_cast<uint8_t>((g_y >> 8) & 0xFF);
  sensor_raw.GYRO_YOUT_L = static_cast<uint8_t>(g_y & 0xFF);
  sensor_raw.GYRO_ZOUT_H = static_cast<uint8_t>((g_z >> 8) & 0xFF);
  sensor_raw.GYRO_ZOUT_L = static_cast<uint8_t>(g_z & 0xFF);

  sensor_pub_.publish(sensor_raw);
}
*/
void Model::reset_params()
{
  // Physical Parameters
  node_.getParam("Mass", params_.mass);
  node_.getParam("Radius", params_.radius);
  node_.getParam("GAccel", params_.gravity);
  node_.getParam("kForce", params_.force);
  node_.getParam("kTorque", params_.torque);

  // Sensor Parameters
  int gyro_fs_sel, accel_fs_sel;
  std::vector<float> gyro_fs_var, accel_fs_var, gyro_scale_var, accel_scale_var;
  node_.getParam("GYRO_FS_SEL", gyro_fs_sel);
  node_.getParam("ACCEL_FS_SEL", accel_fs_sel);
  node_.getParam("Gyro_FS_Var", gyro_fs_var);
  node_.getParam("Accel_FS_Var", accel_fs_var);
  node_.getParam("Gyro_Scale", gyro_scale_var);
  node_.getParam("Accel_Scale", accel_scale_var);
  gyro_noise_ = std::normal_distribution<double>(0.0, gyro_fs_var[gyro_fs_sel]);
  accel_noise_ = std::normal_distribution<double>(0.0, accel_fs_var[accel_fs_sel]);
  gyro_scale_ = gyro_scale_var[gyro_fs_sel];
  accel_scale_ = accel_scale_var[accel_fs_sel];
}
void Model::zero()
{
  time_ = ros::Time::now();
  state_.p = {0.0, 0.0, 0.0};
  state_.v = {0.0, 0.0, 0.0};
  state_.q = {0.0, 0.0, 0.0, 1.0};
  state_.w = {0.0, 0.0, 0.0};
  input_ = {0.0, 0.0, 0.0, 0.0};
}
  void Model::move(const quad_msgs::Motor::ConstPtr& msg)
{
  input_ = toStruct(*msg);
}

void Model::update(const ros::TimerEvent& e)
{
  ros::Time this_time = ros::Time::now();
  double dt = (this_time-time_).toSec();
  state_ = step(state_, input_, params_, dt);
  time_ = this_time;

  // Publish new pose.
  geometry_msgs::Pose new_pose = toMsg(state_);
  pose_pub_.publish(new_pose);
}
} // namespace quadcopter
