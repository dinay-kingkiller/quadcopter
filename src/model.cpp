// BSD 3-Clause License
//
// Copyright (c) 2024, Dinay Kingkiller
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

#include "quadcopter/Motor.h"
#include "quadcopter/Sensor.h"
#include "quadcopter/ICM20948.h"

namespace quadcopter
{
Model::Model(ros::NodeHandle node)
: node_(node)
{
  Model::reset_params();
  Model::zero();
  rand_gen_ = std::mt19937(std::random_device{}());
  pose_pub_ = node.advertise<geometry_msgs::Pose>("pose", 1000);
  sensor_pub_ = node.advertise<quadcopter::ICM20948>("ICM20948", 1000);
}
void Model::sense(const ros::TimerEvent& e)
{
  Sensor sensor_actual;
  ICM20948 sensor_raw;

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
void Model::reset_params()
{
  // Physical Parameters
  node_.getParam("Mass", mass_);
  node_.getParam("Radius", radius_);
  node_.getParam("GAccel", k_gravity_);
  node_.getParam("kForce", k_force_);
  node_.getParam("kTorque", k_torque_);

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
  state_.p.x = 0.0;
  state_.p.y = 0.0;
  state_.p.z = 0.0;
  state_.v.x = 0.0;
  state_.v.y = 0.0;
  state_.v.z = 0.0;
  state_.q.x = 0.0;
  state_.q.y = 0.0;
  state_.q.z = 0.0;
  state_.q.w = 1.0;
  state_.w.x = 0.0;
  state_.w.y = 0.0;
  state_.w.z = 0.0;
  input_.front = 0.0;
  input_.right = 0.0;
  input_.back = 0.0;
  input_.left = 0.0;
}
void Model::move(Motor input) {input_ = input;}
State Model::get_trajectory(State state, Motor input) const
{
  State deriv;

  // Position vector derivative
  deriv.p = state.v;

  // Quaternion update from angular velocity
  deriv.q.w = -state.w.x*state_.q.x - state.w.y*state.q.y
    - state.w.z*state.q.z;
  
  deriv.q.x =  state.w.x*state.q.w + state.w.y*state.q.z
    - state.w.z*state.q.y;
  
  deriv.q.y = -state.w.x*state.q.z + state.w.y*state.q.w
    + state.w.z*state.q.x;
  
  deriv.q.z = state.w.x*state.q.y - state.w.y*state.q.x
    + state.w.z*state.q.w;

  // Sum of torques
  float r_torque = input.right*input.right - input.left * input.left;
  float p_torque = input.back * input.back - input.front * input.front;
  float y_torque = input.front * input.front + input.back * input.back
    - input.left * input.left - input.right * input.right;
  float k_moment = k_force_ / mass_ / radius_;
  
  deriv.w.x = 2.0 * k_moment * r_torque - state.w.y * state.w.z;
  deriv.w.y = 2.0 * k_moment * p_torque + state.w.x * state.w.z;
  deriv.w.z = k_moment * y_torque;

  // Velocity vector derivative
  float thrust = k_force_  * input.front * input.front
    + k_force_ * input.right * input.right
    + k_force_ * input.back * input.back
    + k_force_ * input.left * input.left;

  // Acceleration in inertial frame
  deriv.v.x = state.w.z*state.v.x - state.w.y*state.v.z - state.w.x;
  deriv.v.y = 2*thrust*state.q.x*state.q.w + 2*state.q.y*state.q.z;
  deriv.v.z = - thrust*state.q.x*state.q.x - thrust*state.q.y*state.q.y
    + thrust*state.q.z*state.q.z + thrust*state.q.w*state.q.w - k_gravity_;

  // Remove rounding errors
  if (abs(deriv.p.x) < 0.001) deriv.p.x = 0.0;
  if (abs(deriv.p.y) < 0.001) deriv.p.y = 0.0;
  if (abs(deriv.p.z) < 0.001) deriv.p.z = 0.0;
  if (abs(deriv.v.x) < 0.001) deriv.v.x = 0.0;
  if (abs(deriv.v.y) < 0.001) deriv.v.y = 0.0;
  if (abs(deriv.v.z) < 0.001) deriv.v.z = 0.0;
  if (abs(deriv.q.x) < 0.001) deriv.q.x = 0.0;
  if (abs(deriv.q.y) < 0.001) deriv.q.y = 0.0;
  if (abs(deriv.q.z) < 0.001) deriv.q.z = 0.0;
  if (abs(deriv.q.w) < 0.001) deriv.q.w = 0.0;
  
  return deriv;
}

void Model::update(const ros::TimerEvent& e)
{
  ros::Time this_time = ros::Time::now();
  float dt = (this_time-time_).toSec();
  State deriv = Model::get_trajectory(state_, input_);
  time_ = this_time;

  // Integration
  state_.p.x += deriv.p.x * dt;
  state_.p.y += deriv.p.y * dt;
  state_.p.z += deriv.p.z * dt;
  state_.v.x += deriv.v.x * dt;
  state_.v.y += deriv.v.y * dt;
  state_.v.z += deriv.v.z * dt;
  state_.q.x += deriv.q.x * dt;
  state_.q.y += deriv.q.y * dt;
  state_.q.z += deriv.q.z * dt;
  state_.q.w += deriv.q.w * dt;
  state_.w.x += deriv.w.x * dt;
  state_.w.y += deriv.w.y * dt;
  state_.w.z += deriv.w.z * dt;

  // Weak contact mechanics. TODO: Make sure motors don't go through the ground either.
  if (state_.p.z < 0.0)
  {
      state_.p.z = 0.0;
      if (state_.v.z < 0.0) state_.v.z = 0.0;
  }

  // Normalize quaternion
  float norm2 = state_.q.x*state_.q.x + state_.q.y*state_.q.y
    + state_.q.z*state_.q.z + state_.q.w*state_.q.w;
  ROS_ASSERT_MSG(norm2 > 0.001, "Unable to normalize quaternion.  ||q||^2 = %f", norm2);
  float inv_norm = 1.0 / sqrt(norm2);
  state_.q.x *= inv_norm;
  state_.q.y *= inv_norm;
  state_.q.z *= inv_norm;
  state_.q.w *= inv_norm;

  // Set true IMU Values
  float bx_nz = 2.0*state_.q.x*state_.q.z + 2.0*state_.q.y*state_.q.w;
  float by_nz = - 2.0*state_.q.x*state_.q.w + 2.0*state_.q.y*state_.q.z;
  float bz_nz = - state_.q.x*state_.q.x - state_.q.y*state_.q.y
    + state_.q.z*state_.q.z + state_.q.w*state_.q.w;
  accel_.x = deriv.v.x + k_gravity_ * bx_nz;
  accel_.y = deriv.v.y + k_gravity_ * by_nz;
  accel_.z = deriv.v.z + k_gravity_ * bz_nz;
  gyro_ = state_.w;

  // Publish new pose.
  geometry_msgs::Pose new_pose;
  new_pose.position.x = state_.p.x;
  new_pose.position.y = state_.p.y;
  new_pose.position.z = state_.p.z;
  new_pose.orientation = state_.q;
  pose_pub_.publish(new_pose);
}
} // namespace quadcopter
