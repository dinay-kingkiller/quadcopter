// BSD 3-Clause License
//
// Copyright (c) 2023, Dinay Kingkiller
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

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose.h"

#include "quadcopter/Motor.h"
#include "quadcopter/Sensor.h"

namespace quadcopter
{
Model::Model(ros::NodeHandle node)
: node_(node)
{
  Model::reset_params();
  Model::zero();
  pose_pub_ = node.advertise<geometry_msgs::Pose>("pose", 1000);
  sensor_pub_ = node.advertise<quadcopter::Sensor>("imu", 1000);
}
void Model::sense(const ros::TimerEvent& e)
{
  Sensor sensor;
  sensor.accelerometer.x = accel_;
  sensor.gyroscope = gyro_;

  sensor_pub_.publish(sensor);
}
void Model::reset_params()
{
  node_.getParam("Mass", mass_);
  node_.getParam("Radius", radius_);
  node_.getParam("GAccel", k_gravity_);
  node_.getParam("kForce", k_force_);
  node_.getParam("kTorque", k_torque_);
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

  // Quaternion rotation derivative
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
  float thrust = k_force_ / mass_ * input.front * input.front
    + k_force_ / mass_ * input.right * input.right
    + k_force_ / mass_ * input.back * input.back
    + k_force_ / mass_ * input.left * input.left;

  // Calculate the various reference frame accelerations
  geometry_msgs::Vector3 euler;
  euler.x = state.p.z * deriv.w.y - state.p.y * deriv.w.z;
  euler.y = state.p.x * deriv.w.z - state.p.z * deriv.w.x;
  euler.z = state.p.y * deriv.w.x - state.p.x * deriv.w.y;
  
  geometry_msgs::Vector3 coriolis;
  coriolis.x = 2.0*state.w.y*state.v.z - 2.0*state.w.z*state.v.y;
  coriolis.y = 2.0*state.w.z*state.v.x - 2.0*state.w.x*state.v.z;
  coriolis.z = 2.0*state.w.x*state.v.y - 2.0*state.w.y*state.v.x;

  geometry_msgs::Vector3 centrifugal;
  centrifugal.x = state.w.x * state.w.y * state.p.y
    + state.w.x * state.w.z * state.p.z
    - state.w.y * state.w.y * state.w.x
    - state.w.z * state.w.z * state.w.x;

  centrifugal.y = state.w.x * state.w.y * state.p.x
    + state.w.y * state.w.z * state.p.z
    - state.w.x * state.w.x * state.p.y
    - state.w.z * state.w.z * state.p.y;

  centrifugal.z = state.w.x * state.w.z * state.p.x
    + state.w.y * state.w.z * state.p.y
    - state.w.x * state.w.x * state.p.z
    - state.w.y * state.w.y * state.p.z;

  geometry_msgs::Vector3 gravity;
  gravity.x = 2.0*k_gravity_*(state.q.x*state.q.z + state.q.y*state.q.w);
  gravity.y = 2.0*k_gravity_*(state.q.y*state.q.z - state.q.x*state.q.w);
  gravity.z = k_gravity_*(state.q.z*state.q.z + state.q.w*state.q.w
			  -state.q.x*state.q.x - state.q.y*state.q.y);

  // Then combine them to find the velocity trajectory
  deriv.v.x = - euler.x - centrifugal.x - coriolis.x - gravity.x;
  deriv.v.y = - euler.y - centrifugal.y - coriolis.y - gravity.y;
  deriv.v.z = thrust - euler.z - centrifugal.z - coriolis.z - gravity.z;
  
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

  // Normalize quaternion
  float norm2 = state_.q.x*state_.q.x + state_.q.y*state_.q.y
    + state_.q.z*state_.q.z + state_.q.w*state_.q.w;
  ROS_ASSERT_MSG(norm2 > 0.001, "Unable to normalize quaternion.  ||q||^2 = %f", norm2);
  float inv_norm = 1.0 / sqrt(norm2);
  state_.q.x *= inv_norm;
  state_.q.y *= inv_norm;
  state_.q.z *= inv_norm;
  state_.q.w *= inv_norm;

  // IMU Values
  accel_ = deriv.v;
  gyro_ = state_.w;

  // Publish new pose.
  geometry_msgs::Pose new_pose;
  new_pose.position.x = state_.p.x; // convert Vector3 to Point
  new_pose.position.y = state_.p.y;
  new_pose.position.z = state_.p.z;
  new_pose.orientation = state_.q;
  pose_pub_.publish(new_pose);
}
} // namespace quadcopter
