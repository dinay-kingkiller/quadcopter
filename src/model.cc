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

#include "quadcopter/Motor.h"
#include "quadcopter/Sensor.h"

namespace quadcopter
{
Model::Model(float Mass, float ArmLength, float GAccel, float kForce, float kTorque)
  : mass_(Mass),
    arm_length_(ArmLength),
    k_gravity_(GAccel),
    k_torque_(kTorque),
    k_force_(kForce)
{
  Model::zero();
}
Sensor Model::sense()
{
  /// TODO: Add robot acceleration
  Sensor sensor;
  sensor.accelerometer.x = 0.0;
  sensor.accelerometer.y = 0.0;
  sensor.accelerometer.z = -k_gravity_;
  sensor.gyroscope.x = state_.spin[0];
  sensor.gyroscope.y = state_.spin[1];
  sensor.gyroscope.z = state_.spin[2];
  return sensor;
}
void Model::zero()
{
  time_ = ros::Time::now();
  state_ = quadcopter::ZERO_STATE;
  input_.front = 0.0;
  input_.right = 0.0;
  input_.back = 0.0;
  input_.left = 0.0;
}
void Model::move(Motor input) {input_ = input;}
State Model::get_trajectory() const
{
  State deriv;
  deriv.pos = state_.vel;
  deriv.ori[0] = -state_.spin[0]*state_.ori[1];
  deriv.ori[0] -= state_.spin[1]*state_.ori[2];
  deriv.ori[0] -= state_.spin[2]*state_.ori[3];
  
  deriv.ori[1] =  state_.spin[0]*state_.ori[0];
  deriv.ori[1] += state_.spin[1]*state_.ori[3];
  deriv.ori[1] -= state_.spin[2]*state_.ori[2];
  
  deriv.ori[2] = -state_.spin[0]*state_.ori[3];
  deriv.ori[2] += state_.spin[1]*state_.ori[0];
  deriv.ori[2] += state_.spin[2]*state_.ori[1];
  
  deriv.ori[3] =  state_.spin[0]*state_.ori[2];
  deriv.ori[3] -= state_.spin[1]*state_.ori[1];
  deriv.ori[3] += state_.spin[2]*state_.ori[0];

  float thrust = input_.front*input_.front;
  thrust += input_.right*input_.right;
  thrust += input_.back*input_.back;
  thrust += input_.left*input_.left;

  deriv.pos[0] =  2 * state_.ori[0] * state_.ori[3];
  deriv.pos[0] += 2 * state_.ori[1] * state_.ori[2];
  deriv.pos[0] *= k_force_ * thrust / mass_;

  deriv.pos[1] = state_.ori[0] * state_.ori[0];
  deriv.pos[1] -= state_.ori[1] * state_.ori[1];
  deriv.pos[1] -= state_.ori[2] * state_.ori[2];
  deriv.pos[1] -= state_.ori[3] * state_.ori[3];
  deriv.pos[1] *= k_force_* thrust / mass_;

  deriv.pos[2] = 2 * state_.ori[0] * state_.ori[1];
  deriv.pos[2] += 2 * state_.ori[2] * state_.ori[3];
  deriv.pos[2] *= k_force_ * thrust / mass_;
  deriv.pos[2] -= k_gravity_;
  
  float r_torque = input_.right*input_.right;
  r_torque -= input_.left*input_.left;
  float p_torque = input_.back*input_.back;
  p_torque -= input_.front*input_.front;
  float y_torque = input_.front*input_.front;
  y_torque += input_.back*input_.back;
  y_torque -= input_.left*input_.left;
  y_torque -= input_.right*input_.right;
  float k_moment = k_force_ / mass_ / arm_length_;
  
  deriv.spin[0] = -state_.spin[1] * state_.spin[2];
  deriv.spin[0] += 2 * k_moment * r_torque;

  deriv.spin[1] = state_.spin[0]*state_.spin[2];
  deriv.spin[1] += 2 * k_moment * p_torque;
  
  deriv.spin[2] = k_moment * y_torque;
  return deriv;
}
geometry_msgs::Pose Model::update()
{
  ros::Time this_time = ros::Time::now();
  float dt = this_time.toSec() - time_.toSec();
  State deriv = Model::get_trajectory();
  time_ = this_time;
}
} // namespace quadcopter
