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

/// \file model.cc
/// \brief A ROS node that models a quadcopter.
///

#include "quadcopter/model.h"

#include "ros/ros.h"

#include "quadcopter/FullState.h"
#include "quadcopter/IMU.h"
#include "quadcopter/Input.h"
#include "quadcopter/State.h"

namespace quadcopter {

FullState Model::ode(FullState state, Input input) {
  FullState deriv;
  float input2 = input.front * input.front
    + input.right * input.right
    + input.back * input.back
    + input.left * input.left;
  deriv.x = state.diff_x;
  deriv.y = state.diff_y;
  deriv.z = state.diff_z;
  deriv.a = state.diff_a;
  deriv.b = state.diff_b;
  deriv.c = state.diff_c;
  deriv.diff_x = k_force / mass * sin(state.a) * input2;
  deriv.diff_y = - k_force / mass * cos(state.a) * sin(state.b) * input2;
  deriv.diff_z = k_force / mass * cos(state.a) * cos(state.b) * input2 - k_gravity;
  // The conservation of angular momentum equation is more complicated
  float m_inertia = mass * arm_length * arm_length;
  float f0 = m_inertia * state.diff_b * state.diff_c * cos(state.c) * cos(state.a) / 2.0
    + m_inertia * state.diff_a * state.diff_b * sin(state.a) * cos(state.c) / 2.0
    - m_inertia * state.diff_a * state.diff_c * cos(state.c) * cos(state.c) / 2.0
    + k_force * arm_length * (input.right*input.right - input.left*input.left);
  float f1 = m_inertia * state.diff_a * state.diff_b * sin(state.a) * sin(state.c) / 2.0
    - m_inertia * state.diff_b * state.diff_c * cos(state.a) * cos(state.c) / 2.0
    + m_inertia * state.diff_a * state.diff_c * sin(state.c) / 2.0
    + k_force * arm_length * (input.back*input.back - input.front*input.front);
  float yaw_torque = input.front*input.front + input.back*input.back
    - input.left*input.left - input.right*input.right;
  float f2 = k_torque * yaw_torque - m_inertia * state.diff_a * state.diff_b * cos(state.a);
  float mm00 = m_inertia * cos(state.a) * cos(state.c) / 2.0;
  float mm01 = -m_inertia * cos(state.a) * sin(state.c) / 2.0;
  float mm02 = m_inertia * sin(state.a);
  float mm10 = m_inertia * sin(state.c) / 2.0;
  float mm11 = m_inertia * cos(state.c) / 2.0;
  assert(mm01!=0.0 and mm10!=0.0);
  float inv_det = - 1.0 / (mm01 * mm10);
  deriv.a = inv_det * (f0*mm11 - f1*mm01 - f2*mm02*mm11);
  deriv.b = inv_det * (-f0*mm10 + f1*mm00 + f2*mm02*mm10);
  deriv.c = inv_det * f2 * (mm00*mm11-mm01*mm10);
  return deriv;
}

Input Model::add_motor_noise(quadcopter::Input clean) {
  return clean;
}

IMU Model::add_sensor_noise(quadcopter::IMU clean) {
  return clean;
}


Model::Model(float Mass, float ArmLength, float GAccel, float kTorque, float kForce,
      quadcopter::FullState initial_state)
  : last_state(initial_state),
    mass(Mass),
    arm_length(ArmLength),
    k_gravity(GAccel),
    k_torque(kTorque),
    k_force(kForce) {
  last_time = ros::Time::now();
  last_input.front = 0.0;
  last_input.right = 0.0;
  last_input.back = 0.0;
  last_input.left = 0.0;
}


void Model::update() {
  ros::Time next_time = ros::Time::now();
  float dt = next_time.toSec() - last_time.toSec();
  quadcopter::FullState deriv = ode(last_state, last_input);
  last_state.x = deriv.x * dt;
  last_state.y = deriv.y * dt;
  last_state.z = deriv.z * dt;
  last_state.a = deriv.a * dt;
  last_state.b = deriv.b * dt;
  last_state.c = deriv.c * dt;
  last_state.diff_x = deriv.diff_x * dt;
  last_state.diff_y = deriv.diff_y * dt;
  last_state.diff_z = deriv.diff_z * dt;
  last_state.diff_a = deriv.diff_a * dt;
  last_state.diff_b = deriv.diff_b * dt;
  last_state.diff_c = deriv.diff_c * dt;
}
} // namespace quadcopter
