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
  State deriv = Model::ODE(last_state_, last_input_);
  // Update the state since last sense.
  last_state_ = Model::integrateState(last_state_, deriv, ros::Time::now());
  return Model::measureState(last_state_);
}
void Model::zero()
{
  last_state_.t = ros::Time::now();
  last_state_= State;
  last_input_.front = 0.0;
  last_input_.right = 0.0;
  last_input_.back = 0.0;
  last_input_.left = 0.0;
}
void Model::move(Motor input) {input_ = input;}
State Model::ODE() const
{
  State deriv;
  deriv.pos = state_.vel;
  deriv.ori[0] = -state_.spin[0]*state.ori[1] - state.spin[1]*state.ori[2] - state.spin[2]*state.ori[3];
  deriv.ori[1] =  state.spin[0]*state.ori[0] + state.spin[1]*state.ori[3] - state.spin[2]*state.ori[2];
  deriv.ori[2] = -state.spin[0]*state.ori[3] + state.spin[1]*state.ori[0] + state.spin[2]*state.ori[1];
  deriv.ori[3] =  state.spin[0]*state.ori[2] - state.spin[1]*state.ori[1] + state.spin[2]*state.ori[0]
  float input2 = input.front * input.front + input.right * input.right
    + input.back * input.back + input.left * input.left;
  deriv.pos[0] = k_force_ / mass_ * sin(state.a) * input2;
  deriv.pos[1] = - k_force_ / mass_ * cos(state.a) * sin(state.b) * input2;
  deriv.pos[2] = k_force_ / mass_ * cos(state.a) * cos(state.b) * input2 - k_gravity_;
  // The conservation of angular momentum equation is more complicated
  float m_inertia = mass_ * arm_length_ * arm_length_;
  float f0 = m_inertia * state.diff_b * state.diff_c * cos(state.c) * cos(state.a) / 2.0
    + m_inertia * state.diff_a * state.diff_b * sin(state.a) * cos(state.c) / 2.0
    - m_inertia * state.diff_a * state.diff_c * cos(state.c) * cos(state.c) / 2.0
    + k_force_ * arm_length_ * (input.right*input.right - input.left*input.left);
  float f1 = m_inertia * state.diff_a * state.diff_b * sin(state.a) * sin(state.c) / 2.0
    - m_inertia * state.diff_b * state.diff_c * cos(state.a) * cos(state.c) / 2.0
    + m_inertia * state.diff_a * state.diff_c * sin(state.c) / 2.0
    + k_force_ * arm_length_ * (input.back*input.back - input.front*input.front);
  float yaw_torque = input.front*input.front + input.back*input.back
    - input.left*input.left - input.right*input.right;
  float f2 = k_torque_ * yaw_torque - m_inertia * state.diff_a * state.diff_b * cos(state.a);
  float mm00 = m_inertia * cos(state.a) * cos(state.c) / 2.0;
  float mm01 = -m_inertia * cos(state.a) * sin(state.c) / 2.0;
  float mm02 = m_inertia * sin(state.a);
  float mm10 = m_inertia * sin(state.c) / 2.0;
  float mm11 = m_inertia * cos(state.c) / 2.0;
  // check for division by zero
  ROS_ASSERT_MSG(mm01 != 0.0, "Division by zero. mm01 = %f", mm01);
  ROS_ASSERT_MSG(mm10 != 0.0, "Division by zero. mm10 = %f", mm10);
  float inv_det = - 1.0 / (mm01 * mm10);
  deriv.spin[0] = inv_det * (f0*mm11 - f1*mm01 - f2*mm02*mm11);
  deriv.spin[1] = inv_det * (-f0*mm10 + f1*mm00 + f2*mm02*mm10);
  deriv.spin[2] = inv_det * f2 * (mm00*mm11-mm01*mm10);
  return deriv;
}
Sensor Model::measureState(const FullState state) const
{
  Sensor sensor;
  sensor.accel_x = (1+state.ddiff_x) * sin(state.a)
    * sin(state.b) * cos(state.c)
    - cos(state.a) * sin(state.c);
  sensor.accel_y = (1+state.ddiff_y) * sin(state.a)
    * sin(state.b) * sin(state.c)
    + cos(state.a) * cos(state.c);
  sensor.accel_z = (1+state.ddiff_z) * sin(state.b) * cos(state.a);
  sensor.gyro_x = state.diff_b * cos(state.a) * cos(state.c)
    + state.diff_a * cos(state.b) * cos(state.c);
  sensor.gyro_y = state.diff_b * cos(state.a) * cos(state.c)
    + state.diff_a * cos(state.c);
  sensor.gyro_z = - state.diff_b * sin(state.a)
    + state.diff_a * sin(state.a) * sin(state.c)
    + state.diff_c;
  return sensor;
}  
} // namespace quadcopter
