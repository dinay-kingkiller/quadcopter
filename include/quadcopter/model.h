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

/// \file model.h
/// \brief A ROS node that models a quadcopter.
///

#ifndef QUADCOPTER_MODEL_H_
#define QUADCOPTER_MODEL_H_

#include "ros/ros.h"

#include "quadcopter/FullState.h"
#include "quadcopter/IMU.h"
#include "quadcopter/Input.h"
#include "quadcopter/State.h"


namespace quadcopter {

/// \class quadcopter::Model
/// \brief
class Model {
private:
  quadcopter::FullState last_state;
  quadcopter::Input last_input;
  ros::Time last_time;
  float mass;
  float arm_length;
  float k_gravity;
  float k_torque;
  float k_force;
  quadcopter::FullState ode(quadcopter::FullState state, quadcopter::Input input);
  quadcopter::Input add_motor_noise(quadcopter::Input clean);
  quadcopter::IMU add_sensor_noise(quadcopter::IMU clean);
public:
  Model(float Mass, float ArmLength, float GAccel, float kTorque, float kForce,
        quadcopter::FullState initial_state);
  void update();
};
} //namespace quadcopter

#endif // QUADCOPTER_MODEL_H_
