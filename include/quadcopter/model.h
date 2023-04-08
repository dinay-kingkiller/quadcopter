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

/// \brief A model to simulate quadcopter physics
///
/// Models the equations of motion, taking in motor inputs and sending out sensor outputs.

#ifndef QUADCOPTER_MODEL_H_
#define QUADCOPTER_MODEL_H_

#include "ros/ros.h"

#include "quadcopter/Motor.h"
#include "quadcopter/Sensor.h"
#include "geometry_msgs/Pose.h"


namespace quadcopter
{

/// \brief Provides a succinct way to pass the state to and from the ODE and solver
struct State {
  std::array<float, 3> pos = {{0.0, 0.0, 0.0}};
  std::array<float, 3> vel = {{0.0, 0.0, 0.0}};
  std::array<float, 4> ori = {{1.0, 0.0, 0.0, 0.0}};
  std::array<float, 3> spin = {{0.0, 0.0, 0.0}};
};

/// \brief A State instance for resetting to defaults.
static const struct State DEFAULT_STATE;
  
/// \brief Models the physics of a quadcopter and provides sensor feedback.
class Model
{
public:
  Model(float Mass, float ArmLength, float GAccel, float kForce, float kTorque);
  /// \brief Places the quadcopter back at the origin and restarts the clock.
  void zero();
  /// \brief Returns the trajectory of the quadcopter's current state.
  State get_trajectory() const;
  /// \brief Updates the model and returns the quadcopter pose.
  geometry_msgs::Pose update();
  /// \brief Sets the motor values to a new input.
  void move(Motor input);
  /// \brief Measures and returns the current state of the sensors.
  Sensor sense();
private:
  const float mass_;
  const float radius_;
  const float k_gravity_; 
  const float k_torque_; /// torque constant of motors
  const float k_force_; /// force constant of motors
  ros::Time time_;
  std::array<float, 3> accel_;
  State state_;
  Motor input_;
};
} //namespace quadcopter

#endif // QUADCOPTER_MODEL_H_
