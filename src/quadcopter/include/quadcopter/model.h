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

/// \file model.h
/// \brief Defines the quadcopter physics model, including motor input handling and sensor output.

#ifndef QUADCOPTER_MODEL_H_
#define QUADCOPTER_MODEL_H_

#include <random>

#include "ros/ros.h"

#include "quadcopter/types.h"
#include "quad_msgs/Motor.h"

namespace quadcopter
{
/// \brief Models the physics of a quadcopter and provides sensor feedback.
/// 
/// This class implements a simple physics model for a quadcopter, integrating motor inputs
/// over time to produce position, orientation, and sensor outputs. It can be used for testing
/// controllers and simulating flight in a ROS environment.
class Model
{
public:
  /// \brief Constructs a Model instance with a ROS node handle.
  /// \param node A ROS node handle for publishers/subscribers.
  Model(ros::NodeHandle node);

  /// \brief Resets the quadcopter to the origin and zeroes its state.
  ///
  /// This method places the quadcopter at (0,0,0) with no linear or angular velocity.
  void zero();

  /// \brief Loads or resets model parameters from the ROS parameter server.
  ///
  /// Reads parameters such as mass, gravity constant, force/torque constants, and sensor scales.
  void reset_params();

  /// \brief Timer callback to update the physics model.
  ///
  /// Integrates motor inputs over the time step and updates the quadcopter's pose and sensor outputs.
  /// \param e Timer event information from ROS (contains dt).
  void update(const ros::TimerEvent& e);

  /// \brief Updates the current motor inputs.
  ///
  /// Receives a ROS `quad_msgs::Motor` message and updates the internal motor state.
  /// \param msg Pointer to the motor message received from a ROS topic.
  void move(const quad_msgs::Motor::ConstPtr& msg);

  /// \brief Measures and publishes the current sensor outputs.
  ///
  /// Uses internal state plus noise to simulate sensor readings.
  /// Currently commented out; implement if sensor output is needed.
  // void sense(const ros::TimerEvent& e);

private:
  /// \brief ROS node handle used for publishers and subscribers.
  ros::NodeHandle node_;

  /// \brief Publisher for simulated sensor messages.
  ros::Publisher sensor_pub_;

  /// \brief Publisher for the current quadcopter pose.
  ros::Publisher pose_pub_;

  /// \brief Time the state was last updated.
  ros::Time time_;

  /// \brief Current state of the quadcopter (position, orientation, linear and angular velocities).
  State state_;

  /// \brief Last motor inputs applied to the quadcopter.
  Motor input_;

  /// \brief Physical model parameters (mass, gravity, force/torque constants, etc.).
  Params params_;

  /// \brief Random number generator for sensor noise simulation.
  std::mt19937 rand_gen_;

  /// \brief Gaussian noise distribution for accelerometer readings.
  std::normal_distribution<double> accel_noise_;

  /// \brief Gaussian noise distribution for gyroscope readings.
  std::normal_distribution<double> gyro_noise_;

  /// \brief Maximum scale for gyroscope readings.
  double gyro_scale_;

  /// \brief Maximum scale for accelerometer readings.
  double accel_scale_;
};
} //namespace quadcopter

#endif // QUADCOPTER_MODEL_H_
