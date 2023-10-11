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

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

#include "quadcopter/Motor.h"
#include "quadcopter/Sensor.h"

namespace quadcopter
{

/// \brief Provides a succinct way to pass the state to and from the ODE and solver
struct State
{
  geometry_msgs::Vector3 p; /// Position in inertial frame
  geometry_msgs::Vector3 v; /// Velocity in inertial frame
  geometry_msgs::Quaternion q; /// Rotation from inertial frame
  geometry_msgs::Vector3 w; /// Rotation from inertial frame in robot frame
};

/// \brief Models the physics of a quadcopter and provides sensor feedback.
class Model
{
public:
  Model(ros::NodeHandle node);
  /// \brief Places the quadcopter back at the origin and restarts the clock.
  void zero();
  /// \brief Set model parameters taken from ROS node_
  void reset_params();
  /// \brief Returns the trajectory of the state and input arguments.
  State get_trajectory(State state, Motor input) const;
  /// \brief Timer callback that updates the model and publishes the pose.
  void update(const ros::TimerEvent& e);
  /// \brief Sets the motor values to a new input.
  void move(Motor input);
  /// \brief Measures and publishes the current state of the sensors.
  void sense(const ros::TimerEvent& e);
private:
  /// \brief node instance for communicating with ROS
  ros::NodeHandle node_;
  /// \brief publisher for sensor messages
  ros::Publisher sensor_pub_;
  /// \brief publisher for current pose of the quadcopter
  ros::Publisher pose_pub_;
  /// \brief mass of the quadcopter. Model assumes all mass is in the motors.
  float mass_;
  /// \brief length of each arm. Model assumes all mass is in the motors.
  float radius_;
  /// \brief graviational acceleration constant
  float k_gravity_;
  /// \brief force constant of motors
  float k_force_;
  /// \brief torque constant of motors
  float k_torque_; 
  /// \brief time the state was last updated
  ros::Time time_;
  /// \brief last updated state
  State state_;
  /// \brief last motor input given
  Motor input_;
  /// \brief Acceleration in the inertial frame for adding sensor noise.
  geometry_msgs::Vector3 accel_;
  /// \brief Gyro values in the robot frame?
  geometry_msgs::Vector3 gyro_;
};
} //namespace quadcopter

#endif // QUADCOPTER_MODEL_H_
