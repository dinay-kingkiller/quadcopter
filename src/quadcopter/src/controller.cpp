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

#include "quadcopter/controller.h"

#include <string>

#include "ros/ros.h"

#include "quadcopter/Motor.h"

namespace quadcopter
{

ConstantController::ConstantController(ros::NodeHandle node)
: node_(node)
{
  set_msg();
  motor_pub_ = node.advertise<quadcopter::Motor>("motor_input", 1000);
}

void ConstantController::publish_input(const ros::TimerEvent& e)
{
  motor_pub_.publish(motor_msg_);
}

void ConstantController::set_msg()
{
  float mass, k_gravity, k_force, k_torque;
  node_.getParam("Mass", mass);
  node_.getParam("GAccel", k_gravity);
  node_.getParam("kForce", k_force);
  node_.getParam("kTorque", k_torque);
  ROS_ASSERT_MSG(k_torque > 0.001, "Torque Constant set too low.");
  float motor_balance = 0.5 * sqrt(k_gravity/k_force);

  std::string motor_config;
  node_.getParam("motor_config", motor_config);

  if (motor_config == "zero") {
    motor_msg_.front = 0.0;
    motor_msg_.right = 0.0;
    motor_msg_.back = 0.0;
    motor_msg_.left = 0.0;
  }
  else if (motor_config == "high") {
    motor_msg_.front = 2.0 * motor_balance;
    motor_msg_.right = 2.0 * motor_balance;
    motor_msg_.back = 2.0 * motor_balance;
    motor_msg_.left = 2.0 * motor_balance;
  }
  else if (motor_config == "low") {
    motor_msg_.front = 0.5 * motor_balance;
    motor_msg_.right = 0.5 * motor_balance;
    motor_msg_.back = 0.5 * motor_balance;
    motor_msg_.left = 0.5 * motor_balance;
  }
  else if (motor_config == "balanced") {
    motor_msg_.front = motor_balance;
    motor_msg_.right = motor_balance;
    motor_msg_.back = motor_balance;
    motor_msg_.left = motor_balance;
  }
  else if (motor_config == "roll") {
    motor_msg_.front = 0.0;
    motor_msg_.right = 0.75 * motor_balance;
    motor_msg_.back = 0.0;
    motor_msg_.left = 0.25 * motor_balance;
  }
  else if (motor_config == "pitch") {
    motor_msg_.front = 0.75 * motor_balance;
    motor_msg_.right = 0.0;
    motor_msg_.back =  0.25 * motor_balance;
    motor_msg_.left = 0.0;
  }
  else if (motor_config == "yaw") {
    motor_msg_.front = 0.375 * motor_balance;
    motor_msg_.right = 0.125 * motor_balance;
    motor_msg_.back = 0.375 * motor_balance;
    motor_msg_.left = 0.125 * motor_balance;
  }
  else {
    ROS_ERROR_STREAM("motor_config: "<<motor_config);
    ROS_ERROR("Invalid argument sent to controller_constant");
  }
} // ConstantController::set_msg

} // namespace quadcopter
