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

#include <string>
#include "ros/ros.h"
#include "quadcopter/Motor.h"
#include "quadcopter/controller.h"

namespace quadcopter
{

Controller::Controller(
  ros::Publisher motor_pub,
  std::string motor_config,
  float motor_balance)
: motor_pub_(motor_pub)
{
  set_msg(motor_config, motor_balance);
}

void quadcopter::Controller::publish_input(const ros::TimerEvent& e)
{
  motor_pub_.publish(motor_msg_);
}

bool Controller::set_msg(std::string motor_config, float motor_balance)
{
  /// TODO: This validation needs to be pulled out of the constructor.
  if (motor_config == "zero") {
    motor_msg_.front = 0.0;
    motor_msg_.right = 0.0;
    motor_msg_.back = 0.0;
    motor_msg_.left = 0.0;
    return true;
  }
  else if (motor_config == "high") {
    motor_msg_.front = 2.0 * motor_balance;
    motor_msg_.right = 2.0 * motor_balance;
    motor_msg_.back = 2.0 * motor_balance;
    motor_msg_.left = 2.0 * motor_balance;
    return true;
  }
  else if (motor_config == "low") {
    motor_msg_.front = 0.5 * motor_balance;
    motor_msg_.right = 0.5 * motor_balance;
    motor_msg_.back = 0.5 * motor_balance;
    motor_msg_.left = 0.5 * motor_balance;
    return true;
  }
  else if (motor_config == "balanced") {
    motor_msg_.front = motor_balance;
    motor_msg_.right = motor_balance;
    motor_msg_.back = motor_balance;
    motor_msg_.left = motor_balance;
    return true;
  }
  else if (motor_config == "roll") {
    motor_msg_.front = 0.0;
    motor_msg_.right = 0.75 * motor_balance;
    motor_msg_.back = 0.0;
    motor_msg_.left = 0.25 * motor_balance;
    return true;
  }
  else if (motor_config == "pitch") {
    motor_msg_.front = 0.75 * motor_balance;
    motor_msg_.right = 0.0;
    motor_msg_.back =  0.25 * motor_balance;
    motor_msg_.left = 0.0;
    return true;
  }
  else if (motor_config == "yaw") {
    motor_msg_.front = 0.375 * motor_balance;
    motor_msg_.right = 0.125 * motor_balance;
    motor_msg_.back = 0.375 * motor_balance;
    motor_msg_.left = 0.125 * motor_balance;
    return true;
  }
  else {
    ROS_ERROR_STREAM("motor_config: "<<motor_config);
    ROS_ERROR("Invalid argument sent to controller_constant");
    return false;
  }
} // Controller::set_msg
