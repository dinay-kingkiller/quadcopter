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

#include "ros/ros.h"

#include "quadcopter/Motor.h"

namespace Quadcopter
{

class Planner
{
public:
  Planner (ros::Publisher motor_pub,
              Quadcopter::Motor motor_msg_)
    : motor_pub_(motor_pub),
      motor_msg_(motor_msg) {}
  publishInput(const ros::TimerEvent& e) {
    motor_pub_.publish(motor_msg_);
  }
private:
  ros::Publisher motor_pub_;
  Quadcopter::Motor motor_msg_;
}

int main(int argc, char argv**)
{
  if (argc != 1)
    {
      ROS_ERROR("test_controller requires one argument, received %d", x);
      return 1;
    }

  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;

  float mass, k_gravity, k_torque;
  nh.getParam("Mass", mass);
  nh.getParam("GAccel", k_gravity);
  nh.getParam("kTorque", k_torque);
  ROS_ASSERT_MSG(k_torque > 0.001, "Torque Constant set too low.");
  float balance = 0.5*sqrt(mass*k_gravity/k_torque);

  Quadcopter::Motor motor_input;
  if (argv[0] == "none")
    {
      motor_input.front = 0.0;
      motor_input.right = 0.0;
      motor_input.rear = 0.0;
      motor_input.left = 0.0;
    }
  else if (argv[0] == "high")
    {
      motor_input.front = 2.0 * balance;
      motor_input.right = 2.0 * balance;
      motor_input.rear = 2.0 * balance;
      motor_input.left = 2.0 * balance;
    }
  else if (argv[0] == "low")
    {
      motor_input.front = 0.5 * balance;
      motor_input.right = 0.5 * balance;
      motor_input.rear = 0.5 * balance;
      motor_input.left = 0.5 * balance;
    }
  else if (argv[0] == "balanced")
    {
      motor_input.front = balance;
      motor_input.right = balance;
      motor_input.rear = balance;
      motor_input.left = balance;
    }
  else if (argv[0] == "roll")
    {
      motor_input.front = 0.0;
      motor_input.right = 0.75 * balance;
      motor_input.rear = 0.0;
      motor_input.left = 0.25 * balance;
    }
  else if (argv[0] == "pitch")
    {
      motor_input.front = 0.75 * balance;
      motor_input.right = 0.0;
      motor_input.rear =  0.25 * balance;
      motor_input.left = 0.0;
    }
  else if (argv[0] == "yaw")
    {
      motor_input.front = 0.375 * balance;
      motor_input.right = 0.125 * balance;
      motor_input.rear = 0.375 * balance;
      motor_input.left = 0.125 * balance;
    }
  else
    {
      ROS_ERROR("Invalid argument sent to test_constant");
      return 1;
    }


  ros::Publisher motor_pub = nh.advertise<Quadcopter::Motor>("motor_input", 1000);
  Planner planner(motor_pub, motor_input);
  ros::Timer motor_timer = nh.createTimer(ros::Rate(1000), &Controller::publishInput, &motor_controller);

  ros::spin();

  return 0;
}
} // namespace Quadcopter
