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

/// \file model_node.cc
/// \brief A ROS node that models a quadcopter.


#include "ros/ros.h"

#include "quadcopter/model.h"

#include "quadcopter/Sensor.h"
#include "quadcopter/Motor.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "model");
  ros::NodeHandle node;
  float Mass, ArmLength, kGravity, kTorque, kForce;
  node.getParam("Mass", Mass);
  node.getParam("ArmLength", ArmLength);
  node.getParam("kGravity", kGravity);
  node.getParam("kForce", kForce);
  node.getParam("kTorque", kTorque);
  quadcopter::Model drone(Mass, ArmLength, kGravity, kTorque, kForce);
  ros::Rate sensor_rate(1000);
  ros::Publisher sensor_pub = node.advertise<quadcopter::Sensor>
    ("sensor_output", 1000);
  ros::Subscriber motor_sub = node.subscribe<quadcopter::Motor>
    ("motor_input", 1000, &quadcopter::Model::move, &drone);
  while (ros::ok())
  {
    quadcopter::Sensor sensor_msg = drone.sense();
    sensor_pub.publish(sensor_msg);
    sensor_rate.sleep();
  }  
  return 0;
}