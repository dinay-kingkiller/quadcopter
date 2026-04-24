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

#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#include "geometry_msgs/Pose.h"

#include "quad_msgs/Motor.h"
#include "quadcopter/types.h"

namespace quadcopter
{
/**
 * @brief Convert a ROS Motor message to the internal quadcopter::Motor struct.
 *
 * This function copies the fields from a ROS Motor message into
 * the internal Motor representation used by the quadcopter code.
 *
 * @param msg The ROS Motor message to convert.
 * @return Motor The internal Motor struct populated from the message.
 */
Motor toStruct(const quad_msgs::Motor& msg);

/**
 * @brief Convert the internal quadcopter::Motor struct to a ROS Motor message.
 *
 * This function copies the fields from the internal Motor struct
 * into a ROS Motor message for publishing.
 *
 * @param m The internal Motor struct to convert.
 * @return quad_msgs::Motor The ROS message populated from the struct.
 */
quad_msgs::Motor toMsg(const Motor& m);

/**
 * @brief Convert the internal quadcopter::State struct to a ROS Pose message.
 *
 * This function maps position and orientation information from the
 * internal State representation to a geometry_msgs::Pose message.
 *
 * @param s The internal State struct to convert.
 * @return geometry_msgs::Pose The ROS Pose message representing the state.
 */
geometry_msgs::Pose toMsg(const State& s);

} // namespace quadcopter

#endif // CONVERSIONS_H_
