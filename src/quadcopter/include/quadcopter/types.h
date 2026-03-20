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

/**
 * @file types.h
 * @brief Basic data types for quadcopter state, motors, and parameters
 */

#ifndef QUADCOPTER_TYPES_H_
#define QUADCOPTER_TYPES_H_

namespace quadcopter
{
/** 3D vector. */
struct Vector3
{
  double x, y, z;
};

/** Quaternion. */
struct Quaternion
{
  double x, y, z, w;
};

/** Complete state of the quadcopter. */
struct State
{
  Vector3 p;      ///< Position in inertial frame
  Quaternion q;   ///< Rotation from robot frame to inertial frame
  Vector3 v;      ///< Linear velocity in robot frame
  Vector3 w;      ///< Angular velocity of robot relative to inertial frame, expressed in robot frame
};

/** Physical parameters of the quadcopter. */
struct Params
{
  double mass;    ///< mass [kg]
  double radius;  ///< distance to motor [m]
  double gravity; ///< gravity [m/s^2]
  double force;   ///< motor force coefficient
  double torque;  ///< motor torque coefficient
};

/** Motor inputs. */
struct Motor
{
  double front;
  double right;
  double left;
  double back;
};
}
#endif // QUADCOPTER_TYPES_H_
