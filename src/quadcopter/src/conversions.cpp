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

#include "quadcopter/conversions.h"

namespace quadcopter
{
Motor toStruct(const quad_msgs::Motor& msg)
{
  Motor m;
  m.front = msg.front;
  m.back = msg.back;
  m.right = msg.right;
  m.left = msg.left;
  return m;
}
quad_msgs::Motor toMsg(const Motor& m)
{
  quad_msgs::Motor msg;
  msg.front = m.front;
  msg.back = m.back;
  msg.right = m.right;
  msg.left = m.left;
  return msg;
}
geometry_msgs::Pose toMsg(const State& s)
{
  geometry_msgs::Pose msg;
  msg.position.x = s.p.x;
  msg.position.y = s.p.y;
  msg.position.z = s.p.z;
  msg.orientation.x = s.q.x;
  msg.orientation.y = s.q.y;
  msg.orientation.z = s.q.z;
  msg.orientation.w = s.q.w;
  return msg;
}
}
