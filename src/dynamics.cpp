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

#include "quadcopter/dynamics.h"

#include <cmath>

namespace quadcopter
{
namespace
{
// Some temporary vector/quaternion math.
Vector3 cross(const Vector3& u, const Vector3& v)
{
  Vector3 r;
  r.x = u.y * v.z - u.z * v.y;
  r.y = u.z * v.x - u.x * v.z;
  r.z = u.x * v.y - u.y * v.x;
  return r;
}
Vector3 rotate(const Quaternion& q, const Vector3& v)
{
  Vector3 t
    {
     2.0 * (q.y * v.z - q.z * v.y),
     2.0 * (q.z * v.x - q.x * v.z),
     2.0 * (q.x * v.y - q.y * v.x)
    };
  Vector3 r
    {
     v.x + q.w * t.x + q.y * t.z - q.z * t.y,
     v.y + q.w * t.y + q.z * t.x - q.x * t.z,
     v.z + q.w * t.z + q.x * t.y - q.y * t.x
    };
  return r;
}
double norm(const Vector3& v)
{
  return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}
Quaternion normalize(const Quaternion& q)
{
  double inv_bar = 1.0 / sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
  Quaternion r;
  r.x = q.x*inv_bar;
  r.y = q.y*inv_bar;
  r.z = q.z*inv_bar;
  r.w = q.w*inv_bar;
  return r;
}
Quaternion quat_mult(const Quaternion& p, const Quaternion& q)
{
    Quaternion r;
    r.w = p.w*q.w - p.x*q.x - p.y*q.y - p.z*q.z;
    r.x = p.w*q.x + p.x*q.w + p.y*q.z - p.z*q.y;
    r.y = p.w*q.y - p.x*q.z + p.y*q.w + p.z*q.x;
    r.z = p.w*q.z + p.x*q.y - p.y*q.x + p.z*q.w;
    return r;
}
}

State step(const State& s, const Motor& u, const Params& p, double dt)
{
  State n; // next state
  Vector3 w_dot, v_dot, x_dot;
  Quaternion q_dot;

  double f = u.front*u.front;
  double r = u.right*u.right;
  double l = u.left*u.left;
  double b = u.back*u.back;

  // Sum of torques
  double p_moment = p.force / (p.mass * p.radius);
  w_dot.x = 2.0 * p_moment * (r-l) - s.w.y * s.w.z;
  w_dot.y = 2.0 * p_moment * (b-f) + s.w.x * s.w.z;
  w_dot.z = p_moment * (f+b-l-r);

  // Velocity vector derivative
  double T = p.force * (f+r+l+b);
  Vector3 wxv = cross(s.w, s.v);
  v_dot.x = -2.0*(s.q.x*s.q.z+s.q.y*s.q.w)*p.gravity-wxv.x;
  v_dot.y = -2.0*(s.q.x*s.q.w+s.q.y*s.q.z)*p.gravity-wxv.y;
  v_dot.z = (s.q.w*s.q.w-s.q.x*s.q.x-s.q.y*s.q.y+s.q.z*s.q.z)*p.gravity+T-wxv.z;

  // Update twist
  n.w.x = s.w.x + dt * w_dot.x;
  n.w.y = s.w.y + dt * w_dot.y;
  n.w.z = s.w.z + dt * w_dot.z;
  n.v.x = s.v.x + dt * v_dot.x;
  n.v.y = s.v.y + dt * v_dot.y;
  n.v.z = s.v.z + dt * v_dot.z;

  double w_bar = norm(n.w);
  double theta = w_bar * dt;
  double inv_w = 1.0 / w_bar;
  double inv_w2 = inv_w*inv_w;
  double inv_w3 = inv_w2*inv_w;

  // Update position over SE(3)
  wxv = cross(n.w, n.v);
  Vector3 wxwxv = cross(n.w, wxv);
  Vector3 v_hat;
  double c_coef, s_coef;
  if (theta < 1e-3)
    {
      // First-order Taylor expansion for small angles
      c_coef = 0.5*dt*dt;
      s_coef = dt*dt*dt / 6.0;
    }
  else
    {
      c_coef = inv_w2 * (1-cos(theta));
      s_coef = inv_w3 * (theta-sin(theta));
    }
  v_hat.x = n.v.x + c_coef*wxv.x + s_coef*wxwxv.x;
  v_hat.y = n.v.y + c_coef*wxv.y + s_coef*wxwxv.y;
  v_hat.z = n.v.z + c_coef*wxv.z + s_coef*wxwxv.z;
  Vector3 p_dot = rotate(s.q, v_hat);
  n.p.x = s.p.x + dt * p_dot.x;
  n.p.y = s.p.y + dt * p_dot.y;
  n.p.z = s.p.z + dt * p_dot.z;

  // Update quaternion over SO(3)
  if (theta < 1e-3)
    {
      // First-order Taylor expansion for small angles
      c_coef = 1 - 0.125*theta*theta;
      s_coef = dt * (0.5 - theta*theta/48.0);
    }
  else
    {
      c_coef = cos(0.5*theta);
      s_coef = inv_w * sin(0.5*theta);
    }
  q_dot.w = c_coef;
  q_dot.x = s_coef * n.w.x;
  q_dot.y = s_coef * n.w.y;
  q_dot.z = s_coef * n.w.z;
  // Re-normalize to prevent drift.
  n.q = normalize(quat_mult(s.q, q_dot));

  return n;
}
}
