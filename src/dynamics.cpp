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


namespace quadcopter
{
  namespace
  {
  Vector3 cross(Vector3 u, Vector3 v)
  {
    Vector3 r;
    r.x = u.y * v.z - u.z * v.y;
    r.y = u.z * v.x - u.x * v.z;
    r.z = u.x * v.y - u.y * v.x;
    return r;
  }
  Vector3 rotate(Quaternion q, Vector3 v)
  {
    Vector3 t
      {
       2.0 * (q.y * v.z - q.z * v.y),
       2.0 * (q.z * v.x - q.x * v.z),
       2.0 * (q.x * v.y - q.y * v.x)
      };
    Vector3 r
      {
       v.x + q.w * t.x + (q.y * t.z - q.z * t.y),
       v.y + q.w * t.y + (q.z * t.x - q.x * t.z),
       v.z + q.w * t.z + (q.x * t.y - q.y * t.x)
      };
    return r;
  }
  }
  
  Vector3 get_lin_accel(State s, Motor u)
  {
  }

  Vector3 get_lin_accel(State s, Motor u)
  {
  }

  State get_trajectory(State s, Motor u)
  {
  }

  State get_slope(State s, Motor u)
  {
  }

  State get_next_state(State s, double t, Motor u)
  {
    
  }
}
