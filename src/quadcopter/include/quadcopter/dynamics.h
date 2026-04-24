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

#ifndef QUADCOPTER_DYNAMICS_H_
#define QUADCOPTER_DYNAMICS_H_

#include "quadcopter/types.h"

namespace quadcopter
{
/**
  * @brief Advances the quadcopter state by a timestep dt.
  *
  * This function performs a single integration step over SE(3) and SO(3) using
  * the current state `s`, motor inputs `u`, and system parameters `p`.
  * Small-angle approximations are used when angular velocity is small.
  * Quaternion normalization is applied to maintain valid rotations.
  *
  * @param s Current state of the quadcopter.
  * @param u Motor inputs.
  * @param p Quadcopter parameters (mass, gravity, force constants, etc.).
  * @param dt Time step over which to integrate.
  * @return State Next state of the quadcopter after dt.
  */
State step(const State& s, const Motor& u, const Params& p, double dt);
}


#endif // QUADCOPTER_DYNAMICS_H_
