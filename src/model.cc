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

/// \file model.cpp
/// \brief A ROS node that models a quadcopter.
///
/// This node provides

#include "ros/ros.h"
#include "quadcopter/FullState.h"
#include "quadcopter/Input.h"
#include "quadcopter/State.h"

namespace quadcopter {

/// \class quadcopter::Model
/// \brief
class Model {
private:
  quadcopter::State current;
  float mass;
  float arm_length;
  float k_gravity;
  float k_torque;
  float k_force; // b
public:
  Model(float Mass, float ArmLength, float GAccel, float kTorque, float kForce)
    : mass(Mass),
      arm_length(ArmLength),
      k_gravity(GAccel),
      k_torque(kTorque),
      k_force(kForce) {}
  /// \fn quadcopter::State Quadcopter::Model::ode(state, input)
  /// \brief Given a state and input, returns the derivative of the state.
  quadcopter::FullState ode(quadcopter::FullState state, quadcopter::Input input) {
    quadcopter::FullState deriv;
    float input2 = input.front * input.front
                 + input.right * input.right
                 + input.back * input.back
                 + input.left * input.left;
    deriv.x = state.diff_x;
    deriv.y = state.diff_y;
    deriv.z = state.diff_z;
    deriv.a = state.diff_a;
    deriv.b = state.diff_b;
    deriv.c = state.diff_c;
    deriv.diff_x = k_force / mass * sin(state.a) * input2;
    deriv.diff_y = - k_force / mass * cos(state.a) * sin(state.b) * input2;
    deriv.diff_z = k_force / mass * cos(state.a) * cos(state.b) * input2 - k_gravity;
  }
};
}

// [nx, bx] = r
// <ny, by> = p
// <[nz, bx], nx> = y


// f=[(k*wB**2 + k*wF**2 + k*wL**2 + k*wR**2)*sin(theta),
// -(k*wB**2 + k*wF**2 + k*wL**2 + k*wR**2)*sin(phi)*cos(theta),
// -g*m + (k*wB**2 + k*wF**2 + k*wL**2 + k*wR**2)*cos(phi)*cos(theta),
// -L**2*m*(-dphi*dpsi*sin(psi)*cos(theta) - dphi*dtheta*sin(theta)*cos(psi) + dpsi*dtheta*cos(psi))/2 - L*k*wL**2 + L*k*wR**2,
// -L**2*m*(-dphi*dpsi*cos(psi)*cos(theta) + dphi*dtheta*sin(psi)*sin(theta) - dpsi*dtheta*sin(psi))/2 + L*k*wB**2 - L*k*wF**2,
// -L**2*dphi*dtheta*m*cos(theta) + b*wB**2 + b*wF**2 - b*wL**2 - b*wR**2]
// mm=[[m, 0, 0, 0, 0, 0],
// [0, m, 0, 0, 0, 0],
// [0, 0, m, 0, 0, 0],
// [0, 0, 0, -L**2*m*(-dphi*dpsi*sin(psi)*cos(theta) - dphi*dtheta*sin(theta)*cos(psi) + dpsi*dtheta*cos(psi))/2 + L**2*m*(-dphi*dpsi*sin(psi)*cos(theta) - dphi*dtheta*sin(theta)*cos(psi) + dpsi*dtheta*cos(psi) + cos(psi)*cos(theta))/2,
// -L**2*m*(-dphi*dpsi*cos(psi)*cos(theta) + dphi*dtheta*sin(psi)*sin(theta) - dpsi*dtheta*sin(psi))/2 + L**2*m*(-dphi*dpsi*cos(psi)*cos(theta) + dphi*dtheta*sin(psi)*sin(theta) - dpsi*dtheta*sin(psi) - sin(psi)*cos(theta))/2,
// -L**2*dphi*dtheta*m*cos(theta) + L**2*m*(dphi*dtheta*cos(theta) + sin(theta))],
// [0, 0, 0, -L**2*m*(-dphi*dpsi*sin(psi)*cos(theta) - dphi*dtheta*sin(theta)*cos(psi) + dpsi*dtheta*cos(psi))/2 + L**2*m*(-dphi*dpsi*sin(psi)*cos(theta) - dphi*dtheta*sin(theta)*cos(psi) + dpsi*dtheta*cos(psi) + sin(psi))/2, -L**2*m*(-dphi*dpsi*cos(psi)*cos(theta) + dphi*dtheta*sin(psi)*sin(theta) - dpsi*dtheta*sin(psi))/2 + L**2*m*(-dphi*dpsi*cos(psi)*cos(theta) + dphi*dtheta*sin(psi)*sin(theta) - dpsi*dtheta*sin(psi) + cos(psi))/2, 0],
// [0, 0, 0, 0, 0, -L**2*dphi*dtheta*m*cos(theta) + L**2*m*(dphi*dtheta*cos(theta) + 1)]]

int main(int argc, char **argv) {
  return 0;
}
