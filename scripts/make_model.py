#!/usr/bin/env python
from sympy import symbols
from sympy.physics.mechanics import *
m, r = symbols("m, r")
diff_x, diff_y, diff_z = dynamicsymbols("x, y, z", 1)
a, b, c = dynamicsymbols("a, b, c") 

N = ReferenceFrame("N")
C = N.orientnew("C", "Body", (a, b, c), "123")

c = Point("c")
c.set_vel(N, diff_x*N.x + diff_y*N.y + diff_z*N.z)

I = inertia_of_point_mass(m, r*C.x, C)
I += inertia_of_point_mass(m, r*C.y, C)
I += inertia_of_point_mass(m, -r*C.x, C)
I += inertia_of_point_mass(m, -r*C.y, C)

robot = RigidBody("R", c, C, m, (I, c))
print("velocity =", C.ang_vel_in(N))
print("---")
print("acceleration = ", C.ang_acc_in(N))
print("---")
print("inertia = ", I)
print("---")
print("momentum =", robot.angular_momentum(c, N))
print("---")
print("f* = ", I * C.ang_acc_in(N) + C.ang_vel_in(N).cross(robot.angular_momentum(c, N)))
