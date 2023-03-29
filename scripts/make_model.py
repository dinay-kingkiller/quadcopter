#!/usr/bin/env python
from sympy import expand, simplify, Symbol, symbols
from sympy.physics.mechanics import dynamicsymbols, inertia_of_point_mass, msubs, Point, ReferenceFrame, RigidBody, Vector
from sympy.physics.vector.printing import vpprint, vlatex

m, r = symbols("m, r")
u, v, w = dynamicsymbols("u, v, w")
a_d, b_d, c_d = dynamicsymbols("a, b, c")

N = ReferenceFrame("N")
C = N.orientnew("C", "Body", (a_d, b_d, c_d), "123")

c = Point("c")
c.set_vel(N, u*N.x + v*N.y + w*N.z) # this doesn't matter

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
f = I.dot(C.ang_acc_in(N))+C.ang_vel_in(N).cross(robot.angular_momentum(c, N))

### Cleanup
sub_dict = {a_d: Symbol("a"),
             b_d: Symbol("b"),
             c_d: Symbol("c"),
             a_d.diff(): Symbol("diff_a"),
             b_d.diff(): Symbol("diff_b"),
             c_d.diff(): Symbol("diff_c"),
             a_d.diff().diff(): Symbol("ddiff_a"),
             b_d.diff().diff(): Symbol("ddiff_b"),
             c_d.diff().diff(): Symbol("ddiff_c")}


print("f.dot(C.x) =", expand(simplify(msubs(f.dot(C.x), sub_dict))))
print("f.dot(C.y) =", expand(simplify(msubs(f.dot(C.y), sub_dict))))
print("f.dot(C.z) =", expand(simplify(msubs(f.dot(C.z), sub_dict))))
