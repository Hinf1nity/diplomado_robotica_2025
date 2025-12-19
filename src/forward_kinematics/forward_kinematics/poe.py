import sympy as sp
from sympy import symbols, Matrix, eye, sin, cos, expand, pprint, pi
from scipy.spatial.transform import Rotation as R_scipy
import numpy as np
# Define symbols
theta = symbols('theta', real=True)
w0, w1, w2 = symbols('omega_0 omega_1 omega_2', real=True)

# Define vectors
w = Matrix([w0, w1, w2])   # rotation axis, unit vector


# Skew-symmetric matrix of w
skew_w = Matrix([
    [0, -w[2], w[1]],
    [w[2], 0, -w[0]],
    [-w[1], w[0], 0]
])

pprint(skew_w)
# Rodrigues rotation matrix
R = eye(3) + sin(theta)*skew_w + (1 - cos(theta))*(skew_w**2)

qx, qy, qz = symbols('q_x q_y q_z', real=True)
q = Matrix([[qx], [qy], [qz]])


L1, L2 = symbols('L_1 L_2', real=True)
t1, t2, t3, t4, t5 = symbols(
    't1 t2 t3 t4 t5', real=True)

v = -skew_w*q
pprint(v)
pprint(v.subs({w0: 1, w1: 0, w2: 0, qx: 0, qy: 0, qz: -L1}))
Rv = (eye(3)*theta + (1 - cos(theta)) *
      (skew_w) + (theta-sin(theta))*(skew_w**2))*v

# Display results

pprint(R.subs({theta: 0.524, w0: 0, w1: 0.866, w2: 0.5}))


T = Matrix([[R[0, 0], R[0, 1], R[0, 2], Rv[0]], [R[1, 0], R[1, 1],
           R[1, 2], Rv[1]], [R[2, 0], R[2, 1], R[2, 2], Rv[2]], [0, 0, 0, 1]])

# Examples
# M=Matrix([[0,0,1,L1],[0,1,0,0],[-1,0,0,-L2],[0,0,0,1]])
# T1=T.subs({theta:t1,w0:0,w1:0,w2:1,qx:0, qy:0, qz:0})
# T2=T.subs({theta:t2,w0:0,w1:-1,w2:0,qx:L1, qy:0, qz:0})
# T3=T.subs({theta:t3,w0:1,w1:0,w2:0,qx:0, qy:0, qz:-L2})

T = Matrix([[R[0, 0], R[0, 1], R[0, 2], Rv[0]], [R[1, 0], R[1, 1], R[1, 2], Rv[1]], [
    R[2, 0], R[2, 1], R[2, 2], Rv[2]], [0, 0, 0, 1]])

M = Matrix([[0, 0, 1, 0.895107], [0, 1, 0, 0],
            [-1, 0, 0, 1.26315], [0, 0, 0, 1]])
T1 = T.subs({theta: t1, w0: 0, w1: 0,
            w2: 1, qx: 0, qy: 0, qz: 0.45})
T2 = T.subs({theta: t2, w0: 0, w1: 1,
            w2: 0, qx: 0.155, qy: 0, qz: 0.45})
T3 = T.subs({theta: t3, w0: 0, w1: -1, w2: 0,
            qx: 0.154877, qy: 0, qz: 1.064})
T4 = T.subs({theta: t4, w0: 0, w1: 1, w2: 0,
            qx: 0.795107, qy: 0, qz: 1.26326})
T5 = T.subs({theta: t5, w0: -1, w1: 0, w2: 0,
            qx: 0.795107, qy: 0, qz: 1.26326})
print("T1:")
pprint(T1)
print("T2:")
pprint(T2)
print("T3:")
pprint(T3)
print("T4:")
pprint(T4)
print("T5:")
pprint(T5)
# pprint(T1*T2*T3*M)

# T13=T1*T2*T3*M
# pprint(T13.subs({t1:0,t2:pi/2,t3:0}))
# exponential
T1345 = T1*T2*T3*T4*T5*M
print("px: ", T1345[0, 3])
print("py: ", T1345[1, 3])
print("pz: ", T1345[2, 3])
# res=T1345.subs({t1:1.556,t2:0,t3:-0.705,t4:0.649,t5:-1.257})
# print("res:")
# pprint(res)
# rotation = R_scipy.from_matrix(np.array(res)[0:3, 0:3])
# quaternion = rotation.as_quat()
# print("position (x,y,z): ", res[0:3, 3])
# print("orientation (x,y,z,w): ", quaternion)
