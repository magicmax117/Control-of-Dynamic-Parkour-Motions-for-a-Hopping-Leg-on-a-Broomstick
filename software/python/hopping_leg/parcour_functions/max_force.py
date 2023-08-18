import numpy as np
from numpy.linalg import inv
from hopping_leg.plant.plant import HopperPlant

g = 9.80665
L1   = 0.15
L2   = 0.14
mass = [0.91281,   1.2804,     0.13037]
Izz  = [0.0015899, 6.3388E-05]
com1 = [0.059331,  1.3564E-05]
com2 = [0.078298,  1.088E-08]

contact_threshold = np.array([2, 0.5, 3, 2])

plant = HopperPlant(mass=mass,
                    Izz=Izz,
                    com1=com1,
                    com2=com2,
                    link_length=[L1, L2],
                    gravity=g)

v = 3
m = 1.2
tau_max = 10

r_c = 0.13
r_e = 0.18
r = np.arange(r_c, r_e, 0.005)
# r = np.array([np.sqrt(0.14**2 + 0.15**2), np.sqrt(0.14**2 + 0.15**2)])
theta = np.radians(np.arange(95, 115, 1))
# theta = np.radians(np.array([90, 90]))
F = m * v**2 / (2 * (r_e - r_c))

# for i in range(len(theta)):
#     Fx = np.sin(theta[i]) * F
#     Fy = -np.cos(theta[i]) * F
#     for j in range(len(r)):
#         q1, q2 = plant.kinematics_new_frame(r[j], theta[i])
#         print(f'q1, q2: {np.degrees([q1, q2])}')
#         J = plant.jacobian(q1, q2)
#         T = np.matmul(J.T, [Fx, Fy])
#         GG = np.matmul(inv(J.T), T)
#         print(f'FORZA: {GG} orig: {[Fx, Fy]}')
#         if (abs(T) >= tau_max).any():
#             print(f'Failed for r = {r[j]} and theta = {np.degrees(theta[i])} :')
#             print(T)

m = 1.2
r = 0.14
theta = np.radians(65)

# F = m * v**2 / (2 * (r_e - r_c))
# Fx = np.sin(theta) * F
# Fy = -np.cos(theta) * F

q1, q2 = plant.kinematics_new_frame(r, theta)
J = plant.jacobian(q1, q2)
print(f'Joint states: {np.degrees([q1, q2])}')
F = np.matmul(inv(J.T), [[0], [10]])
print(f'FORCE VECTOR: {F}')
T = np.matmul(J.T, F)
print(f'TORQUE VECTOR: {T}')
FF = np.sqrt(F[0]**2 + F[1]**2)
v = np.sqrt(FF * 0.06 * 2 / m)
print(f'velocity: {v}')
