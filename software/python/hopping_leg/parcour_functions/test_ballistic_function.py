import numpy as np
import matplotlib.pyplot as plt

g = 9.80665
theta = np.radians(65)
v = 2.47822189

r = 0.2
r_f = 0.17
theta_f = np.radians(93)

x0 = r * np.cos(theta)
z0 = r * np.sin(theta)
x0_f = x0 - r_f * np.cos(theta_f)  # initial foot position in x
z0_f = z0 - r_f * np.sin(theta_f)  # initial foot position in z

vx = np.cos(theta) * v
vz = np.sin(theta) * v

x = np.arange(0, 1, 0.00001)
z = - 0.5 * g * ((x - x0_f) / vx) ** 2 + vz * ((x - x0_f) / vx) + z0_f

X = x0_f
Z = - 0.5 * g * ((X - x0_f) / vx) ** 2 + vz * ((X - x0_f) / vx) + z0_f

# plot the resulting trajectory
fig, ax = plt.subplots(figsize=(20, 10))
ax.set_xlabel("x")
ax.set_ylabel("z")
plt.grid(True)

ax.plot(x, z)
ax.plot(X, Z, 'o')
plt.show()