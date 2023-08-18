import numpy as np
import matplotlib.pyplot as plt

g = 9.80665
r = 0.2
theta = np.radians(65)
r_f = 0.17
theta_f = np.radians(93)

# plot the resulting trajectory
fig, ax = plt.subplots(figsize=(20, 10))
ax.set_xlabel("x")
ax.set_ylabel("z")
plt.grid(True)

for i in range(2):
    x0 = r * np.cos(theta) - 0.1 * i
    z0 = r * np.sin(theta)
    x0_f = x0 - r_f * np.cos(theta_f)  # initial foot position in x
    z0_f = z0 - r_f * np.sin(theta_f)  # initial foot position in z

    dx = 0.6 - x0_f
    dz = 0.1 - z0_f

    t = np.sqrt((dx * np.tan(theta) - dz) / (0.5 * g))
    v = dx / (t * np.cos(theta))
    print(f'Launch Velocity: {v} m/s')
    print(f'Time: {t} s')
    if i == 0:
        v_i = v
        t_i = t
    else:
        print(f'Launch Velocity Difference: {v_i - v} m/s')
        # print(f'Launch Velocity Verhältniss: {v_i / v}')
        print(f'Time Difference: {t_i - t} s')
        # print(f'Launch Velocity Verhältniss: {v_i / v}')
    vx = np.cos(theta) * v
    vz = np.sin(theta) * v

    x = np.arange(0, 1, 0.00001)
    z = - 0.5 * g * ((x - x0_f) / vx) ** 2 + vz * ((x - x0_f) / vx) + z0_f

    Xs = x0_f
    Zs = - 0.5 * g * ((Xs - x0_f) / vx) ** 2 + vz * ((Xs - x0_f) / vx) + z0_f
    Xf = dx + x0_f
    Zf = - 0.5 * g * ((Xf - x0_f) / vx) ** 2 + vz * ((Xf - x0_f) / vx) + z0_f

    ax.plot(x, z)
    ax.plot(Xs, Zs, 'o')
    ax.plot(Xf, Zf, 'o')
plt.show()

# z_max = vz * t / 2 - 0.5 * g * (t / 2) ** 2
# print(f'z_max: {z_max} m')
