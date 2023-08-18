import numpy as np


def parabolic_connection(m, mu):
    g = -9.80665
    alpha_out = np.pi/4
    return alpha_out


def launching_velocity(x, y, alpha_out):
    g = 9.80665
    v_x = x / (np.sqrt((x * np.tan(alpha_out) - y) * 2 / g))
    v_y = v_x * np.tan(alpha_out)
    v = v_y / np.sin(alpha_out)
    return v, v_x, v_y


def force(m, d, v):
    F = 0.5 * m * v**2 / d
    return F


def edit(dx, dz, theta):
    g = 9.80665
    t = np.sqrt((dx * np.tan(theta) - dz) / (0.5 * g))
    v = dx / (t * np.cos(theta))
    return v
