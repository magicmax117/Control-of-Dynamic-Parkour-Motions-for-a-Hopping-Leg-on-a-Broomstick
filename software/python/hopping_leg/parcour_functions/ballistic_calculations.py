import numpy as np

g = 9.80665
v = 2.8
theta = np.radians(85)
t = 2 * v * np.sin(theta) / g
# t = v * np.sin(theta) / (2 * g) + np.sqrt((v * np.sin(theta) / (2 * g))**2)
print(f'Jumping Time t: {np.round(t, 4)}s')
x = v * np.cos(theta) * t
print(f'Jumping Distance x: {np.round(x, 4)}m')
z_max = v * np.sin(theta) * t / 2 - 0.5 * g * (t / 2)**2
print(f'Maximum Jumping Height z: {np.round(z_max, 4)}m')

# jump length schönrechnung
l_launch = np.cos(np.radians(65)) * 0.2
l_landing = - np.cos(np.radians(87)) * 0.2
print(f'EXTRALENGTH: {l_launch + l_landing}')
print(f'Corrected Distance: {x + l_launch + l_landing}')

r = 0.2
r_f = 0.17
theta_f = np.radians(93)
z0 = np.sin(theta) * r
zf = np.sin(theta_f) * r_f
dz = z0 - zf


