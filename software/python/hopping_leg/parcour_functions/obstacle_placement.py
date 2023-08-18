import numpy as np

# input
distance = 1
height = 0.01

r = 1.17
u = 2 * r * np.pi
share = distance / u
angle = 2 * np.pi * share

x = np.sin(angle) * r
y = - np.cos(angle) * r
z = height / 2

print(f'angle: {angle} rad')
print(f'x-pos: {x} m')
print(f'y-pos: {y} m')
print(f'z-pos: {z} m')
