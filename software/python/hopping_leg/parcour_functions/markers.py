import numpy as np

# U = np.pi * 2 * r

# 2 * np.pi * r / 36 = 0.15
r = 0.2 * 36 / (2 * np.pi)
print(f'radius: {r}')

# jump length schönrechnung
l = np.cos(np.radians(75)) * 0.13
print(f'EXTRALENGTH: {l}')
