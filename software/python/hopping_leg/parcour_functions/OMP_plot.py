import matplotlib.pyplot as plt
import numpy as np

obstacle_course = 2
inital_guess = False
jump = True
hurdle_legend = False
REAL = True
fontsize = 23
g = 9.80665
x0 = 0
z0 = 0
zg = 0
r = 0.2
r_f = 0.17
r_s = 0.13
theta_f = np.radians(93)
fide = 1
L1 = 0.15
L2 = 0.14
L1 = 0.17
L2 = 0.16

dx_hip_foot = r_f * np.cos(theta_f)
dz_hip_foot = r_f * np.sin(theta_f)
alpha = np.arccos((L2 ** 2 - L1 ** 2 - r_f ** 2) / (- 2 * L1 * r_f))
gamma = alpha - (theta_f - np.radians(90))
dz_hip_knee = np.cos(gamma) * L1
dx_hip_knee = np.sin(gamma) * L1
x_start = x0
z_start = z0

if obstacle_course == 0:
    name = 'self_short_v2'
    xg = 1.4
    N = 3
    PARKOUR = {"position": np.array([0.7]),
               "height": np.array([0.2]),
               "width": np.array([0.1]),
               "margin_obstacle_horizontal": 0.03,
               "margin_obstacle_vertical": 0.05,
               "margin_knee": 0.03}
    t = [0.36126011, 0.4871499, 0.35907605]
    v = [1.91938974, 2.60955368, 1.90736001]
    theta = np.radians([65., 65., 65.])


if obstacle_course == 1:
    name = 'self_middle_v7'
    xg = 1.4
    N = 3
    PARKOUR = {"position": np.array([0.7]),
               "height": np.array([0.2]),
               "width": np.array([0.3]),
               "margin_obstacle_horizontal": 0.03,
               "margin_obstacle_vertical": 0.05,
               "margin_knee": 0.03}
    t = [0.285465, 0.33878928, 0.54816983]
    v = [1.5      , 2.4468543, 2.5400151]
    theta = np.radians([65., 65., 65.])

if obstacle_course == 2:
    name = 'full_run'
    xg = 7.2
    N = 12
    PARKOUR = {"position": np.array([1, 1.3, 2.3, 4.2, 4.8, 5.9]),
               "height": np.array([0.1, 0.3, 0.1, 0.2, 0.2, 0.35]),
               "width": np.array([0.1, 0.1, 0.2, 0.2, 0.2, 0.1]),
               "margin_obstacle_horizontal": 0.03,
               "margin_obstacle_vertical": 0.05,
               "margin_knee": 0.03}
    t = [0.51878198, 0.27460536, 0.55826941, 0.41831961, 0.55826941,
       0.52202959, 0.32382857, 0.48489769, 0.59057445, 0.37784753,
       0.63385083, 0.39038826]
    v = [2.78227764, 1.84129489, 2.8       , 2.4966472 , 2.8       ,
       2.8       , 2.39427679, 2.59724779, 2.8       , 2.8       ,
       2.8       , 2.07959885]
    theta = np.radians([65.        , 65.        , 65.        , 65.        , 65.        ,
       65.        , 65.        , 65.        , 65.        , 75.85067896,
       65.        , 65.        ])

if obstacle_course == 3:
    xg = 1
    N = 2
    PARKOUR = {"position": np.array([0.5]),
               "height": np.array([0.1]),
               "width": np.array([0.2]),
               "margin_obstacle_horizontal": 0.03,
               "margin_obstacle_vertical": 0.05,
               "margin_knee": 0.03}
    t = [0.285465, 0.33878928, 0.54816983]
    v = [1.5     , 2.4468543, 2.5400151]
    theta = np.radians([65., 65., 65.])
obstacle_number = len(PARKOUR["position"])

# ################################################### INITIAL GUESS ####################################################

ig_theta = np.radians(65)
dx = xg / N - np.cos(ig_theta) * r + dx_hip_foot
dz = -(np.sin(ig_theta) * r - np.sin(theta_f) * r_f)
ig_t = np.sqrt((-dz + dx * np.tan(ig_theta)) / (0.5 * g))
ig_v = dx / (ig_t * np.cos(ig_theta))

######################################### PLOTTING #########################################
my_dpi = 80
if hurdle_legend or inital_guess or REAL:
    fig, ax = plt.subplots(figsize=(1500/my_dpi, 600/my_dpi), dpi=my_dpi)
    # fig.figure(figsize=(1000/my_dpi, 400/my_dpi), dpi=my_dpi)
else:
    fig, ax = plt.subplots(figsize=(10, 4))
ax.set_xlabel("x [m]", fontsize=fontsize)
ax.set_ylabel("z [m]", fontsize=fontsize)
plt.grid(True)

# PLOT NICE CURVE
relative_t = np.linspace(0, 1, 40)
relative_t_fide = np.linspace(0, 1, fide)
contact_sequence_x = np.zeros(N+1)
contact_sequence_z = np.zeros(N+1)
X0 = r * np.cos(theta[0]) - dx_hip_foot
Z0 = r * np.sin(theta[0]) - dz_hip_foot
for i in range(N):
    T = t[i] * relative_t
    VX = v[i] * np.cos(theta[i])
    VZ = v[i] * np.sin(theta[i])
    X = X0 + VX * T
    Z = Z0 + VZ * T - 0.5 * g * T**2
    X_K = X + dx_hip_foot - dx_hip_knee
    Z_K = Z + dz_hip_foot - dz_hip_knee
    if jump:
        ax.plot(X, Z, "b-", linewidth=2, label='Desired' if i == 0 else '')
        # ax.plot(X_K, Z_K, "y-")
    # fide
        T_fide = t[i] * relative_t_fide
        X_fide = X0 + VX * T_fide
        Z_fide = Z0 + VZ * T_fide - 0.5 * g * T_fide**2
        #if i < N-1:
            # ax.plot(X[-1], Z[-1], "bo", markersize=9, label='Desired Contact' if i == 1 and not REAL else '')
    # SAVE FOOT LOCATION
    contact_sequence_x[i+1] = X[-1]
    contact_sequence_z[i+1] = Z[-1]
    if i < N-1:
        X0 = X[-1] + r * np.cos(theta[i+1]) - dx_hip_foot
        Z0 = Z[-1] + r * np.sin(theta[i+1]) - dz_hip_foot

plot_stepsize = 0.0001
parkour_plot_x = np.arange(x_start, xg + plot_stepsize, plot_stepsize)
parkour_plot_obstacle_z = np.zeros(len(parkour_plot_x))

for i in range(obstacle_number):
    start_point = (PARKOUR["position"][i] - PARKOUR["width"][i] / 2) / plot_stepsize
    end_point = (PARKOUR["position"][i] + PARKOUR["width"][i] / 2) / plot_stepsize
    parkour_plot_obstacle_z[int(start_point):int(end_point)] = PARKOUR["height"][i]
    parkour_plot_obstacle_z[[int(start_point), int(end_point)]] = (PARKOUR["height"][i] +
                                                                   PARKOUR["margin_obstacle_vertical"])
if hurdle_legend:
    ax.plot(parkour_plot_x, parkour_plot_obstacle_z, 'g', linewidth=3, label='Contact Allowed')
else:
    ax.plot(np.append(np.array([-10]), np.append(parkour_plot_x, np.array([10]))),
            np.append(np.array([0]), np.append(parkour_plot_obstacle_z, np.array([0]))), 'g', linewidth=3)

for i in range(obstacle_number):
    front_a = (PARKOUR["position"][i] - PARKOUR["width"][i] / 2 - PARKOUR["margin_obstacle_horizontal"]) / plot_stepsize
    front_b = (PARKOUR["position"][i] - PARKOUR["width"][i] / 2 + PARKOUR["margin_obstacle_horizontal"]) / plot_stepsize
    back_a = (PARKOUR["position"][i] + PARKOUR["width"][i] / 2 - PARKOUR["margin_obstacle_horizontal"]) / plot_stepsize
    back_b = (PARKOUR["position"][i] + PARKOUR["width"][i] / 2 + PARKOUR["margin_obstacle_horizontal"]) / plot_stepsize
    if i == obstacle_number - 1 and hurdle_legend:
        ax.plot(parkour_plot_x[int(front_a):int(front_b)], parkour_plot_obstacle_z[int(front_a):int(front_b)],
                'r', linewidth=3)
        ax.plot(parkour_plot_x[int(back_a):int(back_b)], parkour_plot_obstacle_z[int(back_a):int(back_b)],
                'r', linewidth=3, label='Contact Prohibited')
    else:
        ax.plot(parkour_plot_x[int(front_a):int(front_b)], parkour_plot_obstacle_z[int(front_a):int(front_b)],
                'r', linewidth=3)
        ax.plot(parkour_plot_x[int(back_a):int(back_b)], parkour_plot_obstacle_z[int(back_a):int(back_b)],
                'r', linewidth=3)

        # hurdle constraints
        if obstacle_course == 2:
            ax.plot(parkour_plot_x[int(1.05 / plot_stepsize):int(1.65 / plot_stepsize)],
                    parkour_plot_obstacle_z[int(1.05 / plot_stepsize):int(1.65 / plot_stepsize)], 'r', linewidth=3)
            ax.plot(parkour_plot_x[int(1.79 / plot_stepsize):int(2.21 / plot_stepsize)],
                    parkour_plot_obstacle_z[int(1.79 / plot_stepsize):int(2.21 / plot_stepsize)], 'r', linewidth=3)
            ax.plot(parkour_plot_x[int(2.4 / plot_stepsize):int(2.8 / plot_stepsize)],
                    parkour_plot_obstacle_z[int(2.4 / plot_stepsize):int(2.8 / plot_stepsize)], 'r', linewidth=3)
            ax.plot(parkour_plot_x[int(4.3 / plot_stepsize):int(4.7 / plot_stepsize)],
                    parkour_plot_obstacle_z[int(4.3 / plot_stepsize):int(4.7 / plot_stepsize)], 'r', linewidth=3)
            # prog.AddConstraint(abs(x - 1.35) >= 0.3)
            # prog.AddConstraint(abs(x - 2.0) >= 0.21)
            # prog.AddConstraint(abs(x - 2.6) >= 0.2)
            # prog.AddConstraint(abs(x - 4.5) >= 0.2)

if inital_guess:
    X0 = r * np.cos(ig_theta) - dx_hip_foot
    Z0 = r * np.sin(ig_theta) - dz_hip_foot
    for i in range(N):
        T = ig_t * relative_t
        VX = ig_v * np.cos(ig_theta)
        VZ = ig_v * np.sin(ig_theta)
        X = X0 + VX * T
        Z = Z0 + VZ * T - 0.5 * g * T ** 2
        X_K = X + dx_hip_foot - dx_hip_knee
        Z_K = Z + dz_hip_foot - dz_hip_knee
        ax.plot(X, Z, "b--", linewidth=2, label='Inital Guess' if i == 0 else '')
        if i == 0:
            ax.plot(X[-1], Z[-1], "bo", markersize=9, label='Contact Point')
        elif i < N-1:
            ax.plot(X[-1], Z[-1], "bo", markersize=7)
        if i < N - 1:
            X0 = X[-1] + r * np.cos(ig_theta) - dx_hip_foot
            Z0 = Z[-1] + r * np.sin(ig_theta) - dz_hip_foot

if REAL:
    path = f'software/python/experiments/parcour/data/plot/{name}_plot.csv'
    dtype = {'names': ('X', 'Y', 'Z', 'x_park'),
             'formats': ['f8', 'f8', 'f8', 'f8']}
    data = np.loadtxt(path, dtype, skiprows=1, delimiter=",")
    X = data["X"]
    Y = data["Y"]
    Z = data["Z"]
    x_park = data["x_park"]
    ax.plot(x_park, Z, 'C1', linewidth=2, label='Measured')

ax.plot(x0, z0, '^b', markersize=10, label='Start' if not REAL else'')
ax.plot(contact_sequence_x[1:-1], contact_sequence_z[1:-1], 'bo', markersize=10)
ax.plot(xg, 0, 'vb', markersize=10, label='Goal' if not REAL else '')
plt.xticks(fontsize=fontsize)
plt.yticks(fontsize=fontsize)
plt.xlim([-0.08, xg + 0.08])
if hurdle_legend or inital_guess:
    plt.ylim([-0.02, 0.42])
ax.legend(loc='upper left', fontsize=fontsize)

plt.show()
