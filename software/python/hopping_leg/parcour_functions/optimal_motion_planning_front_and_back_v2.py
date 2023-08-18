import matplotlib.pyplot as plt
import numpy as np
from pydrake.all import MathematicalProgram, MixedIntegerBranchAndBound, SnoptSolver, IpoptSolver, NloptSolver, GurobiSolver
import time
from datetime import timedelta
from hopping_leg.parcour_functions.parkour_builder import polynom_generator

def motion_planning(obstacle_course=None):

    g = 9.80665
    x0 = 0
    z0 = 0
    zg = 0
    r = 0.2
    r_f = 0.17
    r_s = 0.13
    theta_f = np.radians(93)
    fide = 2
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

    if obstacle_course is None:
        obstacle_course = 0
    if obstacle_course == 0:
        xg = 1.4
        N = 3
        PARKOUR = {"position": np.array([ 0.7]),
                   "height":   np.array([ 0.2]),
                   "width":    np.array([ 0.1]),
                   "hurdle":   np.array([False]),
                   "margin_obstacle_horizontal": 0.03,
                   "margin_obstacle_vertical": 0.05,
                   "margin_knee": 0.03}

    if obstacle_course == 1:
        xg = 1.4
        N = 3
        PARKOUR = {"position": np.array([0.7]),
                   "height": np.array([0.2]),
                   "width": np.array([0.3]),
                   "hurdle": np.array([False]),
                   "margin_obstacle_horizontal": 0.03,
                   "margin_obstacle_vertical": 0.05,
                   "margin_knee": 0.03}

    if obstacle_course == 2:
        xg = 7.2
        N = 12
        PARKOUR = {"position": np.array([    1,  1.3,  2.3, 4.2, 4.8, 5.9]),
                   "height":   np.array([  0.1,  0.3,  0.1, 0.2, 0.2, 0.35]),
                   "width":    np.array([  0.1,  0.1,  0.2, 0.2, 0.2, 0.1]),
                   "hurdle":   np.array([False, False, False]),
                   "margin_obstacle_horizontal": 0.03,
                   "margin_obstacle_vertical": 0.05,
                   "margin_knee": 0.03}

    obstacle_number = len(PARKOUR["position"])

    # ----------------------------------------------- create program -----------------------------------------------
    prog = MathematicalProgram()
    t = prog.NewContinuousVariables(N, "t")
    prog.AddBoundingBoxConstraint(0.2, 3, t)
    v = prog.NewContinuousVariables(N, "v")
    prog.AddBoundingBoxConstraint(1.5, 2.8, v)
    theta = prog.NewContinuousVariables(N, "theta")
    prog.AddBoundingBoxConstraint(np.radians(65), np.radians(85), theta)

    binarys_front = [None] * len(PARKOUR["position"])
    binarys_back = [None] * len(PARKOUR["position"])
    for i in range(obstacle_number):
        name_front = f'delta_{i+1}_hurdle_front'
        name_back = f'delta_{i+1}_hurdle_back'
        binarys_front[i] = prog.NewBinaryVariables(N, name_front)
        binarys_back[i] = prog.NewBinaryVariables(N, name_back)
    PARKOUR["binarys_hurdle_front"] = binarys_front
    PARKOUR["binarys_hurdle_back"] = binarys_back

    # ----------------------------------------------- set constraints -----------------------------------------------
    print('Starting to define Constraints')
    t_start = time.time()
    x0 = r * np.cos(theta[0]) - dx_hip_foot
    z0 = r * np.sin(theta[0]) - dz_hip_foot
    x0_knee = r * np.cos(theta[0]) - dx_hip_knee
    z0_knee = r * np.sin(theta[0]) - dz_hip_knee
    for i in range(N):
        vx = v[i] * np.cos(theta[i])
        vz = v[i] * np.sin(theta[i])
        x = x0 + vx * t[i]
        z = z0 + t[i] * vz - 0.5 * g * t[i] * t[i]
        parkour_height = 0        # prog.AddConstraint(abs(x - 2.425) >= 0.275)
        for k in range(obstacle_number):
            # x_pos of front and back hurdle
            x_position_hurdle_front = PARKOUR["position"][k] - PARKOUR["width"][k] / 2
            x_position_hurdle_back = PARKOUR["position"][k] + PARKOUR["width"][k] / 2

            # front/back binary for obstacle 0 before x_pos and 1 after
            prog.AddConstraint(PARKOUR["binarys_hurdle_front"][k][i] * (x_position_hurdle_front - x) <= 0)
            prog.AddConstraint((1 - PARKOUR["binarys_hurdle_front"][k][i]) * (x - x_position_hurdle_front) <= 0)
            prog.AddConstraint(PARKOUR["binarys_hurdle_back"][k][i] * (x_position_hurdle_back - x) <= 0)
            prog.AddConstraint((1 - PARKOUR["binarys_hurdle_back"][k][i]) * (x - x_position_hurdle_back) <= 0)

            # landing height on platform
            parkour_height += PARKOUR["height"][k] * (PARKOUR["binarys_hurdle_front"][k][i] -
                                                      PARKOUR["binarys_hurdle_back"][k][i])

            # hurdle constraints
            if obstacle_course == 2:
                prog.AddConstraint(abs(x - 1.35) >= 0.3)
                prog.AddConstraint(abs(x - 2.0) >= 0.21)
                prog.AddConstraint(abs(x - 2.6) >= 0.2)
                prog.AddConstraint(abs(x - 4.5) >= 0.2)
            # if PARKOUR["hurdle"][k]:
            #     prog.AddConstraint(PARKOUR["binarys_hurdle_front"][k][i] - PARKOUR["binarys_hurdle_back"][k][i] <= 0)

            # constraint for allowed landing range on platform
            # if PARKOUR["hurdle"][k]:
            #     prog.AddConstraint(abs(x - PARKOUR["position"][k]) >= PARKOUR["width"][k] / 2 +
            #     PARKOUR["margin_obstacle_horizontal"])
            # else:
            #     pass
            prog.AddConstraint(abs(x - x_position_hurdle_front) >= PARKOUR["margin_obstacle_horizontal"])
            prog.AddConstraint(abs(x - x_position_hurdle_back) >= PARKOUR["margin_obstacle_horizontal"])
            # prog.AddConstraint(abs(x - (x_position_hurdle_front + PARKOUR["margin_obstacle_horizontal"] / 2)) >=
            #                    PARKOUR["margin_obstacle_horizontal"] / 2)
            # prog.AddConstraint(abs(x - (x_position_hurdle_back - PARKOUR["margin_obstacle_horizontal"] / 2)) >=
            #                    PARKOUR["margin_obstacle_horizontal"] / 2)
            # prog.AddConstraint((x - (x_position_hurdle_front + PARKOUR["margin_obstacle_horizontal"] / 2))**2 >=
            #                    (PARKOUR["margin_obstacle_horizontal"] / 2)**2)
            # prog.AddConstraint((x - (x_position_hurdle_back +- PARKOUR["margin_obstacle_horizontal"] / 2)) ** 2 >=
            #                    (PARKOUR["margin_obstacle_horizontal"] / 2) ** 2 + 0.0001)
            # prog.AddConstraint(abs(x - x_position_hurdle_back) >= PARKOUR["margin_obstacle_horizontal"])
            # prog.AddConstraint(abs(x - (x_position_hurdle_back - PARKOUR["margin_obstacle_horizontal"] / 2)) >=
            #                    PARKOUR["margin_obstacle_horizontal"] / 2 + 0.01)
            # prog.AddConstraint((x - (x_position_hurdle_back - PARKOUR["margin_obstacle_horizontal"] / 2))**2)

            # z_pos of foot/knee at hurdle front/back
            z_foot_hurdle_front = (- 0.5 * g * ((x_position_hurdle_front - x0) / vx) ** 2
                                   + vz * ((x_position_hurdle_front - x0) / vx) + z0)
            z_knee_hurdle_back = (- 0.5 * g * ((x_position_hurdle_back - x0_knee) / vx) ** 2
                                  + vz * ((x_position_hurdle_back - x0_knee) / vx) + z0_knee)

            if i == 0:
                prog.AddConstraint((PARKOUR["height"][k] + PARKOUR["margin_obstacle_vertical"] - z_foot_hurdle_front) *
                                   PARKOUR["binarys_hurdle_front"][k][i] <= 0)
                prog.AddConstraint((PARKOUR["height"][k] + PARKOUR["margin_obstacle_vertical"] - z_knee_hurdle_back) *
                                   PARKOUR["binarys_hurdle_back"][k][i] <= 0)
            else:
                prog.AddConstraint((PARKOUR["height"][k] + PARKOUR["margin_obstacle_vertical"] - z_foot_hurdle_front) *
                                   (PARKOUR["binarys_hurdle_front"][k][i] -
                                    PARKOUR["binarys_hurdle_front"][k][i-1]) <= 0)
                prog.AddConstraint((PARKOUR["height"][k] + PARKOUR["margin_obstacle_vertical"] - z_knee_hurdle_back) *
                                   (PARKOUR["binarys_hurdle_back"][k][i] -
                                    PARKOUR["binarys_hurdle_back"][k][i-1]) <= 0)

        prog.AddConstraint(z == parkour_height)
        if i == N - 1:
            prog.AddConstraint(x == xg)
            # pass
        else:
            contact_x = x
            x0 = x + r * np.cos(theta[i+1]) - dx_hip_foot
            z0 = z + r * np.sin(theta[i+1]) - dz_hip_foot
            x0_knee = x + r * np.cos(theta[i+1]) - dx_hip_knee
            z0_knee = z + r * np.sin(theta[i+1]) - dz_hip_knee

    print(f'Constraints Set - Calculation Time: {round(time.time() - t_start, 4)}')

    # prog.AddCost()
    # prog.AddCost(sum(v))
    prog.AddCost(sum(t))
    # prog.AddCost(sum(v**2))
    # prog.AddCost(sum((v * np.cos(theta) * t))**2)
    # prog.AddCost((x-xg)**2 * 100)

    # ################################################### INITIAL GUESS ####################################################
    # print('Starting to set initial guess')
    # t_start = time.time()
    ig_theta = np.radians(65)
    dx = xg / N - np.cos(ig_theta) * r + dx_hip_foot
    dz = -(np.sin(ig_theta) * r - np.sin(theta_f) * r_f)
    ig_t = np.sqrt((-dz + dx * np.tan(ig_theta)) / (0.5 * g))
    ig_v = dx / (ig_t * np.cos(ig_theta))
    # ig_v = np.sqrt(xg * g / (2 * np.cos(ig_theta) * np.sin(ig_theta)))

    prog.SetInitialGuess(theta, np.ones(N) * ig_theta)
    prog.SetInitialGuess(t, np.ones(N) * ig_t)
    prog.SetInitialGuess(v, np.ones(N) * ig_v)
    # print(f'Initial Guess Set - Calculation Time: {round(time.time() - t_start, 4)}')

    print('Solver Engaged')
    t_start = time.time()
    result = MixedIntegerBranchAndBound(prog, SnoptSolver().solver_id())
    print(f'Solver Finished - Calculation Time: {round(time.time() - t_start, 4)}')

    # print(f'Delta 1 hurdle front: {result.GetSolution(delta_1_hurdle_front)}')
    # print(f'Delta 1 hurdle back: {result.GetSolution(delta_1_hurdle_back)}')
    # print(f'Delta 2 hurdle front: {result.GetSolution(delta_2_hurdle_front)}')
    # print(f'Delta 2 hurdle back: {result.GetSolution(delta_2_hurdle_back)}')

    ######################################### PLOTTING #########################################
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.set_xlabel("x")
    ax.set_ylabel("z")
    plt.grid(True)

    # PLOT NICE CURVE
    relative_t = np.linspace(0, 1, 20)
    relative_t_fide = np.linspace(0, 1, fide)
    contact_sequence_x = np.zeros(N+1)
    contact_sequence_z = np.zeros(N+1)
    X0 = r * np.cos(result.GetSolution(theta[0])) - dx_hip_foot
    Z0 = r * np.sin(result.GetSolution(theta[0])) - dz_hip_foot
    for i in range(N):
        T = result.GetSolution(t[i]) * relative_t
        VX = result.GetSolution(v[i]) * np.cos(result.GetSolution(theta[i]))
        VZ = result.GetSolution(v[i]) * np.sin(result.GetSolution(theta[i]))
        X = X0 + VX * T
        Z = Z0 + VZ * T - 0.5 * g * T**2
        X_K = X + dx_hip_foot - dx_hip_knee
        Z_K = Z + dz_hip_foot - dz_hip_knee
        ax.plot(X, Z, "b-")
        ax.plot(X, Z, "b-")
        ax.plot(X_K, Z_K, "y-")
        # fide
        T_fide = result.GetSolution(t[i]) * relative_t_fide
        X_fide = X0 + VX * T_fide
        Z_fide = Z0 + VZ * T_fide - 0.5 * g * T_fide**2
        ax.plot(X_fide, Z_fide, "bo")
        # SAVE FOOT LOCATION
        contact_sequence_x[i+1] = X[-1]
        contact_sequence_z[i+1] = Z[-1]
        if i < N-1:
            X0 = X[-1] + r * np.cos(result.GetSolution(theta[i+1])) - dx_hip_foot
            Z0 = Z[-1] + r * np.sin(result.GetSolution(theta[i+1])) - dz_hip_foot

    plot_stepsize = 0.0001
    parkour_plot_x = np.arange(x_start, xg + plot_stepsize, plot_stepsize)
    parkour_plot_obstacle_z = np.zeros(len(parkour_plot_x))

    for i in range(obstacle_number):
        start_point = (PARKOUR["position"][i] - PARKOUR["width"][i] / 2) / plot_stepsize
        end_point = (PARKOUR["position"][i] + PARKOUR["width"][i] / 2) / plot_stepsize
        parkour_plot_obstacle_z[int(start_point):int(end_point)] = PARKOUR["height"][i]
        parkour_plot_obstacle_z[[int(start_point), int(end_point)]] = (PARKOUR["height"][i] +
                                                                        PARKOUR["margin_obstacle_vertical"])

    ax.plot(parkour_plot_x, parkour_plot_obstacle_z, 'r')

    for i in range(obstacle_number):
        start_point = (PARKOUR["position"][i] - PARKOUR["width"][i] / 2 +
                       PARKOUR["margin_obstacle_horizontal"])
        end_point = (PARKOUR["position"][i] + PARKOUR["width"][i] / 2 -
                     PARKOUR["margin_obstacle_horizontal"])
        ax.plot([start_point, end_point], [PARKOUR["height"][i], PARKOUR["height"][i]], 'g', linewidth=3)

    print(f'Time: {repr(result.GetSolution(t))}')
    print(f'Velocities: {repr(result.GetSolution(v))}')
    print(f'Angles: {repr(np.degrees(result.GetSolution(theta)))}')
    print(f'Contact Sequence x: {repr(contact_sequence_x)}')
    print(f'Contact Sequence z: {repr(contact_sequence_z)}')
    print(f'Contact Binarys front: {repr(result.GetSolution(PARKOUR["binarys_hurdle_front"]))}')
    print(f'Contact Binarys Back: {repr(result.GetSolution(PARKOUR["binarys_hurdle_back"]))}')
    plt.show()

    return result.GetSolution(v), result.GetSolution(theta), contact_sequence_x, contact_sequence_z
