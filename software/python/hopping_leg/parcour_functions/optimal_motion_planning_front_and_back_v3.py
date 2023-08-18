import matplotlib.pyplot as plt
import numpy as np
from pydrake.all import MathematicalProgram, MixedIntegerBranchAndBound, SnoptSolver, IpoptSolver, NloptSolver, GurobiSolver
import time
from datetime import timedelta
from hopping_leg.parcour_functions.parkour_builder import polynom_generator

def motion_planning():

    g = 9.80665
    x0 = 0
    z0 = 0
    xg = 3.6
    zg = 0
    r = 0.2
    r_f = 0.17
    r_s = 0.13
    theta_f = np.radians(93)
    fide = 2
    L1 = 0.15
    L2 = 0.14

    dx_hip_foot = r_f * np.cos(theta_f)
    dz_hip_foot = r_f * np.sin(theta_f)
    alpha = np.arccos((L2 ** 2 - L1 ** 2 - r_f ** 2) / (- 2 * L1 * r_f))
    gamma = alpha - (theta_f - np.radians(90))
    dz_hip_knee = np.cos(gamma) * L1
    dx_hip_knee = np.sin(gamma) * L1
    print(dx_hip_knee)
    print(dz_hip_knee)
    x_start = x0
    z_start = z0

    N = 8

    # parkour creation
    PARKOUR = {"position": np.array([1]),#, 1.3, 2.3]),
               "height": np.array([0.1]),#, 0.3, 0.1]),
               "width": np.array([0.1]),#, 0.1, 0.1, 0.1, 0.1]),
               "hurdle_margin_front": 0,#0.05,
               "hurdle_margin_back": 0,#0.2,
               "platform_margin": 0}#0.025}

    # obstacle_polys, hurdle_polys, platform_polys = polynom_generator(PARKOUR)
    # PARKOUR["obstacle_polynom"] = obstacle_polys
    # PARKOUR["hurdle_polynom"] = hurdle_polys
    # PARKOUR["platform_polynom"] = platform_polys

    obstacle_number = len(PARKOUR["position"])

    # ----------------------------------------------- create program -----------------------------------------------
    prog = MathematicalProgram()
    t = prog.NewContinuousVariables(N, "t")
    prog.AddBoundingBoxConstraint(0.1, 3, t)
    v = prog.NewContinuousVariables(N, "v")
    prog.AddBoundingBoxConstraint(0.5, 2.8, v)
    theta = prog.NewContinuousVariables(N, "theta")
    prog.AddBoundingBoxConstraint(np.radians(65), np.radians(85), theta)

    skipper = prog.NewBinaryVariables(N, "skipper")

    binarys_front = [None] * len(PARKOUR["position"])
    binarys_back = [None] * len(PARKOUR["position"])
    for i in range(obstacle_number):
        name_front = f'delta_{i+1}_hurdle_front'
        name_back = f'delta_{i+1}_hurdle_back'
        binarys_front[i] = prog.NewBinaryVariables(N, name_front)
        binarys_back[i] = prog.NewBinaryVariables(N, name_back)
    PARKOUR["binarys_hurdle_front"] = binarys_front
    PARKOUR["binarys_hurdle_back"] = binarys_back

    # delta_1_hurdle_front = prog.NewBinaryVariables(N, "delta_1_hurdle_front")
    # delta_1_hurdle_back = prog.NewBinaryVariables(N, "delta_1_hurdle_back")
    # delta_2_hurdle_front = prog.NewBinaryVariables(N, "delta_2_hurdle_front")
    # delta_2_hurdle_back = prog.NewBinaryVariables(N, "delta_2_hurdle_back")
    # PARKOUR["binarys_hurdle_front"] = [delta_1_hurdle_front, delta_2_hurdle_front]
    # PARKOUR["binarys_hurdle_back"] = [delta_1_hurdle_back, delta_2_hurdle_back]

    contact_x = x0
    x0 = (r * np.cos(theta[0]) - dx_hip_foot) * skipper[i]
    z0 = (r * np.sin(theta[0]) - dz_hip_foot) * skipper[i]
    x0_knee = (r * np.cos(theta[0]) - dx_hip_knee) * skipper[i]
    z0_knee = (r * np.sin(theta[0]) - dz_hip_knee) * skipper[i]
    for i in range(N):
        vx = v[i] * np.cos(theta[i])
        vz = v[i] * np.sin(theta[i])
        x = x0 + (vx * t[i]) * skipper[i]
        z = z0 + (t[i] * vz - 0.5 * g * t[i] * t[i]) * skipper[i]
        parkour_height = 0
        for k in range(obstacle_number):
            x_position_hurdle_front = PARKOUR["position"][k] - PARKOUR["width"][k] / 2 - PARKOUR["hurdle_margin_front"]
            z_foot_hurdle_front = (- 0.5 * g * ((x_position_hurdle_front - x0) / vx) ** 2
                                   + vz * ((x_position_hurdle_front - x0) / vx) + z0)

            prog.AddConstraint(PARKOUR["binarys_hurdle_front"][k][i] * (x_position_hurdle_front - x) <= 0)
            prog.AddConstraint((1 - PARKOUR["binarys_hurdle_front"][k][i]) * (x - x_position_hurdle_front) <= 0)

            x_position_hurdle_back = PARKOUR["position"][k] + PARKOUR["width"][k] / 2 + PARKOUR["hurdle_margin_back"]
            z_foot_hurdle_back = (- 0.5 * g * ((x_position_hurdle_back - x0_knee) / vx) ** 2
                                  + vz * ((x_position_hurdle_back - x0_knee) / vx) + z0_knee)

            prog.AddConstraint(PARKOUR["binarys_hurdle_back"][k][i] * (x_position_hurdle_back - x) <= 0)
            prog.AddConstraint((1 - PARKOUR["binarys_hurdle_back"][k][i]) * (x - x_position_hurdle_back) <= 0)

            parkour_height += PARKOUR["height"][k] * (PARKOUR["binarys_hurdle_front"][k][i] -
                                                      PARKOUR["binarys_hurdle_back"][k][i])

            prog.AddConstraint(abs(x - x_position_hurdle_front) >= 0.02)
            prog.AddConstraint(abs(x - x_position_hurdle_back) >= 0.02)
            if i == 0:
                prog.AddConstraint((PARKOUR["height"][k] + 0.05 - z_foot_hurdle_front) *
                                   PARKOUR["binarys_hurdle_front"][k][i] <= 0)
                prog.AddConstraint((PARKOUR["height"][k] + 0.05 - z_foot_hurdle_back) *
                                   PARKOUR["binarys_hurdle_back"][k][i] <= 0)
            else:
                prog.AddConstraint((PARKOUR["height"][k] + 0.05 - z_foot_hurdle_front) *
                                   (PARKOUR["binarys_hurdle_front"][k][i] -
                                    PARKOUR["binarys_hurdle_front"][k][i-1]) <= 0)
                prog.AddConstraint((PARKOUR["height"][k] + 0.05 - z_foot_hurdle_back) *
                                   (PARKOUR["binarys_hurdle_back"][k][i] -
                                    PARKOUR["binarys_hurdle_back"][k][i-1]) <= 0)

        prog.AddConstraint(z == parkour_height)
        if i == N - 1:
            prog.AddConstraint(x == xg)
        else:
            contact_x = x
            x0 = x + (r * np.cos(theta[i+1]) - dx_hip_foot) * skipper[i]
            z0 = z + (r * np.sin(theta[i+1]) - dz_hip_foot) * skipper[i]
            x0_knee = x + (r * np.cos(theta[i+1]) - dx_hip_knee) * skipper[i]
            z0_knee = z + (r * np.sin(theta[i+1]) - dz_hip_knee) * skipper[i]


    # prog.AddCost(sum(v))
    # prog.AddCost(sum(t))
    prog.AddCost(sum(skipper))
    # prog.AddCost(sum((v * np.cos(theta) * t))**2)

    # ################################################### INITIAL GUESS ####################################################
    ig_theta = np.radians(65)
    dx = xg / N - np.cos(ig_theta) * r + dx_hip_foot
    dz = -(np.sin(ig_theta) * r - np.sin(theta_f) * r_f)
    ig_t = np.sqrt((-dz + dx * np.tan(ig_theta)) / (0.5 * g))
    ig_v = dx / (ig_t * np.cos(ig_theta))
    # ig_v = np.sqrt(xg * g / (2 * np.cos(ig_theta) * np.sin(ig_theta)))

    prog.SetInitialGuess(theta, np.ones(N) * ig_theta)
    prog.SetInitialGuess(t, np.ones(N) * ig_t)
    prog.SetInitialGuess(v, np.ones(N) * ig_v)
    # prog.SetInitialGuess(delta_1_hurdle_front, np.array([1, 0, 0]))
    # prog.SetInitialGuess(delta_1_hurdle_back, np.array([0, 1, 0]))
    # prog.SetInitialGuess(delta_1_platform, np.array([1, 0, 0]))
    # prog.SetInitialGuess(delta_1_platform, np.array([0, 1, 0, 0]))
    # prog.SetInitialGuess(delta_1_platform, np.array([0, 0, 1, 0]))
    # prog.SetInitialGuess(hurdle_1, np.array([0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]))  # np.ones((fide + 1) * N))
    # prog.SetInitialGuess(hurdle_2, np.array([0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]))  # np.ones((fide + 1) * N))

    solver = MixedIntegerBranchAndBound(prog, SnoptSolver().solver_id())
    print('Solver Engaged')
    t_start = time.time()
    result = solver.Solve()
    print(f'Solver Finished - Calculation Time: {round(time.time() - t_start, 4)}')
    print(f'Velocities: {solver.GetSolution(v)}')
    print(f'Angles: {np.degrees(solver.GetSolution(theta))}')
    print(f'Skipper: {solver.GetSolution(skipper)}')
    # print(f'Delta 1 hurdle front: {solver.GetSolution(delta_1_hurdle_front)}')
    # print(f'Delta 1 hurdle back: {solver.GetSolution(delta_1_hurdle_back)}')
    # print(f'Delta 2 hurdle front: {solver.GetSolution(delta_2_hurdle_front)}')
    # print(f'Delta 2 hurdle back: {solver.GetSolution(delta_2_hurdle_back)}')

    ######################################### PLOTTING #########################################
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.set_xlabel("x")
    ax.set_ylabel("z")
    plt.grid(True)

    # PLOT NICE CURVE
    relative_t = np.linspace(0, 1, 20)
    relative_t_fide = np.linspace(0, 1, fide)
    contact_sequence_x = np.zeros(N)
    X0 = r * np.cos(solver.GetSolution(theta[0])) - dx_hip_foot
    Z0 = r * np.sin(solver.GetSolution(theta[0])) - dz_hip_foot
    for i in range(N):
        T = solver.GetSolution(t[i]) * relative_t
        VX = solver.GetSolution(v[i]) * np.cos(solver.GetSolution(theta[i]))
        VZ = solver.GetSolution(v[i]) * np.sin(solver.GetSolution(theta[i]))
        X = X0 + VX * T
        Z = Z0 + VZ * T - 0.5 * g * T**2
        X_K = X + dx_hip_foot - dx_hip_knee
        Z_K = Z + dz_hip_foot - dz_hip_knee
        ax.plot(X, Z, "b-")
        ax.plot(X, Z, "b-")
        ax.plot(X_K, Z_K, "y-")
        # fide
        T_fide = solver.GetSolution(t[i]) * relative_t_fide
        X_fide = X0 + VX * T_fide
        Z_fide = Z0 + VZ * T_fide - 0.5 * g * T_fide**2
        ax.plot(X_fide, Z_fide, "bo")
        # SAVE FOOT LOCATION
        contact_sequence_x[i] = X[-1]
        if i < N-1:
            X0 = X[-1] + r * np.cos(solver.GetSolution(theta[i+1])) - dx_hip_foot
            Z0 = Z[-1] + r * np.sin(solver.GetSolution(theta[i+1])) - dz_hip_foot

    plot_stepsize = 0.0001
    parkour_plot_x = np.arange(x_start, xg + plot_stepsize, plot_stepsize)
    parkour_plot_obstacle_z = np.zeros(len(parkour_plot_x))
    parkour_plot_hurdle_z = np.zeros(len(parkour_plot_x))
    parkour_plot_platform_z = np.zeros(len(parkour_plot_x))

    for i in range(obstacle_number):
        # obstacle
        start_point = (PARKOUR["position"][i] - PARKOUR["width"][i] / 2) / plot_stepsize
        end_point = (PARKOUR["position"][i] + PARKOUR["width"][i] / 2) / plot_stepsize
        parkour_plot_obstacle_z[int(start_point):int(end_point)] = PARKOUR["height"][i]

        # hurdle
        start_point = (PARKOUR["position"][i] - PARKOUR["width"][i] / 2 - PARKOUR["hurdle_margin_front"]) / plot_stepsize
        end_point = (PARKOUR["position"][i] + PARKOUR["width"][i] / 2 + PARKOUR["hurdle_margin_back"]) / plot_stepsize
        parkour_plot_hurdle_z[int(start_point):int(end_point)] = PARKOUR["height"][i]

        # platform
        start_point = (PARKOUR["position"][i] - PARKOUR["width"][i] / 2 + PARKOUR["platform_margin"]) / plot_stepsize
        end_point = (PARKOUR["position"][i] + PARKOUR["width"][i] / 2 - PARKOUR["platform_margin"]) / plot_stepsize
        parkour_plot_platform_z[int(start_point):int(end_point)] = PARKOUR["height"][i]

    ax.plot(parkour_plot_x, parkour_plot_obstacle_z, 'r')
    ax.plot(parkour_plot_x, parkour_plot_hurdle_z, 'y--')
    ax.plot(parkour_plot_x, parkour_plot_platform_z, 'g--')

    plt.show()

    return solver.GetSolution(v), solver.GetSolution(theta), contact_sequence_x