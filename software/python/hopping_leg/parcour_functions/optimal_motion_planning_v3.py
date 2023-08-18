import matplotlib.pyplot as plt
import numpy as np
from pydrake.all import MathematicalProgram, MixedIntegerBranchAndBound, SnoptSolver, IpoptSolver, NloptSolver, GurobiSolver

def motion_planning():
    def obstacle_builder(position, height, width, degree=2):
        points_x = [position - width / 2, position, position + width / 2]
        points_y = [0, height, 0]
        coefficients = np.polyfit(points_x, points_y, degree)
        polynom = np.poly1d(coefficients)
        return polynom, height


    obstacle_1_polynom, h1 = obstacle_builder(position=0.6, height=0.0, width=0.1)
    obstacle_1_hurdle_polynom, h1_hurdle = obstacle_builder(position=0.6, height=0.2, width=0.3)

    g = 9.80665
    # x0_hip = 0
    # z0_hip = 0
    xg = 1.2
    zg = 0
    r = 0.2
    r_f = 0.17
    r_s = 0.13
    theta_f = np.radians(93)
    fide = 10
    L1 = 0.15
    L2 = 0.14

    dx_hip_foot_flight = r_f * np.cos(theta_f)
    dz_hip_foot_flight = r_f * np.sin(theta_f)
    alpha = np.arccos((L2 ** 2 - L1 ** 2 - r_f ** 2) / (- 2 * L1 * r_f))
    gamma = alpha - (theta_f - np.radians(90))
    dz_hip_knee_flight = np.cos(gamma) * L1
    dx_hip_knee_flight = np.sin(gamma) * L1

    x_start = x0
    z_start = z0

    N = 3

    prog = MathematicalProgram()
    t = prog.NewContinuousVariables(N, "t")
    prog.AddBoundingBoxConstraint(0.3, 3, t)
    v = prog.NewContinuousVariables(N, "v")
    prog.AddBoundingBoxConstraint(0, 2.8, v)
    theta = prog.NewContinuousVariables(N, "theta")
    prog.AddBoundingBoxConstraint(np.radians(65), np.radians(90), theta)
    hurdle_1 = prog.NewBinaryVariables(fide * N, "hurdle_1")
    # hurdle_1_knee = prog.NewBinaryVariables(fide * N, "hurdle_1_knee")
    # hurdle_1 = prog.NewBinaryVariables(N, "hurdle_1")
    # spacer = prog.NewBinaryVariables((fide - 1) * N)

    x0_hip = r * np.cos(theta[0])
    z0_hip = r * np.sin(theta[0])
    for i in range(N):
        vx = v[i] * np.cos(theta[i])
        vz = v[i] * np.sin(theta[i])
        x_hip = x0_hip + vx * t[i]
        z_hip = z0_hip + t[i] * vz - 0.5 * g * t[i] * t[i]
        x_foot = x_hip - r_f * np.cos(theta_f)
        z_foot = z_hip - r_f * np.sin(theta_f)
        dh = t[i] / fide
        for j in range(fide - 1):
            x_foot_flight = x0_hip + vx * dh * (j + 1) - r_f * np.cos(theta_f)
            z_foot_flight = z0_hip + vz * dh * (j + 1) - 0.5 * g * dh * (j + 1) * dh * (j + 1) - dz_hip_foot_flight
            f1 = obstacle_1_hurdle_polynom(x_foot_flight)
            prog.AddConstraint(hurdle_1[fide * i + j] >= f1)
            prog.AddConstraint(hurdle_1[fide * i + j] * f1 >= 0)
            p_foot_flight = h1_hurdle * hurdle_1[fide * i + j]
            prog.AddConstraint(z_foot_flight >= p_foot_flight)


            # z_knee = z0 + vz * dh * (j + 1) - 0.5 * g * dh * (j + 1) * dh * (j + 1) - dz_hip_knee
            # prog.AddConstraint(z_knee >= p_knee)
        if i == N - 1:
            prog.AddConstraint(x_f == xg)
            prog.AddConstraint(z_f == zg)
        else:
            f1 = obstacle_1_polynom(x_f)
            prog.AddConstraint(hurdle_1[fide - 1 + fide * i] >= f1)
            prog.AddConstraint(hurdle_1[fide - 1 + fide * i] * f1 >= 0)
            p = h1 * hurdle_1[fide - 1 + fide * i]
            # prog.AddConstraint(hurdle_1[i] >= f1)
            # prog.AddConstraint(hurdle_1[i] * f1 >= 0)
            # p = h1 * hurdle_1[i]
            prog.AddConstraint(z_f >= p)
            prog.AddConstraint(z_f <= p)
            x0_hip = x_f + r * np.cos(theta[i+1])
            z0_hip = z_f + r * np.sin(theta[i+1])


    # prog.AddCost(sum(v))
    prog.AddCost(sum(t))
    # prog.AddCost(sum((v * np.cos(theta) * t))**2)

    # ################################################### INITIAL GUESS ####################################################
    ig_theta = np.radians(65)
    dx = xg / N - np.cos(ig_theta) * r + np.cos(theta_f) * r_f
    dz = np.sin(ig_theta) * r - np.sin(theta_f) * r_f
    ig_t = np.sqrt((-dz + dx * np.tan(ig_theta)) / (0.5 * g))
    ig_v = dx / (ig_t * np.cos(ig_theta))
    # ig_v = np.sqrt(xg * g / (2 * np.cos(ig_theta) * np.sin(ig_theta)))

    prog.SetInitialGuess(theta, np.ones(N) * ig_theta)
    prog.SetInitialGuess(t, np.ones(N) * ig_t)
    prog.SetInitialGuess(v, np.ones(N) * ig_v)

    solver = MixedIntegerBranchAndBound(prog, SnoptSolver().solver_id())
    result = solver.Solve()
    print(solver.GetSolution(t))
    print(solver.GetSolution(v))
    print(np.degrees(solver.GetSolution(theta)))
    print(np.degrees(solver.GetSolution(theta)))


    # plot the resulting trajectory
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.set_xlabel("x")
    ax.set_ylabel("z")
    plt.grid(True)
    # ax.set_xlim(x0-.1, xf+.1)
    ax.set_prop_cycle(plt.rcParams["axes.prop_cycle"])  # reset color cycle
    relative_t = np.linspace(0, 1, fide + 1)
    # relative_t = np.linspace(0, 1, 10)
    contact_sequence_x = np.zeros(N)
    X_c = 0
    X0 = r * np.cos(solver.GetSolution(theta[0]))
    Z0 = r * np.sin(solver.GetSolution(theta[0]))
    X_S = [r_s * np.cos(solver.GetSolution(theta[0])), X0]
    Z_S = [r_s * np.sin(solver.GetSolution(theta[0])), Z0]
    ax.plot(X_S, Z_S, "g.-")
    for i in range(N):
        T = solver.GetSolution(t[i]) * relative_t
        VX = solver.GetSolution(v[i]) * np.cos(solver.GetSolution(theta[i]))
        VZ = solver.GetSolution(v[i]) * np.sin(solver.GetSolution(theta[i]))
        X = X0 + VX * T
        Z = Z0 + VZ * T - 0.5 * g * T**2
        X_F = X - r_f * np.cos(theta_f)
        Z_F = Z - r_f * np.sin(theta_f)
        ax.plot(X, Z, "b.-")
        ax.plot(X_F, Z_F, "b.-")
        # SAVE FOOT LOCATION
        contact_sequence_x[i] = X_F[-1]
        if i < N-1:
            X0 = X_F[-1] + r * np.cos(solver.GetSolution(theta[i+1]))
            Z0 = Z_F[-1] + r * np.sin(solver.GetSolution(theta[i+1]))
            # STANCE PHASE PLOT
            X_S = [X_F[-1] + r_s * np.cos(solver.GetSolution(theta[i+1])), X0]
            Z_S = [Z_F[-1] + r_s * np.sin(solver.GetSolution(theta[i + 1])), Z0]
            ax.plot(X_S, Z_S, "g.-")
    ax.plot(0, 0, 'or')
    ax.plot(xg, zg, 'xr')

    parkour_function_x = np.arange(x_start, xg + 0.001, 0.001)
    parkour_function_z = np.zeros(len(parkour_function_x))
    i = 0
    for A in obstacle_1_polynom(parkour_function_x):
        if A - 1e-9 > 0:
            parkour_function_z[i] = h1
        i = i + 1

    parkour_function_hurdle_z = np.zeros(len(parkour_function_x))
    i = 0
    for A in obstacle_1_hurdle_polynom(parkour_function_x):
        if A - 1e-9 > 0:
            parkour_function_hurdle_z[i] = h1_hurdle
        i = i + 1
    # print(interpolated_polynominal(parkour_function_x))
    # ax.plot(parkour_function_x, np.array([0 for A in interpolated_polynominal(parkour_function_x) if A <= 0]))
    ax.plot(parkour_function_x, parkour_function_z, 'r')
    ax.plot(parkour_function_x, parkour_function_hurdle_z, 'y')
    print(f'Velocities: {solver.GetSolution(v)}')
    print(f'Angles: {np.degrees(solver.GetSolution(theta))}')
    print(f'Contact Sequence X: {contact_sequence_x}')
    plt.show()

    return solver.GetSolution(v), solver.GetSolution(theta), contact_sequence_x
