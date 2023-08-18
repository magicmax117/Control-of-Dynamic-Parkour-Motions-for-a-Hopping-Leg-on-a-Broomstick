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


    obstacle_1_polynom, h1 = obstacle_builder(position=0.4, height=0.15, width=0.1)
    obstacle_1_polynom_c, h1 = obstacle_builder(position=0.4, height=0.15, width=0.2)
    g = 9.80665
    x0 = 0
    z0 = 0
    xg = 0.8
    zg = 0

    fide = 10

    x_start = x0
    z_start = z0

    N = 2

    prog = MathematicalProgram()
    x = prog.NewContinuousVariables(N, "x")
    prog.AddBoundingBoxConstraint(0.01, 1, x)
    v = prog.NewContinuousVariables(N, "v")
    prog.AddBoundingBoxConstraint(0.1, 2.5, v)
    theta = prog.NewContinuousVariables(N, "theta")
    prog.AddBoundingBoxConstraint(np.radians(65), np.radians(85), theta)
    hurdle_1 = prog.NewBinaryVariables(N, "hurdle_1")
    obstacle_1 = prog.NewBinaryVariables((fide-2) * N, "obstacle_1")
    # hurdle_1 = prog.NewBinaryVariables(N, "hurdle_1")
    # spacer = prog.NewBinaryVariables((fide - 1) * N)

    for i in range(N):
        h = (x[i]) / (v[i] * np.cos(theta[i]))
        z = z0 + h * v[i] * np.sin(theta[i]) - 0.5 * g * h * h
        dx = x[i] / fide
        for j in range(fide - 2):
            t = (dx * (j + 2)) / (v[i] * np.cos(theta[i]))
            collision_z = z0 + v[i] * np.sin(theta[i]) * t - 0.5 * g * t * t
            f1 = obstacle_1_polynom_c(x0 + dx * (j + 1))
            prog.AddConstraint(obstacle_1[(fide-2) * i + j] >= f1)
            prog.AddConstraint(obstacle_1[(fide-2) * i + j] * f1 >= 0)
            p = (h1 + 0.05) * obstacle_1[(fide-2) * i + j]
            prog.AddConstraint(collision_z >= p)
        if i == N - 1:
            prog.AddConstraint(x[i] >= xg - x0 - 1e-3)
            prog.AddConstraint(x[i] <= xg - x0 + 1e-3)
            prog.AddConstraint(z >= zg - 1e-3)
            prog.AddConstraint(z <= zg + 1e-3)
        else:
            x_park = x[i]+x0
            f1 = obstacle_1_polynom(x_park)
            prog.AddConstraint(hurdle_1[i] >= f1)
            prog.AddConstraint(hurdle_1[i] * f1 >= 0)
            p = h1 * hurdle_1[i]
            # prog.AddConstraint(hurdle_1[i] >= f1)
            # prog.AddConstraint(hurdle_1[i] * f1 >= 0)
            # p = h1 * hurdle_1[i]
            prog.AddConstraint(z >= p - 1e-3)
            prog.AddConstraint(z <= p + 1e-3)
            x0 = x0 + x[i]
            z0 = z


    # prog.AddCost(sum(v))
    # prog.AddCost(x[0]**2 + x[1]**2 + x[2]**2)
    prog.AddCost(sum(v))
    # prog.AddCost(sum(x/ (v * np.cos(theta))))
    # ################################################### INITIAL GUESS ####################################################
    ig_x = xg / N
    ig_theta = np.radians(60)
    ig_v = np.sqrt(xg * g / (2 * np.cos(ig_theta) * np.sin(ig_theta)))
    prog.SetInitialGuess(x, np.ones(N) * ig_x)
    prog.SetInitialGuess(theta, np.ones(N) * ig_theta)
    prog.SetInitialGuess(v, np.ones(N) * ig_v)
    solver = MixedIntegerBranchAndBound(prog, SnoptSolver().solver_id())
    result = solver.Solve()

    # plot the resulting trajectory
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.set_xlabel("x")
    ax.set_ylabel("z")
    plt.grid(True)
    # ax.set_xlim(x0-.1, xf+.1)
    ax.set_prop_cycle(plt.rcParams["axes.prop_cycle"])  # reset color cycle
    relative_x = np.linspace(0, 1, fide + 1)
    X_c = 0

    for i in range(N):
        X = solver.GetSolution(x[i]) * relative_x
        t = (X) / (solver.GetSolution(v[i]) * np.cos(solver.GetSolution(theta[i])))
        # x = x_start + result.GetSolution(v[i]) * np.cos(result.GetSolution(theta[i])) * t
        Z = z_start + solver.GetSolution(v[i]) * np.sin(solver.GetSolution(theta[i])) * t - 0.5 * g * t * t
        ax.plot(X if i == 0 else X + X_c, Z, ".-")
        for j in range(fide - 3):
            ax.plot(X[j+2] if i == 0 else X[j+2] + X_c, Z[j+2], "ro")
        X_c = X_c + X[-1]
        z_start = Z[-1]
    ax.plot(0, 0, 'or')
    ax.plot(xg, zg, 'xr')

    parkour_function_x = np.arange(x_start, xg, 0.01)
    parkour_function_z = np.zeros(len(parkour_function_x))
    i = 0
    for A in obstacle_1_polynom(parkour_function_x):
        if A - 1e-9 > 0:
            parkour_function_z[i] = h1
        i = i + 1
    ax.plot(parkour_function_x, parkour_function_z)
    i = 0
    for A in obstacle_1_polynom_c(parkour_function_x):
        if A - 1e-9 > 0:
            parkour_function_z[i] = h1 + 0.05
        i = i + 1
    # print(interpolated_polynominal(parkour_function_x))
    # ax.plot(parkour_function_x, np.array([0 for A in interpolated_polynominal(parkour_function_x) if A <= 0]))
    ax.plot(parkour_function_x, parkour_function_z)
    print(f'Velocities: {solver.GetSolution(v)}')
    print(f'Angles: {np.degrees(solver.GetSolution(theta))}')
    plt.show()

    return solver.GetSolution(v), solver.GetSolution(theta)
