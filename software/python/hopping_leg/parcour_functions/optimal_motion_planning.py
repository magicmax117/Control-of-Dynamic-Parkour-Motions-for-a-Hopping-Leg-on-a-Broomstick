import matplotlib.pyplot as plt
import numpy as np
from pydrake.all import MathematicalProgram, MixedIntegerBranchAndBound, SnoptSolver


def motion_planning():
    def obstacle_builder(position, height, width, degree=2):
        points_x = [position - width / 2, position, position + width / 2]
        points_y = [0, height, 0]
        coefficients = np.polyfit(points_x, points_y, degree)
        polynom = np.poly1d(coefficients)
        return polynom, height

    margin_x = 0.01
    margin_z = 0.05

    obstacle_1_polynom, h1 = obstacle_builder(position=0.3, height=0.3, width=0.1)
    obstacle_2_polynom, h2 = obstacle_builder(position=1, height=0.15, width=0.1)
    obstacle_3_polynom, h3 = obstacle_builder(position=1.5, height=0.4, width=0.1)

    obstacle_1_polynom_collision, h1_collision = obstacle_builder(position=0.3, height=0.3, width=0.1 + 2 * margin_x)
    obstacle_2_polynom_collision, h2_collision = obstacle_builder(position=1, height=0.15, width=0.1 + 2 * margin_x)
    obstacle_3_polynom_collision, h3_collision = obstacle_builder(position=1.5, height=0.4, width=0.1 + 2 * margin_x)

    g = 9.80665
    x0 = 0
    z0 = 0
    xg = 1.5
    zg = 0.4

    fide = 10
    dist = 0.05

    x_start = x0
    z_start = z0

    N = 3
    prog = MathematicalProgram()
    x = prog.NewContinuousVariables(N, "x")
    prog.AddBoundingBoxConstraint(0.01, 1, x)
    v = prog.NewContinuousVariables(N, "v")
    prog.AddBoundingBoxConstraint(0.1, 3, v)
    theta = prog.NewContinuousVariables(N, "theta")
    prog.AddBoundingBoxConstraint(np.radians(60), np.radians(89), theta)
    hurdle_1 = prog.NewBinaryVariables(fide * N, "hurdle_1")
    hurdle_2 = prog.NewBinaryVariables(fide * N, "hurdle_2")
    hurdle_3 = prog.NewBinaryVariables(fide * N, "hurdle_3")
    # spacer = prog.NewBinaryVariables((fide - 1) * N)

    for i in range(N):
        h = (x[i]) / (v[i] * np.cos(theta[i]))
        v_landing = v[i] - g * h
        # theta_landing = 0.5 * np.arcsin((g * x[i]) / v_landing**2)
        # prog.AddConstraint((x[i]) / (v_landing * h) <= 0.9)
        # prog.AddConstraint((x[i]) / (v_landing * h) >= -0.9)
        # theta_landing = np.arccos((x[i]) / (v_landing * h))
        # prog.AddConstraint(theta_landing >= np.radians(50))
        # x = x0 + h[i] * v[i] * np.cos(theta[i])
        z = z0 + h * v[i] * np.sin(theta[i]) - 0.5 * g * h * h
        dx = x[i] / fide
        for j in range(fide - 1):
            t = (dx * (j + 1)) / (v[i] * np.cos(theta[i]))
            collision_z = z0 + v[i] * np.sin(theta[i]) * t - 0.5 * g * t * t
            f1 = obstacle_1_polynom(x0 + dx * (j + 1))
            prog.AddConstraint(hurdle_1[fide * i + j] >= f1)
            prog.AddConstraint(hurdle_1[fide * i + j] * f1 >= 0)
            f2 = obstacle_2_polynom(x0 + dx * (j + 1))
            prog.AddConstraint(hurdle_2[fide * i + j] >= f2)
            prog.AddConstraint(hurdle_2[fide * i + j] * f2 >= 0)
            f3 = obstacle_3_polynom(x0 + dx * (j + 1))
            prog.AddConstraint(hurdle_3[fide * i + j] >= f3)
            prog.AddConstraint(hurdle_3[fide * i + j] * f3 >= 0)
            p = h1 * hurdle_1[fide * i + j] + h2 * hurdle_2[fide * i + j] + h3 * hurdle_3[fide * i + j]
            prog.AddConstraint(collision_z >= p + 0.05)
            # prog.AddConstraint(collision_z >= interpolated_polynominal(x0 + dx * (j + 1)) - 1e-9)
        if i == N - 1:
            prog.AddConstraint(x[i] == xg - x0)
            prog.AddConstraint(z == zg)
        else:
            x_park = x[i]+x0
            f1 = obstacle_1_polynom(x_park)
            prog.AddConstraint(hurdle_1[fide - 1 + fide * i] >= f1)
            prog.AddConstraint(hurdle_1[fide - 1 + fide * i] * f1 >= 0)
            f2 = obstacle_2_polynom(x_park)
            prog.AddConstraint(hurdle_2[fide - 1 + fide * i] >= f2)
            prog.AddConstraint(hurdle_2[fide - 1 + fide * i] * f2 >= 0)
            f3 = obstacle_3_polynom(x_park)
            prog.AddConstraint(hurdle_3[fide - 1 + fide * i] >= f3)
            prog.AddConstraint(hurdle_3[fide - 1 + fide * i] * f3 >= 0)
            p = h1 * hurdle_1[fide - 1 + fide * i] + h2 * hurdle_2[fide - 1 + fide * i] + h3 * hurdle_3[fide - 1 + fide * i]
            prog.AddConstraint(z == p)
            x0 = x0 + x[i]
            z0 = z


    prog.AddCost(sum(v))
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
    i = 0
    for A in obstacle_2_polynom(parkour_function_x):
        if A - 1e-9 > 0:
            parkour_function_z[i] = h2
        i = i + 1
    i = 0
    for A in obstacle_3_polynom(parkour_function_x):
        if A - 1e-9 > 0:
            parkour_function_z[i] = h3
        i = i + 1
    # print(interpolated_polynominal(parkour_function_x))
    # ax.plot(parkour_function_x, np.array([0 for A in interpolated_polynominal(parkour_function_x) if A <= 0]))
    ax.plot(parkour_function_x, parkour_function_z)
    print(np.degrees(solver.GetSolution(theta[0])))
    plt.show()

    return solver.GetSolution(v), solver.GetSolution(theta)
