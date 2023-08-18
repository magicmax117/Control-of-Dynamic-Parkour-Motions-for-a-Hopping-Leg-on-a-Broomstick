import numpy as np
import matplotlib.pyplot as plt
import hyrodyn


# urdf = '/home/dfki.uni-bremen.de/malbracht/PycharmProjects/hopping_leg/model/legV2/boomstick/urdf/BS_T_003_000_000_hyrodyn.urdf'
urdf = '/home/dfki.uni-bremen.de/malbracht/PycharmProjects/hopping_leg/model/legV2/boomstick/urdf/BS_hyrodyn.urdf'
mech = '/home/dfki.uni-bremen.de/malbracht/PycharmProjects/hopping_leg/model/legV2/boomstick/config/hydrodyn_test.yaml'
body = 'Link_Endeffector'

robot = hyrodyn.RobotModel(urdf, mech)

# forward kinematics
robot.calculate_forward_kinematics(body)
print("Forward kinematics of the body " + body + " ([X Y Z Qx Qy Qz Qw]):", robot.pose)
steps = 100
yaw = np.linspace(0, 0.3 * np.pi, steps)
y = np.zeros(12)
X = np.zeros(steps)
Y = np.zeros(steps)
Z = np.zeros(steps)
for i in range(steps):
    y[1] = yaw[i] - np.radians(3.0)
    y[2] = np.radians(15.29)  # np.radians(7.34)
    y[9] = 1.432306623788595967
    y[10] = -2.110821417578966130
    robot.y = y
    robot.calculate_forward_kinematics(body)
    X[i] = robot.pose[0, 0]
    Y[i] = robot.pose[0, 1]
    Z[i] = robot.pose[0, 2]

# plt.axes(projection='3d')
# plt.plot(X, Y, Z)
# plt.axis('scaled')
# plt.xlabel('x')
# plt.ylabel('y')
# plt.show()

U = Y[0] * 2 * np.pi
print(U)

yaw = np.arctan(X/-Y)
x_park = 7.2 * yaw/(2 * np.pi)
plt.figure()
plt.plot(x_park, Z)
plt.grid(visible=True)
plt.show()
# print("Input joint config = ", robot.y)
# robot.calculate_forward_kinematics(body)
# print("Forward kinematics of the body " + body + " ([X Y Z Qx Qy Qz Qw]):", robot.pose)

# keys = robot.jointnames_spanningtree
# values = robot.Q[0].tolist()
# config_dict = dict(zip(keys, values))
# print(config_dict)
# robot.y = np.array([1, 2, 3])
# robot.calculate_system_state()
# print(robot.Q)
