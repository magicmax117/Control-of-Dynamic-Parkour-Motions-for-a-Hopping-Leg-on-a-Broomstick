import numpy as np
import matplotlib.pyplot as plt
import hyrodyn

class Hyrodyn:
    def __init__(self, data_dict, endtime, name, save, bound):
        self.data = data_dict
        self.endtime = endtime
        self.name = name
        self.save = save
        self.bound = bound
        if name == 'self_short_v2':
            self.yaw_cor = np.radians(3.25)
            self.pitch_cor = np.radians(15.29)
        elif name == 'self_middle_v7':
            self.yaw_cor = np.radians(3.25)
            self.pitch_cor = np.radians(15.29)
        elif name == 'full_run':
            self.yaw_cor = np.radians(3.25)
            self.pitch_cor = np.radians(15.29)

    def Endeffector(self):
        # urdf = '/home/dfki.uni-bremen.de/malbracht/PycharmProjects/hopping_leg/model/legV2/boomstick/urdf/BS_T_003_000_000_hyrodyn.urdf'
        urdf = '/home/dfki.uni-bremen.de/malbracht/PycharmProjects/hopping_leg/model/legV2/boomstick/urdf/BS_hyrodyn.urdf'
        mech = '/home/dfki.uni-bremen.de/malbracht/PycharmProjects/hopping_leg/model/legV2/boomstick/config/hydrodyn_test.yaml'
        body = 'Link_Endeffector'

        robot = hyrodyn.RobotModel(urdf, mech)

        # forward kinematics
        robot.calculate_forward_kinematics(body)
        print("Forward kinematics of the body " + body + " ([X Y Z Qx Qy Qz Qw]):", robot.pose)
        steps = len(self.data["time"])
        y = np.zeros(12)
        X = np.zeros(steps)
        Y = np.zeros(steps)
        Z = np.zeros(steps)
        for i in range(steps):
            y[1] = self.data["yaw_pos"][i] + self.yaw_cor
            y[2] = self.pitch_cor - self.data["pitch_pos"][i]
            y[9] = self.data["hip_pos"][i]
            y[10] = self.data["knee_pos"][i]
            robot.y = y
            robot.calculate_forward_kinematics(body)
            X[i] = robot.pose[0, 0]
            Y[i] = robot.pose[0, 1]
            Z[i] = robot.pose[0, 2]

        plt.axes(projection='3d')
        plt.plot(X, Y, Z)
        plt.axis('scaled')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.show()

        U = Y[0] * 2 * np.pi
        print(U)

        # R = np.sqrt(X**2 + Y**2)
        # yaw = np.sin(X/R)
        yaw = np.arctan(X/-Y)
        factor = 1
        gate = -1
        switch = True
        for i in range(len(yaw)):
            if yaw[i] <= gate and switch:
                yaw[i:] += np.pi
                gate = np.pi
                switch = False
            if yaw[i] >= np.pi and switch is False:
                switch = True
                factor += 2
        print(yaw)

        x_park = 7.2 * yaw/(2 * np.pi)
        plt.figure()
        plt.plot(x_park, Z)
        plt.show()
        savepath =f'/home/dfki.uni-bremen.de/malbracht/PycharmProjects/hopping_leg/software/python/experiments/parcour/data/plot/{self.name}_plot.csv'
        fmt = ['%.18e', '%.18e', '%.18e', '%.18e']
        data = np.hstack((X[:, np.newaxis], Y[:, np.newaxis], Z[:, np.newaxis], x_park[:, np.newaxis]))
        np.savetxt(savepath, data, fmt, delimiter=',', header=f'X,Y,Z,x_park', comments="")
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
