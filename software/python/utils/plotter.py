import numpy as np
import matplotlib as mlp
import matplotlib.pyplot as plt
from matplotlib.patches import Patch


class Plotter:
    def __init__(self, data_dict, endtime, name, save, bound):
        self.data = data_dict
        self.endtime = endtime
        self.name = name
        self.save = save
        self.bound = bound

    def encoder(self):
        plt.figure(figsize=(15, 10))
        plt.subplot(2, 1, 1)
        plt.plot(self.data["time"], np.degrees(self.data["pitch_pos"]))
        plt.ylabel('Pitch angle [deg]')
        plt.xlabel('Time [sec]')
        plt.grid(True)
        plt.subplot(2, 1, 2)
        plt.plot(self.data["time"], np.degrees(self.data["yaw_pos"]))
        plt.ylabel('Yaw angle [deg]')
        plt.xlabel('Time [sec]')
        plt.grid(True)
        plt.show()
        if self.save:
            plt.savefig(f'software/python/experiments/parcour/figures/{self.name}/encoder.png')

    def actuator(self):
        plt.figure(figsize=(15, 10))
        plt.subplot(4, 1, 1)
        plt.plot(self.data["time"], np.degrees(self.data["hip_pos"]))
        plt.ylabel('Hip angle [deg]')
        plt.xlabel('Time [sec]')
        plt.grid(True)
        plt.subplot(4, 1, 2)
        plt.plot(self.data["time"], np.degrees(self.data["knee_pos"]))
        plt.ylabel('Knee angle [deg]')
        plt.xlabel('Time [sec]')
        plt.grid(True)
        plt.subplot(4, 1, 3)
        plt.plot(self.data["time"], np.degrees(self.data["hip_vel"]))
        plt.ylabel('Hip velocity [deg/s]')
        plt.xlabel('Time [sec]')
        plt.grid(True)
        plt.subplot(4, 1, 4)
        plt.plot(self.data["time"], np.degrees(self.data["knee_vel"]))
        plt.ylabel('Knee velocity [deg/s]')
        plt.xlabel('Time [sec]')
        plt.grid(True)
        plt.show()

    def torque(self):
        fontsize = 14
        plt.figure(figsize=(15, 10))
        ax1 = plt.subplot(2, 1, 1)
        plt.plot(self.data["time"], self.data["hip_tau_des"], label='Desired', linewidth=2)
        plt.plot(self.data["time"], self.data["hip_tau"], label='Measured', linewidth=1)
        if self.bound:
            plt.plot(self.data["time"], np.ones(len(self.data["time"])) * 10)
            plt.plot(self.data["time"], np.ones(len(self.data["time"])) * -10)
        # plt.xlabel('Time [sec]')
        plt.ylabel('Hip Torque [Nm]', fontsize=fontsize)
        plt.ylim([-11, 11])
        plt.xlim([0, self.endtime])
        plt.xticks(fontsize=fontsize)
        plt.yticks(fontsize=fontsize)
        leg1 = plt.legend(fontsize=fontsize, loc='upper left')
        plt.tick_params('x', labelbottom=False)

        ax2 = plt.subplot(2, 1, 2)
        plt.plot(self.data["time"], self.data["knee_tau_des"], label='Desired', linewidth=2)
        plt.plot(self.data["time"], self.data["knee_tau"], label='Measured', linewidth=1)
        if self.bound:
            plt.plot(self.data["time"], np.ones(len(self.data["time"])) * 10)
            plt.plot(self.data["time"], np.ones(len(self.data["time"])) * -10)
        plt.xlabel('Time [sec]', fontsize=12)
        plt.ylabel('Knee Torque [Nm]', fontsize=12)
        plt.xlim([0, self.endtime])
        plt.ylim([-11, 11])
        plt.xticks(fontsize=fontsize)
        plt.yticks(fontsize=fontsize)
        leg2 = plt.legend(fontsize=fontsize, loc='upper left')

        phase = self.data["phase"][0]
        start = 0
        # cmap = np.array(['Greens', 'Oranges', 'Blues'])
        cmap = np.array(['spring', 'Wistia', 'winter'])
        color = np.array(['g', 'r', 'c'])
        Z = np.array([0.4, 0.1, 0.3])
        count = 0
        for i in range(len(self.data["time"])):
            if self.data["phase"][i] != phase or i == len(self.data["time"])-1:
                phase = self.data["phase"][i]
                if (self.data["phase"][i] == b'EXERTION' or self.data["phase"][i] == b'FLIGHT'
                        or self.data["phase"][i] == b'TOUCHDOWN' or i == len(self.data["time"])-1):
                    ax1.pcolor([self.data["time"][start], self.data["time"][i]], [-15, 15],
                               [[Z[int(np.remainder(count, 3))]]], vmin=0, vmax=1,
                               cmap='Set1', alpha=0.3)
                    ax2.pcolor([self.data["time"][start], self.data["time"][i]], [-15, 15],
                               [[Z[int(np.remainder(count, 3))]]], vmin=0, vmax=1,
                               cmap='Set1', alpha=0.3)
                    count += 1
                    start = i
        mlp.rcParams['grid.color'] = 'black'
        ax1.grid()
        ax2.grid()
        ax1.legend([Patch(facecolor=plt.cm.Set1(Z[0]), alpha=0.3), Patch(facecolor=plt.cm.Set1(Z[1]), alpha=0.3),
                    Patch(facecolor=plt.cm.Set1(Z[2]), alpha=0.3)],
                   ['Stance', 'Exertion', 'Flight'], loc='upper center', fancybox=True,
                   bbox_to_anchor=(0.5, 1.17), ncol=3, fontsize=fontsize)
        ax1.add_artist(leg1)
        plt.show()
