import numpy as np
from pyrock import NameService, RTT


class Encoder:

    def __init__(self, timesteps):
        ns = NameService()
        task = ns.get_task_context("joints_drv")
        self.reader = task.reader("joints_status")
        self.time = np.zeros(timesteps)
        self.pitch_pos = np.zeros(timesteps)
        self.pitch_vel = np.zeros(timesteps)
        self.yaw_pos = np.zeros(timesteps)
        self.yaw_vel = np.zeros(timesteps)
        # status, self.data = self.reader.read(copy_old_data=True, return_status=True)
        # print(self.data["names"])
        # self.stick_state = np.zeros((1, len(self.data_init["names"])))
        # self.calibration = np.zeros((1, len(self.data_init["names"])))
        # for i in range(len(data["names"])):
        #     self.calibration[i] = data["elements"][i]["position"]

    def read(self, n):
        status, data = self.reader.read(copy_old_data=True, return_status=True)
        if n == 0:
            self.calibration = data
            self.stick_state = np.zeros((2, len(data["names"])))
        for i in range(len(data["names"])):
            self.stick_state[0, i] = -(data["elements"][i]["position"] - self.calibration["elements"][i]["position"])
            self.stick_state[1, i] = data["elements"][i]["speed"]
            # stick_state[2, i] = data["elements"][i]["acceleration"]
        self.pitch_pos[n] = self.stick_state[0, 1]
        self.pitch_vel[n] = self.stick_state[1, 1]
        self.yaw_pos[n] = self.stick_state[0, 2]
        self.yaw_vel[n] = self.stick_state[1, 2]
        return self.stick_state

    def save(self, path, record, n):
        # data_dict = {"pitch_pos": self.pitch_pos,
        #              "yaw_pos": self.yaw_pos,
        #              }
        time = record.t
        hid = record.motor_names[0]
        # hpd, hvd, htd = record.matrix_des(0)
        hpm, hvm, htm = record.matrix_msr(0)
        kid = record.motor_names[1]
        # kpd, kvd, ktd = record.matrix_des(0)
        kpm, kvm, ktm = record.matrix_msr(1)
        # print(record.phase)
        fmt = ['%.18e', '%s', '%.18e', '%.18e', '%.18e',  '%.18e', '%.18e', '%.18e', '%.18e', '%.18e']
        data = np.hstack((time[:n+1, np.newaxis], record.phase[:n+1, np.newaxis],
                          self.pitch_pos[:n+1, np.newaxis], self.yaw_pos.T[:n+1, np.newaxis],
                          hpm[:n+1, np.newaxis], kpm[:n+1, np.newaxis],
                          self.pitch_vel[:n+1, np.newaxis], self.yaw_vel[:n+1, np.newaxis],
                          hvm[:n+1, np.newaxis], kvm[:n+1, np.newaxis]))
        # print(data)
        np.savetxt(path, data, fmt, delimiter=',', header=f'time,phase,pitch_pos,yaw_pos,{hid},{kid},'
                                                          f'pitch_vel,yaw_vel,{hid}_vel,{kid}_vel', comments="")
