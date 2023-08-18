import numpy as np
import matplotlib.pyplot as plt
# import hyrodyn
# from software.python.hopping_leg.analysis.plot import plot_task_space
from software.python.utils.plotter import Plotter
from software.python.hopping_leg.parcour_functions.hyrodyn_plotter import Hyrodyn

# inputs
name = 'self_short_v2'
# name = 'self_middle_v7'
name = 'full_run'
save = False
bound = False
if name == 'full_run':
    endtime = 20
else:
    endtime = 4

# set path
path = f'software/python/experiments/parcour/data/mjbots/final/{name}/{name}.csv'

# load encoder data from csv file
dtype={'names': ('time', 'phase', 'pitch_pos', 'yaw_pos', 'hip_pos', 'knee_pos',
                 'pitch_vel', 'yaw_vel', 'hip_vel', 'knee_vel'),
       'formats': ['f8', 'S10', 'f4', 'f4', 'f4', 'f4', 'f4', 'f4', 'f4', 'f4']}
data_encoder = np.loadtxt(path, dtype, skiprows=1, delimiter=",")

# load record data from csv file
path = f'software/python/experiments/parcour/data/mjbots/final/{name}/record.csv'
dtype={'names': ('time', 'hip_tau', 'knee_tau', 'hip_tau_des', 'knee_tau_des'),
       'formats': ['f8', 'f4', 'f4', 'f4', 'f4']}
data_record = np.loadtxt(path, dtype, skiprows=1, usecols=(0, 3, 6, 9, 12), delimiter=",")

# create data dictionary
index = -1
for i in range(len(data_encoder['time'])):
    if data_encoder['time'][i] > endtime:
        index = i
        break

data_dict = {"time": data_encoder['time'][0:index],
             "phase": data_encoder['phase'][0:index],
             "pitch_pos": data_encoder['pitch_pos'][0:index],
             "yaw_pos": data_encoder['yaw_pos'][0:index],
             "hip_pos": -data_encoder['hip_pos'][0:index],
             "hip_vel": -data_encoder['hip_vel'][0:index],
             "hip_tau": -np.clip(data_record['hip_tau'][0:index], -10, 10),
             "hip_tau_des": -np.clip(data_record['hip_tau_des'][0:index], -10, 10),
             "knee_pos": -data_encoder['knee_pos'][0:index],
             "knee_vel": -data_encoder['knee_vel'][0:index],
             "knee_tau": -np.clip(data_record['knee_tau'][0:index], -10, 10),
             "knee_tau_des": -np.clip(data_record['knee_tau_des'][0:index], -10, 10),
             }
data_dict["hip_tau"][0:2] = 0
data_dict["hip_tau_des"][0:2] = 0
data_dict["knee_tau"][0:2] = 0
data_dict["knee_tau_des"][0:2] = 0
# create plots
plotter = Plotter(data_dict, endtime, name, save, bound)
hyrodyn = Hyrodyn(data_dict, endtime, name, save, bound)
hyrodyn.Endeffector()
# plotter.encoder()
# plotter.actuator()
# plotter.torque()



















# load record data from csv file
# path = f'software/python/experiments/parcour/data/mjbots/final/{name}/record.csv'
# dtype={'names': ('time', 'hip_pos', 'hip_vel', 'hip_tau', 'knee_pos', 'knee_vel', 'knee_tau',
#                  'hip_pos_des', 'hip_vel_des', 'hip_tau_des', 'knee_pos_des', 'knee_vel_des', 'knee_tau_des',
#                  'phase'),#, 'base_msr', 'base_est'),
#        'formats': ['f8', 'f4', 'f4', 'f4', 'f4', 'f4', 'f4',
#                    'f4', 'f4', 'f4', 'f4', 'f4', 'f4',
#                    'S10']}#, '', 'f4']}
# dtype={'names': ('time', 'hip_tau', 'knee_tau', 'hip_tau_des', 'knee_tau_des'),
#        'formats': ['f8', 'f4', 'f4', 'f4', 'f4']}
# data = np.loadtxt(path, dtype, skiprows=1, usecols=(0, 3, 6, 9, 12), delimiter=",")
# data_dict_record = {"time": data['time'],
#                     "hip_pos": data['hip_pos'],
#                     "hip_vel": data['hip_vel'],
#                     "hip_tau": data['hip_tau'],
#                     "hip_tau_des": data['hip_tau_des'],
#                     "knee_pos": data['knee_pos'],
#                     "knee_vel": data['knee_vel'],
#                     "knee_tau": data['knee_tau'],
#                     "knee_tau_des": data['knee_tau_des'],
#                     "phase": data['phase']
#                     }


# plotter = Plotter(data_dict_record)
# plotter.encoder()
# plotter.actuator()

# plot_task_space(plant,
#                 RECORD,
#                 save=False,
#                 show=PLOT,
#                 filename=os.path.join(RECORD.path, 'task_space') if hasattr(RECORD, 'path') else '')
# juhu = {
#         'names': ['Roll', 'Pitch', 'Yaw'],
#         'elements': [{'position': 2.60544839072185, 'speed': 0.0, 'effort': nan, 'raw': nan, 'acceleration': 0.0},
#         {'position': 0.37399170645429997, 'speed': 0.0, 'effort': nan, 'raw': nan, 'acceleration': 0.0},
#         {'position': 1.9205439455744, 'speed': 0.0, 'effort': nan, 'raw': nan, 'acceleration': 0.0}],
#         'time': {'microseconds': 1685537059059823}
#         }

# print(data_dict_record["time"][300:600] == data_dict_encoder["time"][300:600])

# plt.figure(figsize=(15, 10))
# plt.plot(data_dict_record["time"], data_dict_record["time"]**3)
# plt.plot(data_dict_encoder["time"], data_dict_encoder["time"]**3)
# plt.show()
