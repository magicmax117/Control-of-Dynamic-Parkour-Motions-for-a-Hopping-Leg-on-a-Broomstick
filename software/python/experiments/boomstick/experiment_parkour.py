"""
Experiment
==========
"""

import os
import sys
import shutil
from importlib import import_module

from os.path import join, abspath
sys.path.append('/'.join(abspath(__file__).split('/')[:-5]))

import toml
import numpy as np

# CONTROL LOOP
from spine.utils.log import log_experiment

# HOPPING LEG
from hopping_leg.parcour_functions.optimal_motion_planning_front_and_back_v2 import motion_planning
from hopping_leg.plant.plant                     import HopperPlant
from hopping_leg.analysis.plot             import plot_task_space

from utils.plotter import Plotter

"""
Nomenclature
============

Terms
-----

Variables
---------

"""

########################################################################################################################
# SETTINGS #############################################################################################################
########################################################################################################################
# command line argument
assert isinstance(sys.argv[1], str), 'you must provide the index of the experiment you want to run'
system = {
    '0': 'mjbots',
    '1': 'tmotor',
    '2': 'pybullet'
}[sys.argv[1]]

OMP             = True  # optimal motion planning
obstacle_course = 2
ENCODER         = True
FEEDBACK        = True
BACKFLIP        = False
EXPERIMENT      = 'nahaufnahmen_3'
CONTROLLER      = 'PARCOUR'
savedir         = 'parcour'
SAVE            = False
PLOT            = False
VIDEO           = False
TIMESTAMP       = False
VERBOSE         = False
duration        = 20  # saved data contains samples until (duration - 1) seconds
record_time     = 0

if system == 'pybullet':
    from hopping_leg.state_machines.parcour_pb import StateMachine
    GAINS          = 'experiments/parcour/pb_gains.toml'
    INITIAL_PHASE  = 'STAGING'
    CONTACT_EST    = 'PB'
    ENCODER        = False
else:
    from hopping_leg.state_machines.parcour_pb import StateMachine
    GAINS          = 'experiments/parcour/gains.toml'
    INITIAL_PHASE  = 'STAGING'
    CONTACT_EST    = 'ET'
    from spine.motors import load
    motors = load(f'experiments/robots/legV2_pure_torque.toml')
    control_freq = motors[0].control_freq

# simulation
g              = -9.80665
mu = 2
timestep_delay = False

# independendent variables
independent_variables = [
    ('phase', object),
    'base_height_msr',
    'base_height_est'
]

# log
log_experiment(EXPERIMENT, f'{system} - LEG V2')
path = f'experiments/parcour/data/mjbots/final/{EXPERIMENT}.csv'

# verbose printing function
def verbose(*args, **kwargs):
    if VERBOSE:
        print(*args, **kwargs)
    else:
        pass


########################################################################################################################
# PLANT ################################################################################################################
########################################################################################################################
qd_star = [0, 0]

knee_direction = 1

L1   = 0.15
L2   = 0.14
mass = [0.91281,   1.2804,     0.13037]
Izz  = [0.0015899, 6.3388E-05]
com1 = [0.059331,  1.3564E-05]
com2 = [0.078298,  1.088E-08]

plant = HopperPlant(mass=mass,
                    Izz=Izz,
                    com1=com1,
                    com2=com2,
                    link_length=[L1, L2],
                    gravity=g)


########################################################################################################################
# Control ##############################################################################################################
########################################################################################################################
# initial state
initial_state = plant.kinematics_new_frame(0.13, np.radians(110))

# jumping dict
N = 2
touchdown_r = 0.15
touchdown_theta = np.radians(95)
staging_r = 0.14
flight_r = 0.17
flight_theta = np.array(np.radians([87]))

if OMP:
    launch_velocity, des_theta, contact_sequence_x, contact_sequence_z = motion_planning(obstacle_course)
    staging_theta = np.radians(180) - des_theta

number_of_jumps = len(staging_theta)
staging_theta = np.append(staging_theta, np.radians(100))
flight_theta = np.ones(number_of_jumps) * np.radians(87)
if BACKFLIP:
    flight_theta[-1] -= 2 * np.pi
    staging_theta[-1] -= 2 * np.pi

jumping_dict = {"number_of_jumps": number_of_jumps,  # len(contact_sequence_x),
                "launch_velocity": launch_velocity,
                "touchdown_r": touchdown_r,
                "touchdown_theta": touchdown_theta,
                "staging_r": staging_r,
                "staging_theta": staging_theta,
                "exertion_d": exertion_d,
                "flight_r": flight_r,
                "flight_theta": flight_theta,
                "contact_sequence_x": contact_sequence_x,
                "contact_sequence_z": contact_sequence_z}

print(f'Launch Velocity: {jumping_dict["launch_velocity"]}')
print(f'Theta: {np.degrees(jumping_dict["staging_theta"])}')
print(f'Contact Sequence x: {jumping_dict["contact_sequence_x"]}')
print(f'Contact Sequence z: {jumping_dict["contact_sequence_z"]}')

contact_threshold = np.ones(number_of_jumps)
for i in range(number_of_jumps):
    if contact_sequence_z[i+1] - contact_sequence_z[i] >= 0.2:
        contact_threshold[i] = 0.5

# controller
control_freq = 130
controller = StateMachine(duration,
                          control_freq,
                          plant,
                          knee_direction,
                          qd_star,
                          CONTROLLER,
                          GAINS,
                          INITIAL_PHASE,

                          CONTACT_EST,
                          contact_threshold,
                          jumping_dict,
                          ENCODER,
                          FEEDBACK,
                          path)
controller.initial_state = plant.kinematics_new_frame(staging_r, staging_theta[0])
# if system != 'pybullet':
    # controller.initial_state = initial_state
# controller.final_state   = [0, 0]


######################################################################################################################
# EXECUTION ######################################################################################################################

if system == 'mjbots':

    import asyncio
    # from spine.motors import load
    from spine.mjbots import motor_control_loop

    # motors = load(f'experiments/robots/legV2_pure_torque.toml')

    RECORD = asyncio.run(motor_control_loop(controller,
                                            motors,
                                            duration,
                                            independent_variables=independent_variables,
                                            plot=PLOT,
                                            save=SAVE,
                                            directory=f'experiments/{savedir}/data/mjbots',
                                            series='final',
                                            name=EXPERIMENT,
                                            timestamp=True))

if system == 'tmotor':

    # from spine.motors import load
    from spine.tmotor import motor_control_loop

    # motors = load(f'experiments/robots/legV2_pure_torque.toml')

    RECORD = motor_control_loop(controller,
                                motors,
                                duration,
                                independent_variables=independent_variables,
                                plot=PLOT,
                                save=SAVE,
                                directory=f'experiments/{savedir}/data/tmotor/',
                                series='',
                                name='',
                                timestamp=False)

if system == 'pybullet':
    import pybullet as pb

    from spine.sim          import load
    from spine.sim.pybullet import simulation
    from spine.pybullet     import motor_control_loop
    from hopping_leg.simulation.pybullet_p1 import legV2_boomstick as model

    # load
    urdf, motors = load(f'experiments/robots/bs_legV2_pybullet_pure_torque.toml')

    # set up
    client = simulation([0, 0, g],
                        mu,
                        motors)

    free_joints = [2, 3]
    robot  = model(client,
                   urdf,
                   free_joints=free_joints)

    # start experiment
    RECORD = motor_control_loop(controller,
                                client,
                                pb.TORQUE_CONTROL,
                                robot,
                                motors,
                                timestep_delay,
                                duration,
                                independent_variables=independent_variables,
                                countdown=0,
                                plot=PLOT,
                                save=SAVE,
                                video=VIDEO,
                                directory=f'experiments/{savedir}/data/pybullet/{CONTROLLER.lower()}/V2',
                                series='',
                                name='',
                                timestamp=False)

dtype={'names': ('time', 'phase', 'pitch_pos', 'yaw_pos', 'hip_pos', 'knee_pos',
                 'pitch_vel', 'yaw_vel', 'hip_vel', 'knee_vel'),
       'formats': ['f8', 'S10', 'f8', 'f8', 'f8', 'f8', 'f8', 'f8', 'f8', 'f8']}

# load trajectories from csv file
data_dict = np.loadtxt(path, dtype, skiprows=1, delimiter=",")

plotter = Plotter(data_dict)
plotter.encoder()
plotter.actuator()

# plot_task_space(plant,
#                 RECORD,
#                 save=SAVE,
#                 show=PLOT,
#                 filename=os.path.join(RECORD.path, 'task_space') if hasattr(RECORD, 'path') else '')
