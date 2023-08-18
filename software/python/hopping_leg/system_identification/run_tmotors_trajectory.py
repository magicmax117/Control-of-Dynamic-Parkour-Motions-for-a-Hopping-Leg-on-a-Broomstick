import sys
import os
import time
from motor_driver.canmotorlib import CanMotorController                                     # from motor_driver.canmotorlib import CanMotorController
import numpy as np
import pandas as pd
from state_vector import xb_vector_id_tmotors, xb_vector_large_second_link_cad
from regressor_matrix import yb_matrix
from utils import data_prep as prep_data
from utils import data_plot as plot_data
from utils import error_calculation
from utils import error_plots
from utils import print_to_textfile

if len(sys.argv) != 2:
    print('Provide CAN device name (can0, slcan0 etc.)')
    sys.exit(0)
print("Using Socket {} for can communication".format(sys.argv[1],))

# set motor parameters
can_port = 'can0'                                                              # CAN port
motor_shoulder_id = 0x08                                                       # motor id
motor_elbow_id = 0x09
Kp_shoulder = 200.0                                                            # motor gains
Kd_shoulder = 2.0
Kp_elbow = 200.0 
Kd_elbow = 2.0 
provided_tau = "zero_fftau"   # fftau_from_sys_id, fftau_from_cad, zero_fftau

# define input and output directories
experiment = "20220331_verify_params_tmotors_second_link_0p4m"
input_directory = "../../data/trajectories/"
input_file = "swingup_500Hz"      # cycloidal_trajectory_500Hz swingup_500Hz
suffix = ".csv"
data = pd.read_csv(os.path.join(input_directory, input_file + suffix))
output_directory = "../../results/" + experiment + "/" + f'{input_file}'  
os.makedirs(output_directory)

# get trajectory from csv file
print("Reading Trajectory from " + f'{input_directory }' + f'{input_file}' + \
      f'{suffix}')
des_time_vec = data["time"]
dt = data["time"][1] - data["time"][0]
shoulder_pos_traj = data["shoulder_pos"]
shoulder_vel_traj = data["shoulder_vel"]
shoulder_acc_traj = data["shoulder_acc"]
#shoulder_acc_traj = np.diff(np.diff(shoulder_pos_traj)/dt)/dt                 # if no acceleration is given in the csv file
#shoulder_acc_traj = np.append(shoulder_acc_traj, np.array([0., 0.]))
#shoulder_tau_traj = data["shoulder_torque"]
elbow_pos_traj = data["elbow_pos"]
elbow_vel_traj = data["elbow_vel"]
elbow_acc_traj = data["elbow_acc"]
#elbow_acc_traj = np.diff(np.diff(elbow_pos_traj)/dt)/dt
#elbow_acc_traj = np.append(elbow_acc_traj, np.array([0., 0.]))
#elbow_tau_traj = data["elbow_torque"]

numSteps = len(data)
numSteps_Stabilization = 0
total_num_steps = numSteps + numSteps_Stabilization
time_vec = np.zeros(total_num_steps)                                           # create empty arrays to store measured data faster
shoulder_position = np.zeros(total_num_steps)
elbow_position = np.zeros(total_num_steps)
shoulder_velocity = np.zeros(total_num_steps)
elbow_velocity = np.zeros(total_num_steps)
shoulder_torque = np.zeros(total_num_steps)
elbow_torque = np.zeros(total_num_steps)
shoulder_tau_traj = np.zeros(numSteps)
elbow_tau_traj = np.zeros(numSteps)
shoulder_tau_cad = np.zeros(numSteps)
elbow_tau_cad = np.zeros(numSteps)

for i in range(numSteps):                                                      # compute estimated feedforward torque from cad data
    tau_ff_cad = np.dot(yb_matrix(-9.81, 6, 0.3,
                              [shoulder_pos_traj[i], elbow_pos_traj[i]],
                              [shoulder_vel_traj[i], elbow_vel_traj[i]],
                              [shoulder_acc_traj[i], elbow_acc_traj[i]]),
                               xb_vector_large_second_link_cad())
    shoulder_tau_cad[i] = tau_ff_cad[0]
    elbow_tau_cad[i] = tau_ff_cad[1]

print("Precomputing FF torque with the last identified parameters...")         # compute estimated feedforward torque from system identification
for i in range(numSteps):
    tau_ff = np.dot(yb_matrix(-9.81, 6, 0.3,
                              [shoulder_pos_traj[i], elbow_pos_traj[i]],
                              [shoulder_vel_traj[i], elbow_vel_traj[i]],
                              [shoulder_acc_traj[i], elbow_acc_traj[i]]),
                    xb_vector_id_tmotors())
    shoulder_tau_traj[i] = tau_ff[0]
    elbow_tau_traj[i] = tau_ff[1]
print("Sending Trajectories to Motors... ")


def setZeroPosition(motor, initPos):
    pos = initPos
    while abs(np.rad2deg(pos)) > 0.5:
        pos, vel, curr = motor.set_zero_position()
        print("Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos),
                                                              np.rad2deg(vel),
                                                              curr))


motor_shoulder_controller = CanMotorController(sys.argv[1], motor_shoulder_id)
motor_elbow_controller = CanMotorController(sys.argv[1], motor_elbow_id)

print("Enabling Motors..")
shoulder_pos, shoulder_vel, shoulder_tau = \
    motor_shoulder_controller.enable_motor()
print("Shoulder Motor Status: Pos: {}, Vel: {}, "
      "Torque: {}".format(shoulder_pos, shoulder_vel, shoulder_tau))
elbow_pos, elbow_vel, elbow_tau = motor_elbow_controller.enable_motor()
print("Elbow Motor Status: Pos: {}, Vel: {}, "
      "Torque: {}".format(elbow_pos, elbow_vel, elbow_tau))
print("Setting Shoulder Motor to Zero Position...")
setZeroPosition(motor_shoulder_controller, shoulder_pos)
print("Setting Elbow Motor to Zero Position...")
setZeroPosition(motor_elbow_controller, elbow_pos)

try:
    print("Start")
    t = 0.0
    realStartT = time.time()
    for i in range(numSteps):
        stepStartT = time.time()
        time_vec[i] = t

        shoulder_pos, shoulder_vel, shoulder_tau = \
            motor_shoulder_controller.send_rad_command(shoulder_pos_traj[i],
                                                       shoulder_vel_traj[i],
                                                       Kp_shoulder,            # motor gains
                                                       Kd_shoulder,
                                                       0.0)   # shoulder_tau_traj[i] send pos, vel and tau_ff command and use the in-built low level controller
        elbow_pos, elbow_vel, elbow_tau = \
            motor_elbow_controller.send_rad_command(elbow_pos_traj[i],
                                                    elbow_vel_traj[i],
                                                    Kp_elbow,
                                                    Kd_elbow,
                                                    0.0)         # elbow_tau_traj[i]
        shoulder_position[i] = shoulder_pos
        shoulder_velocity[i] = shoulder_vel
        shoulder_torque[i] = shoulder_tau   
        elbow_position[i] = elbow_pos
        elbow_velocity[i] = elbow_vel
        elbow_torque[i] = elbow_tau

        t = t + dt
        elapsedTime = time.time() - stepStartT

        if elapsedTime > dt:
            print("Loop Index {} takes longer than expected dt "
                  "of {}.".format(i, dt))
        while time.time() - stepStartT < dt:
            pass

    shoulder_pos_final = shoulder_pos_traj[numSteps - 1]
    elbow_pos_final = elbow_pos_traj[numSteps - 1]

    for i in range(numSteps_Stabilization):
        stepStartT = time.time()
        time_vec[i + numSteps] = t

        shoulder_pos, shoulder_vel, shoulder_tau = \
            motor_shoulder_controller.send_rad_command(shoulder_pos_final,
                                                       0.0,
                                                       Kp_shoulder,
                                                       Kd_shoulder,
                                                       0.0)                    # send pos, vel and tau_ff command and use the in-built low level controller
        elbow_pos, elbow_vel, elbow_tau = \
            motor_elbow_controller.send_rad_command(elbow_pos_final,
                                                    0.0,
                                                    Kp_elbow,
                                                    Kd_elbow,
                                                    0.0)
        shoulder_position[i + numSteps] = shoulder_pos
        shoulder_velocity[i + numSteps] = shoulder_vel
        shoulder_torque[i + numSteps] = shoulder_tau
        elbow_position[i + numSteps] = elbow_pos
        elbow_velocity[i + numSteps] = elbow_vel
        elbow_torque[i + numSteps] = elbow_tau

        shoulder_pos_traj[i + numSteps] = shoulder_pos_final
        shoulder_vel_traj[i + numSteps] = 0.0
        shoulder_acc_traj[i + numSteps] = 0.0
        elbow_pos_traj[i + numSteps] = elbow_pos_final
        elbow_vel_traj[i + numSteps] = 0.0
        elbow_acc_traj[i + numSteps] = 0.0

        t = t + dt
        elapsedTime = time.time() - stepStartT

        if elapsedTime > dt:
            print("Loop Index {} takes longer than expected "
                  "dt of {}.".format(i, dt))
        while time.time() - stepStartT < dt:
            pass

    realEndT = time.time()
    realdT = (realEndT - realStartT) / numSteps

    print("End. New dt: {}".format(realdT))
    print("Disabling Motors...")
    shoulder_pos, shoulder_vel, shoulder_tau = \
        motor_shoulder_controller.disable_motor()
    print("Shoulder Motor Status: Pos: {}, Vel: {}, "
          "Torque: {}".format(shoulder_pos, shoulder_vel, shoulder_tau))
    elbow_pos, elbow_vel, elbow_tau = motor_elbow_controller.disable_motor()
    print("Elbow Motor Status: Pos: {}, Vel: {}, "
          "Torque: {}".format(elbow_pos, elbow_vel, elbow_tau))

except Exception as e:
    print(e)
    print("Disabling Motors...")
    shoulder_pos, shoulder_vel, shoulder_tau = \
        motor_shoulder_controller.disable_motor()
    print("Shoulder Motor Status: Pos: {}, Vel: {}, "
          "Torque: {}".format(shoulder_pos, shoulder_vel, shoulder_tau))
    elbow_pos, elbow_vel, elbow_tau = motor_elbow_controller.disable_motor()
    print("Elbow Motor Status: Pos: {}, Vel: {}, "
          "Torque: {}".format(elbow_pos, elbow_vel, elbow_tau))

shoulder_acceleration = np.diff(np.diff(shoulder_position)/dt)/dt
elbow_acceleration = np.diff(np.diff(elbow_position)/dt)/dt

measured_csv_data = np.array([np.array(time_vec[:-2]),
                              np.array(shoulder_position[:-2]),
                              np.array(shoulder_velocity[:-2]),
                              np.array(shoulder_acceleration),
                              np.array(shoulder_torque[:-2]),
                              np.array(elbow_position[:-2]),
                              np.array(elbow_velocity[:-2]),
                              np.array(elbow_acceleration),
                              np.array(elbow_torque[:-2])]).T
measured_data_filepath = os.path.join(output_directory, "measured.csv")
np.savetxt(measured_data_filepath, measured_csv_data,
           delimiter=',',
           header="time,shoulder_pos,shoulder_vel,shoulder_acc,"
                  "shoulder_torque,elbow_pos,elbow_vel,elbow_acc,elbow_torque",
           comments="")

desired_csv_data = np.array([np.array(des_time_vec),
                             np.array(shoulder_pos_traj),
                             np.array(shoulder_vel_traj),
                             np.array(shoulder_acc_traj),
                             np.array(shoulder_tau_traj),
                             np.array(shoulder_tau_cad),
                             np.array(elbow_pos_traj),
                             np.array(elbow_vel_traj),
                             np.array(elbow_acc_traj),
                             np.array(elbow_tau_traj),
                             np.array(elbow_tau_cad)]).T
desired_data_filepath = os.path.join(output_directory, "desired.csv")
np.savetxt(desired_data_filepath, desired_csv_data,
           delimiter=',',
           header="time,shoulder_pos,shoulder_vel,shoulder_acc,"
                  "shoulder_tau_id,shoulder_tau_cad,elbow_pos,elbow_vel,"
                  "elbow_acc,elbow_tau_id,shoulder_tau_cad",
           comments="")

"""
Show the plots
"""
meas_t, meas_shoulder_pos, meas_shoulder_vel, meas_shoulder_acc, \
    meas_shoulder_trq, meas_elbow_pos, meas_elbow_vel, meas_elbow_acc, \
    meas_elbow_trq = prep_data.import_measured_data(measured_data_filepath)    # load measured data from csv file

des_t, des_shoulder_pos, des_shoulder_vel, des_shoulder_acc, \
    des_shoulder_trq_id, des_shoulder_trq_cad, des_elbow_pos, \
    des_elbow_vel, des_elbow_acc, des_elbow_trq_id, des_elbow_trq_cad \
    = prep_data.import_desired_data(desired_data_filepath)                     # load desired data from csv file

filt_t, filt_shoulder_pos, filt_elbow_pos, filt_shoulder_vel, shoulder_vel2, \
    shoulder_vel3, filt_elbow_vel, elbow_vel2, elbow_vel3, filt_shoulder_acc, \
    shoulder_acc,  shoulder_acc2, shoulder_acc3, filt_elbow_acc, elbow_acc, \
    elbow_acc2, elbow_acc3, filt_shoulder_trq, shoulder_trq2, filt_elbow_trq, \
    elbow_trq2 = prep_data.smooth_data(meas_t,
                                       meas_shoulder_pos,
                                       meas_shoulder_vel,
                                       meas_shoulder_trq,
                                       meas_elbow_pos,
                                       meas_elbow_vel,
                                       meas_elbow_trq)                         # smooth noisy measurements

plot_data.sys_id_and_cad_estimate(output_directory, meas_t,
                                  des_shoulder_pos, meas_shoulder_pos,
                                  des_elbow_pos, meas_elbow_pos,
                                  des_shoulder_vel, meas_shoulder_vel, filt_shoulder_vel,
                                  des_elbow_vel, meas_elbow_vel, filt_elbow_vel,
                                  des_shoulder_acc, meas_shoulder_acc, filt_shoulder_acc,
                                  des_elbow_acc, meas_elbow_acc, filt_elbow_acc,
                                  des_shoulder_trq_id, des_shoulder_trq_cad, meas_shoulder_trq, filt_shoulder_trq,
                                  des_elbow_trq_id, des_elbow_trq_cad, meas_elbow_trq, filt_elbow_trq)  # plot commanded vs measured joint positions, velocities, accelerations and torques

mae_dict, rmse_dict \
    = error_calculation.mae_rmse_traj_tracking(des_shoulder_pos, meas_shoulder_pos,
                                               des_elbow_pos, meas_elbow_pos,
                                               des_shoulder_vel, meas_shoulder_vel,
                                               des_elbow_vel, meas_elbow_vel,
                                               des_shoulder_acc, filt_shoulder_acc,
                                               des_elbow_acc, filt_elbow_acc,
                                               des_shoulder_trq_id, des_shoulder_trq_cad, meas_shoulder_trq,
                                               des_elbow_trq_id, des_elbow_trq_cad, meas_elbow_trq)   # calculate mean absolute and root mean square error between measured and desired trajectory data

print_to_textfile.mae_rsme_tracking(output_directory, mae_dict, rmse_dict)     # save mae and rmse values to text file

error_plots.torque_mae_rmse(output_directory, mae_dict, rmse_dict)

