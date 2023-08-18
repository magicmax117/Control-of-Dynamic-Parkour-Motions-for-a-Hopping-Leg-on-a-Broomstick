import sys
import os
import time
import moteus
import asyncio
import numpy as np
import pandas as pd
from state_vector import xb_vector_id_tmotors, xb_vector_cad
from regressor_matrix import yb_matrix
from utils import data_prep as prep_data
from utils import data_plot as plot_data
from utils import error_calculation
from utils import error_plots
from utils import print_to_textfile


def rad2rev(angle_in_radians):
    return angle_in_radians * (1 / (2 * np.pi))


def rev2rad(angle_in_revolution):
    return angle_in_revolution * (2 * np.pi)


# if len(sys.argv) != 2:
#     print('Provide CAN device name (can0, slcan0 etc.)')
#     sys.exit(0)
# print("Using Socket {} for can communication".format(sys.argv[1],))


async def main():
    # Set motor parameters
    print("Motor enabled.")
    # Shoulder motor

    for i in range(numSteps):                                                      # compute estimated feedforward torque from cad data
        tau_ff_cad = np.dot(yb_matrix(-9.81, 0.3,
                                  [shoulder_pos_traj[i], elbow_pos_traj[i]],
                                  [shoulder_vel_traj[i], elbow_vel_traj[i]],
                                  [shoulder_acc_traj[i], elbow_acc_traj[i]]),
                                   xb_vector_cad())
        shoulder_tau_cad[i] = tau_ff_cad[0]
        elbow_tau_cad[i] = tau_ff_cad[1]

    print("Precomputing FF torque with the last identified parameters...")         # compute estimated feedforward torque from system identification
    for i in range(numSteps):
        tau_ff = np.dot(yb_matrix(-9.81, 0.3,
                                  [shoulder_pos_traj[i], elbow_pos_traj[i]],
                                  [shoulder_vel_traj[i], elbow_vel_traj[i]],
                                  [shoulder_acc_traj[i], elbow_acc_traj[i]]),
                        xb_vector_id_tmotors())
        shoulder_tau_traj[i] = tau_ff[0]
        elbow_tau_traj[i] = tau_ff[1]
    print("Sending Trajectories to Motors... ")
    motor_shoulder_controller = moteus.Controller(id=shoulder_id)
    # Elbow motor
    motor_elbow_controller = moteus.Controller(id=elbow_id)

    # stop both motors
    await motor_shoulder_controller.set_stop()
    await motor_elbow_controller.set_stop()

    print("Enabling Motors..")

    try:
        print("Start")
        t = 0.0
        realStartT = time.time()
        for i in range(numSteps):
            stepStartT = time.time()
            time_vec[i] = t
            # position control
            # print("Executing trajectory...")
            state1 = await motor_shoulder_controller.set_position(
                                           position=rad2rev(shoulder_pos_traj[i]),
                                           velocity=rad2rev(shoulder_vel_traj[i]),
                                           kp_scale=kp_scale_sh,  # 50.0, This was value for kp in tomotrs
                                           kd_scale=kd_scale_sh,  # 0.5, This was value for kd in tomotrs
                                           stop_position=None,
                                           feedforward_torque=shoulder_tau_cad[i], # None,
                                           maximum_torque=max_torque,
                                           watchdog_timeout=None,
                                           query=True)
            state2 = await motor_elbow_controller.set_position(
                                           position=rad2rev(elbow_pos_traj[i]),
                                           velocity=rad2rev(elbow_vel_traj[i]),
                                           kp_scale=kp_scale_el,  # 50.0, This was value for kp in tomotrs
                                           kd_scale=kd_scale_el,  # 0.5, This was value for kd in tomotrs
                                           stop_position=None,
                                           feedforward_torque=elbow_tau_cad[i],  # None, #
                                           maximum_torque=max_torque,
                                           watchdog_timeout=None,
                                           query=True)
            # store the measured sensor data of position, velocity and torque in each time step
            shoulder_position[i] = rev2rad(state1.values[moteus.Register.POSITION])
            shoulder_velocity[i] = rev2rad(state1.values[moteus.Register.VELOCITY])
            shoulder_torque[i] = state1.values[moteus.Register.TORQUE]
            elbow_position[i] = rev2rad(state2.values[moteus.Register.POSITION])
            elbow_velocity[i] = rev2rad(state2.values[moteus.Register.VELOCITY])
            elbow_torque[i] = state2.values[moteus.Register.TORQUE]

            t = t + dt
            elapsedTime = time.time() - stepStartT
            # print('First part finished')
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

            # shoulder_pos, shoulder_vel, shoulder_tau = \
            #     motor_shoulder_controller.send_rad_command(shoulder_pos_final,
            #                                                0.0,
            #                                                Kp_shoulder,
            #                                                Kd_shoulder,
            #                                                0.0)                    # send pos, vel and tau_ff command and use the in-built low level controller
            # elbow_pos, elbow_vel, elbow_tau = \
            #     motor_elbow_controller.send_rad_command(elbow_pos_final,
            #                                             0.0,
            #                                             Kp_elbow,
            #                                             Kd_elbow,
            #                                             0.0)
            state1 = await motor_shoulder_controller.set_position(
                                           position=rad2rev(shoulder_pos_final),
                                           velocity=None, #0.,
                                           kp_scale=kp_scale_sh,  # 50.0, This was value for kp in tomotrs
                                           kd_scale=None,  # 0.5, This was value for kd in tomotrs
                                           stop_position=None,
                                           feedforward_torque=None,
                                           maximum_torque=max_torque,
                                           watchdog_timeout=None,
                                           query=True)
            state2 = await motor_elbow_controller.set_position(
                                           position=rad2rev(elbow_pos_final),
                                           velocity=None, #0.0,
                                           kp_scale=kp_scale_el,  # 50.0, This was value for kp in tomotrs
                                           kd_scale=None,  # 0.5, This was value for kd in tomotrs
                                           stop_position=None,
                                           feedforward_torque=None,
                                           maximum_torque=max_torque,
                                           watchdog_timeout=None,
                                           query=True)
            # shoulder_position[i + numSteps] = shoulder_pos
            # shoulder_velocity[i + numSteps] = shoulder_vel
            # shoulder_torque[i + numSteps] = shoulder_tau
            # elbow_position[i + numSteps] = elbow_pos
            # elbow_velocity[i + numSteps] = elbow_vel
            # elbow_torque[i + numSteps] = elbow_tau
            shoulder_position[i + numSteps] = rev2rad(state1.values[moteus.Register.POSITION])
            shoulder_velocity[i + numSteps] = rev2rad(state1.values[moteus.Register.VELOCITY])
            shoulder_torque[i + numSteps] = state1.values[moteus.Register.TORQUE]
            elbow_position[i + numSteps] = rev2rad(state2.values[moteus.Register.POSITION])
            elbow_velocity[i + numSteps] = rev2rad(state2.values[moteus.Register.VELOCITY])
            elbow_torque[i + numSteps] = state2.values[moteus.Register.TORQUE]

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

    except Exception as e:
        print(e)
    finally:
        print("Disabling Motors...")
        os.system(f"sudo moteus_tool --stop -t{shoulder_id},{elbow_id}")
        await motor_shoulder_controller.set_stop()
        await motor_elbow_controller.set_stop()

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


if __name__ == "__main__":
    shoulder_id = 1
    elbow_id = 2
    os.system(f"sudo moteus_tool --zero-offset -t{shoulder_id},{elbow_id}")
    kp_scale_sh = 1
    kd_scale_sh = 1
    kp_scale_el = 1
    kd_scale_el = 1
    max_torque = 8
    provided_tau = "fftau_from_cad"   # fftau_from_sys_id, fftau_from_cad, =fftau

    # define input and output directories
    experiment = "20220204_param_id_excitation_traj_202406_mjbots"
    # input_directory = "../../data/trajectories/"
    # input_file = f"swingup_{frequency}Hz"        # cycloidal_trajectory_500Hz
    input_directory = "../../data/trajectories/excitation_traj_202406/"
    input_file = f'trajectory-pos-50-3x_{250}Hz'
    frequency = 250
    suffix = ".csv"
    data = pd.read_csv(os.path.join(input_directory, input_file + suffix))
    output_directory = "../../results/" + experiment + "/test_data/" + \
                       f'{input_file}' + "/gains_" + f'{500}' + "kp_" + \
                       f'{10}' + "kd_" + f'{provided_tau}'
    output = output_directory
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
    print(dt)
    if input("When ready press y:") == 'y':
        asyncio.run(main())




