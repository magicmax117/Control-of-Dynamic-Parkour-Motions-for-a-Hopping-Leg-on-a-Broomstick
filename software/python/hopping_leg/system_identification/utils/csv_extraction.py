import os
import pandas as pd
import numpy as np
from pydrake.all import PiecewisePolynomial


def rad2rev(angle_in_radians):
    return angle_in_radians * (1 / (2 * np.pi))


def rev2rad(angle_in_revolution):
    return angle_in_revolution * (2 * np.pi)


def read_data(folder, file, up_directory):
    path_to_file = generate_path(folder, file, up_directory)
    data = pd.read_csv(path_to_file)
    n = len(data)
    return data, n


def prepare_data(data, n):
    # # empty arrays for measured data
    # (shoulder_meas_pos,
    #  shoulder_meas_vel,
    #  shoulder_meas_tau,
    #  shoulder_cmd_tau,
    #  elbow_meas_pos,
    #  elbow_meas_vel,
    #  elbow_meas_tau,
    #  elbow_cmd_tau,
    #  meas_time) = prepare_empty_arrays(n)
    # desired trajectory data
    ## shoulder
    des_time = data["time"]
    shoulder_pos = data["shoulder_pos"]#.to_numpy()
    shoulder_vel = data["shoulder_vel"]#.to_numpy()
    shoulder_acc = data["shoulder_acc"]#.to_numpy()
    # shoulder_jerk = data["shoulder_jerk"]#.to_numpy()
    # shoulder_tau = data["shoulder_torque"]#.to_numpy()
    ## elbow
    elbow_pos = data["elbow_pos"]#.to_numpy()
    elbow_vel = data["elbow_vel"]#.to_numpy()
    elbow_acc = data["elbow_acc"]#.to_numpy()
    # elbow_jerk = data["elbow_jerk"]#.to_numpy()
    # elbow_tau = data["elbow_torque"]#.to_numpy()
    # converting the desired trajectories according to the gear ratio
    # desired position in revolutions at the output shaft
    shoulder_pos_out = [rad2rev(x) for x in shoulder_pos]
    elbow_pos_out = [rad2rev(x) for x in elbow_pos]
    # desired velocity in revolutions/s at the output shaft
    shoulder_vel_out = [rad2rev(x) for x in shoulder_vel]
    elbow_vel_out = [rad2rev(x) for x in elbow_vel]
    dt = data["time"][2] - data["time"][1]
    return (shoulder_pos,
            shoulder_vel,
            shoulder_acc,
            # shoulder_jerk,
            # shoulder_tau,
            elbow_pos,
            elbow_vel,
            elbow_acc,
            # elbow_jerk,
            # elbow_tau,
            des_time,
            shoulder_pos_out,
            shoulder_vel_out,
            elbow_pos_out,
            elbow_vel_out,
            # shoulder_meas_pos,
            # shoulder_meas_vel,
            # shoulder_meas_tau,
            # shoulder_cmd_tau,
            # elbow_meas_pos,
            # elbow_meas_vel,
            # elbow_meas_tau,
            # elbow_cmd_tau,
            # meas_time,
            n,
            dt)


def prepare_empty_arrays(n):
    shoulder_meas_pos = np.zeros(n)
    shoulder_meas_vel = np.zeros(n)
    shoulder_meas_tau = np.zeros(n)
    shoulder_cmd_tau = np.zeros(n)
    elbow_meas_pos = np.zeros(n)
    elbow_meas_vel = np.zeros(n)
    elbow_meas_tau = np.zeros(n)
    elbow_cmd_tau = np.zeros(n)
    meas_time = np.zeros(n)
    return (shoulder_meas_pos,
            shoulder_meas_vel,
            shoulder_meas_tau,
            shoulder_cmd_tau,
            elbow_meas_pos,
            elbow_meas_vel,
            elbow_meas_tau,
            elbow_cmd_tau,
            meas_time)


def parent(path):
    return os.path.dirname(path)


def generate_path(path_to_folder, file_name, up_directory_counter):
    cur_dir = os.path.realpath(os.curdir)
    tmp_dir = cur_dir
    i = 0
    while i < up_directory_counter:
        tmp_dir = parent(tmp_dir)
        i += 1
    main_dir = tmp_dir
    return os.path.join(main_dir, path_to_folder, file_name)


def construct_nominal_trajectories(des_time,
                                   shoulder_des_pos,
                                   elbow_des_pose,
                                   shoulder_des_vel,
                                   elbow_des_vel):#,
                                   # des_tau):
    # des_tau = des_tau.values.reshape(des_tau.shape[0], -1).T
    des_time = des_time.values.reshape(des_time.shape[0], -1)
    x0_desc = np.vstack((shoulder_des_pos,
                         elbow_des_pose,
                         shoulder_des_vel,
                         elbow_des_vel))
    x0 = PiecewisePolynomial.CubicShapePreserving(des_time,
                                                  x0_desc,
                                                  zero_end_point_derivatives=True)
    # u0 = PiecewisePolynomial.FirstOrderHold(des_time, des_tau)
    return x0  # , u0


def extract_data_from_polynomial(polynomial, frequency):
    n_points = int(polynomial.end_time() / (1 / frequency))
    time_traj = np.linspace(polynomial.start_time(),
                            polynomial.end_time(),
                            n_points)
    extracted_time = time_traj.reshape(n_points, 1).T
    extracted_data = np.hstack([polynomial.value(t) for t in
                                np.linspace(polynomial.start_time(),
                                            polynomial.end_time(),
                                            n_points)])
    return extracted_data, extracted_time


if __name__ == "__main__":
    trajectory_folder = f'data/trajectories/excitation_traj_202406'
    file_name = f'trajectory-pos-50-3x.csv'
    data, n = read_data(folder=trajectory_folder,
                        file=file_name,
                        up_directory=2)
    (shoulder_des_pos,
     shoulder_des_vel,
     shoulder_des_acc,
     # shoulder_des_jerk,
     # shoulder_des_tau,
     elbow_des_pos,
     elbow_des_vel,
     elbow_des_acc,
     # elbow_des_jerk,
     # elbow_des_tau,
     des_time,
     shoulder_des_pos_out,
     shoulder_des_vel_out,
     elbow_des_pos_out,
     elbow_des_vel_out,
     # shoulder_meas_pos,
     # shoulder_meas_vel,
     # shoulder_meas_tau,
     # shoulder_cmd_tau,
     # elbow_meas_pos,
     # elbow_meas_vel,
     # elbow_meas_tau,
     # elbow_cmd_tau,
     # meas_time,
     n,
     dt) = prepare_data(data, n)

    x_trajectory = construct_nominal_trajectories(des_time,
                                            shoulder_des_pos,
                                            elbow_des_pos,
                                            shoulder_des_vel,
                                            elbow_des_vel)
    acc_trajectory = x_trajectory.derivative(derivative_order=1)

    freq = 250
    states, time_traj = extract_data_from_polynomial(x_trajectory, freq)
    xd_trajectory, time_traj = extract_data_from_polynomial(acc_trajectory, freq)
    shoulder_pos = states[0, :].reshape(time_traj.size, 1).T
    elbow_pos = states[1, :].reshape(time_traj.size, 1).T
    shoulder_vel = states[2, :].reshape(time_traj.size, 1).T
    elbow_vel = states[3, :].reshape(time_traj.size, 1).T
    shoulder_acc = xd_trajectory[2, :].reshape(time_traj.size, 1).T
    elbow_acc = xd_trajectory[3, :].reshape(time_traj.size, 1).T
    # Save Trajectory to a csv file to be sent to the motor.
    csv_data = np.vstack((time_traj,
                          shoulder_pos,
                          shoulder_vel,
                          shoulder_acc,
                          elbow_pos,
                          elbow_vel,
                          elbow_acc)).T
    file_name = 'trajectory-pos-50-3x_250Hz.csv'
    trajectory_path = generate_path(trajectory_folder, file_name, up_directory_counter=2)

    np.savetxt(trajectory_path,
               csv_data, delimiter=',',
               header="time,shoulder_pos,shoulder_vel,shoulder_acc,elbow_pos,elbow_vel,elbow_acc",
               comments="")
    print(f'saved {file_name} in {trajectory_path}')
