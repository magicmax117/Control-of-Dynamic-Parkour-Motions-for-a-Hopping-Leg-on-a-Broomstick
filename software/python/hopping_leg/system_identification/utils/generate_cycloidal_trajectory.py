# global imports
import os
import numpy as np


def cycloidal_trajectory(T, dt, theta_init, theta_final):
    """
    Generates cycloidal trajectory based on timestepsize (dt), total time (T),
    start position (theta_init) and end position (theta_final)
    """
    omega = 2*np.pi/T
    ratio = (theta_final - theta_init)/T
    steps = int(T/dt)
    time_vector = np.linspace(0, T, steps)
    pos = np.zeros(steps)
    vel = np.zeros(steps)
    acc = np.zeros(steps)
    i = 0
    for t in time_vector:
        pos[i] = theta_init + ratio * (t - np.sin(omega * t) / omega)
        vel[i] = ratio * (1.0 - np.cos(omega * t))
        acc[i] = ratio * omega * np.sin(omega * t)
        i = i + 1
    return time_vector, pos, vel, acc


def save_to_csv(output_directory, filename, time_vector,
                shoulder_pos, shoulder_vel, shoulder_acc,
                elbow_pos, elbow_vel, elbow_acc):
    """
    Stores cycloidal trajectory for two joints into csv file
    """
    trajectory_csv_data = np.array([np.array(time_vector),
                                    np.array(shoulder_pos),
                                    np.array(shoulder_vel),
                                    np.array(shoulder_acc),
                                    np.array(elbow_pos),
                                    np.array(elbow_vel),
                                    np.array(elbow_acc)]).T
    data_filepath = os.path.join(output_directory, filename)
    np.savetxt(data_filepath, trajectory_csv_data,
           delimiter=',',
           header="time,shoulder_pos,shoulder_vel,shoulder_acc,"
                  "elbow_pos,elbow_vel,elbow_acc",
           comments="")
    print("trajectory saved to {}{} ".format(output_directory, filename))


if __name__ == "__main__":
    output_directory = "../../../data/trajectories/"
    dt = 0.002                                                                 # time interval
    frequency = int(1/dt)
    filename = "cycloidal_trajectory_" + f'{frequency}' + "Hz.csv"
    T = 5.0                                                                    # total time
    theta_init1 = 0.0                                                          # start position in rad shoulder (joint1),
    theta_final1 = 4.71                                                        # end position in rad shoulder
    theta_init2 = 0.0                                                          # start elbow (joint2)
    theta_final2 = 4.71                                                        # end elbow

time_vector, shoulder_pos, shoulder_vel, shoulder_acc \
    = cycloidal_trajectory(T, dt, theta_init1, theta_final1)                   # calculating shoulder trajectory

time_vector2, elbow_pos, elbow_vel, elbow_acc \
    = cycloidal_trajectory(T, dt, theta_init2, theta_final2)                   # calculating elbow trajectory

save_to_csv(output_directory, filename, time_vector,
                shoulder_pos, shoulder_vel, shoulder_acc,
                elbow_pos, elbow_vel, elbow_acc)


