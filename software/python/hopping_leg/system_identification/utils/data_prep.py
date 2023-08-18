# global imports
import numpy as np
import pandas as pd
from scipy import signal
# from scipy.fft import rfft, irfft, rfftfreq
from scipy.fftpack import rfft, irfft, rfftfreq


def import_measured_data(measured_data_filepath):
    data = pd.read_csv(measured_data_filepath)
    print("Measured Data Shape = ", data.shape)
    meas_t = data["time"].tolist()
    meas_shoulder_pos = data["shoulder_pos"].tolist()
    meas_shoulder_vel = data["shoulder_vel"].tolist()
    meas_shoulder_acc = data["shoulder_acc"].tolist()
    meas_shoulder_trq = data["shoulder_torque"].tolist()
    meas_elbow_pos = data["elbow_pos"].tolist()
    meas_elbow_vel = data["elbow_vel"].tolist()
    meas_elbow_acc = data["elbow_acc"].tolist()
    meas_elbow_trq = data["elbow_torque"].tolist()
    return meas_t, meas_shoulder_pos, meas_shoulder_vel, meas_shoulder_acc, \
           meas_shoulder_trq, meas_elbow_pos, meas_elbow_vel, meas_elbow_acc, \
           meas_elbow_trq


def import_desired_data(desired_data_filepath):
    data = pd.read_csv(desired_data_filepath)
    print("Desired Data Shape = ", data.shape)
    des_t = data["time"].tolist()
    des_shoulder_pos = data["shoulder_pos"].tolist()
    des_shoulder_vel = data["shoulder_vel"].tolist()
    des_shoulder_acc = data["shoulder_acc"].tolist()
    des_shoulder_trq_id = data["shoulder_tau_id"].tolist()
    des_shoulder_trq_cad = data["shoulder_tau_cad"].tolist()
    des_elbow_pos = data["elbow_pos"].tolist()
    des_elbow_vel = data["elbow_vel"].tolist()
    des_elbow_acc = data["elbow_acc"].tolist()
    des_elbow_trq_id = data["elbow_tau_id"].tolist()
    des_elbow_trq_cad = data["shoulder_tau_cad"].tolist()
    return des_t, des_shoulder_pos, des_shoulder_vel, des_shoulder_acc, \
           des_shoulder_trq_id, des_shoulder_trq_cad, des_elbow_pos, \
           des_elbow_vel, des_elbow_acc, des_elbow_trq_id, des_elbow_trq_cad

"""

def import_data(data_filepath):
    data = np.genfromtxt(data_filepath, delimiter=",", skip_header=1)          # load data from csv file
    print(data.shape)
    t = data.T[0]
    shoulder_pos = data.T[1]                                                   # *360.0/2*np.pi for converting rad to deg
    shoulder_vel = data.T[2]                                                   # 360.0/2*np.pi
    shoulder_acc_num_diff = data.T[3]
    shoulder_trq = data.T[4]

    elbow_pos = data.T[5]
    elbow_vel = data.T[6]
    elbow_acc_num_diff = data.T[7]
    elbow_trq = data.T[8]
    return t, shoulder_pos, shoulder_vel, shoulder_acc_num_diff, shoulder_trq, \
           elbow_pos, elbow_vel, elbow_acc_num_diff, elbow_trq
"""

def fft_filter(x, y, smooth_freq=100):
    N = len(x)
    w = rfft(y)                                                                # Fast Fourier Transforms (FFT) of a strictly real sequence from the scipy package, where y is the array with measured data values
    f = rfftfreq(N, x[1]-x[0])                                                 # returns the Discrete Fourier Transform (DFT) sample frequencies
    spectrum = w**2                                                            # squared amplitudes w of the FFT
    cutoff_idx = spectrum < (spectrum.max() / smooth_freq)                     # cuts off frequencies that are much lower (/smooth_freq) then the main frequency at spectrum.max.()
    w2 = w.copy()
    w2[cutoff_idx] = 0
    y2 = irfft(w2)                                                             # inverses the filtered FFT values (where frequencies that are too lo are cut off) back to our original data type
    return y2


def smooth_data(t, shoulder_pos, shoulder_vel, shoulder_trq, elbow_pos,
                elbow_vel, elbow_trq, ret='short'):
    """
    velocity data filter
    """
    shoulder_vel1 = fft_filter(t, shoulder_vel, 200)                           # vel smooth_data
    b_shoulder_vel, a_shoulder_vel = signal.butter(3, 0.2)
    shoulder_vel2 = signal.filtfilt(b_shoulder_vel, a_shoulder_vel,            # vel forward/backward filter with butter
                                    shoulder_vel)
    shoulder_vel3 = np.gradient(shoulder_pos, t)
    b_shoulder_vel, a_shoulder_vel = signal.butter(3, 0.5)
    shoulder_vel3 = signal.filtfilt(b_shoulder_vel, a_shoulder_vel,            # pos/dt forward/backward filter with butter
                                    shoulder_vel3)
    elbow_vel1 = fft_filter(t, elbow_vel, 200)
    b_elbow_vel, a_elbow_vel = signal.butter(3, 0.2)
    elbow_vel2 = signal.filtfilt(b_elbow_vel, a_elbow_vel, elbow_vel)
    elbow_vel3 = np.gradient(elbow_pos, t)
    b_elbow_vel, a_elbow_vel = signal.butter(3, 0.5)
    elbow_vel3 = signal.filtfilt(b_elbow_vel, a_elbow_vel, elbow_vel3)

    """
    torque data filter
    """
    b_shoulder_trq, a_shoulder_trq = signal.butter(3, 0.1)
    shoulder_trq1 = fft_filter(t, shoulder_trq, 50)
    shoulder_trq2 = signal.filtfilt(b_shoulder_trq, a_shoulder_trq, shoulder_trq)
    b_elbow_trq, a_elbow_trq = signal.butter(3, 0.1)
    elbow_trq1 = fft_filter(t, elbow_trq, 50)
    elbow_trq2 = signal.filtfilt(b_elbow_trq, a_elbow_trq, elbow_trq)

    """
    acceleration data filter
    """
    shoulder_acc = np.gradient(shoulder_vel, t)
    shoulder_acc2 = np.gradient(shoulder_vel2, t)
    shoulder_acc3 = np.gradient(shoulder_vel3, t)
    b_shoulder_acc, a_shoulder_acc = signal.butter(3, 0.1)
    shoulder_acc3 = signal.filtfilt(b_shoulder_acc, a_shoulder_acc,
                                    shoulder_acc3)
    elbow_acc = np.gradient(elbow_vel, t)
    elbow_acc2 = np.gradient(elbow_vel2, t)
    elbow_acc3 = np.gradient(elbow_vel3, t)
    b_elbow_acc, a_elbow_acc = signal.butter(3, 0.1)
    elbow_acc3 = signal.filtfilt(b_elbow_acc, a_elbow_acc, elbow_acc3)

    """
    filtered data selected for sys id
    """
    ref_t = t
    ref_shoulder_pos = shoulder_pos
    ref_shoulder_vel = shoulder_vel2
    # ref_shoulder_vel = shoulder_vel
    ref_shoulder_acc = shoulder_acc3
    ref_shoulder_trq = shoulder_trq2
    # ref_shoulder_trq = shoulder_trq
    ref_elbow_pos = elbow_pos
    ref_elbow_vel = elbow_vel2
    # ref_elbow_vel = elbow_vel
    ref_elbow_acc = elbow_acc3
    ref_elbow_trq = elbow_trq2
    # ref_elbow_trq = elbow_trq
    if ret == 'short':
        return ref_t, ref_shoulder_pos, ref_elbow_pos, \
               ref_shoulder_vel, shoulder_vel2, shoulder_vel3, \
               ref_elbow_vel, elbow_vel2, elbow_vel3,\
               ref_shoulder_acc, shoulder_acc,  shoulder_acc2, shoulder_acc3, \
               ref_elbow_acc, elbow_acc, elbow_acc2, elbow_acc3, \
               ref_shoulder_trq, shoulder_trq2, \
               ref_elbow_trq, elbow_trq2

    elif ret == 'all':
        return (ref_t,
                ref_shoulder_pos,
                ref_shoulder_vel,
                ref_shoulder_acc,
                ref_shoulder_trq,
                ref_elbow_pos,
                ref_elbow_vel,
                ref_elbow_acc,
                ref_elbow_trq,
                shoulder_vel1,
                shoulder_vel2,
                elbow_vel1,
                elbow_vel2,
                shoulder_trq1,
                elbow_trq1,
                elbow_trq2,
                shoulder_acc,
                shoulder_acc2,
                elbow_acc,
                elbow_acc2,
                shoulder_vel3,
                elbow_vel3,
                shoulder_trq2,
                shoulder_acc3,
                elbow_acc3)


"""                                                                            # running mean filter
def running_mean(x, N):
    cumsum = np.cumsum(np.insert(x, 0, 0))
    return (cumsum[N:] - cumsum[:-N]) / float(N)


filtered_shoulder_acc = running_mean(shoulder_acceleration, 10)
filtered_elbow_acc = running_mean(elbow_acceleration, 10)
filtered_shoulder_acc_traj = running_mean(np.array(shoulder_acc_traj), 10)
filtered_elbow_acc_traj = running_mean(np.array(elbow_acc_traj), 10)

time_vec_filtered = running_mean(np.array(time_vec), 10)
des_time_vec_filtered = running_mean(np.array(des_time_vec), 10)

plt.plot(time_vec_filtered[:-2], filtered_shoulder_acc)
plt.plot(time_vec_filtered[:-2], filtered_elbow_acc)
plt.plot(des_time_vec_filtered, filtered_shoulder_acc_traj)
plt.plot(des_time_vec_filtered, filtered_elbow_acc_traj)

plt.xlabel("Time (s)")
plt.ylabel("Acceleration")
plt.title("Filtered Acceleration vs Time (s) with moving average "
          "filter (window = 100)")
plt.legend(['Shoulder Measured', 'Elbow Measured', 'Shoulder Desired',
            'Elbow Desired'])
plt.show()
"""

