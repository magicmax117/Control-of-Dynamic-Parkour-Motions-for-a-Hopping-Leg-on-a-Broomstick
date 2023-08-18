# global imports
import os
import numpy as np
from datetime import datetime
from scipy.io import savemat
# local imports
from utils import data_prep as prep_data
from utils import data_plot as plot_data
from utils import params_load as prep_params
from utils import error_calculation
from utils import print_to_textfile
from state_vector import xb_vector_ref_rotor
from regressor_matrix import yb_matrix, yb_friction_matrix, \
    yb_rotor_matrix, yb_link_matrix, yb_SE3_matrix
from least_squares_optimization import solve_least_squares, \
    errfunc_with_friction

timestamp = datetime.now().strftime("%Y%m%d-%I%M%S-%p")                              # get a timestamp
#params_filepath = "../../data/parameters/params_large_second_link_cad.yaml"         # setting file paths
params_filepath = "./data/hopping_leg_v2_mjbots/params_cad.yaml"         # setting file paths
directory = "./data/hopping_leg_v2_mjbots/experiments/DC-R1/"
trajectory = "measured.csv"
trajectory_filepath = os.path.join(directory, trajectory)
output_filepath = os.path.join(directory) + \
                  "/" + f'{timestamp}' + "_results_sys_id"
os.makedirs(output_filepath)


def build_identification_matrices(t, g, n1, n2, L1, xb0, yb_matrix):
    num_samples = len(t)
    num_params = len(xb0)
    num_friction_params = 4                                                    # hard coded for a 2 joint robot
    num_rotor_params = 1                                                       # hard coded for a 2 joint robot
    num_link_params = 5                                                        # hard coded for a 2 joint robot
    phi = np.empty((num_samples*2, num_params))
    phi_link = np.empty((num_samples*2, num_link_params))
    phi_fric = np.empty((num_samples*2, num_friction_params))
    phi_rotor = np.empty((num_samples*2, num_rotor_params))
    phi_SE3 = np.empty((num_samples*2, 20))
    Q = np.empty((num_samples*2, 1))
    b = 0
    r = np.linalg.matrix_rank(phi, tol=0.1)
    print("rank:", r)
    for i in range(len(t)):
        Q[b:b+2, 0] = np.array([ref_shoulder_trq[i], ref_elbow_trq[i]])        # Q contains the measured torques of both joints for every time step
        q_vec = np.array([ref_shoulder_pos[i], ref_elbow_pos[i]])
        dq_vec = np.array([ref_shoulder_vel[i], ref_elbow_vel[i]])
        ddq_vec = np.array([ref_shoulder_acc[i], ref_elbow_acc[i]])
        phi[b:b+2, :] = yb_matrix(g, n1, n2, L1, q_vec, dq_vec, ddq_vec)               # phi contains the regressor matrix (yb) for every time step
        phi_link[b:b+2, :] = yb_link_matrix(g, L1, q_vec, dq_vec, ddq_vec)        
        phi_fric[b:b+2, :] = yb_friction_matrix(dq_vec)
        phi_rotor[b:b+2, :] = yb_rotor_matrix(ddq_vec, n1, n2)
        phi_SE3[b:b+2, :] = yb_SE3_matrix(g, L1, q_vec, dq_vec, ddq_vec)
        b += 2
    return Q, phi, phi_link, phi_fric, phi_rotor, phi_SE3


def get_joint_torques_from_state_vector(phi, phi_link, phi_fric, phi_rotor,
                                        p1, xb0):

    initial_trq = phi.dot(xb0)
    initial_shoulder_trq = initial_trq[::2]
    initial_elbow_trq = initial_trq[1::2]                                      # joint torques for initial parameter guess
    est_trq = phi.dot(p1)
    est_shoulder_trq = est_trq[::2]
    est_elbow_trq = est_trq[1::2]
    est_link_param_vector = np.concatenate([p1[:2], p1[5:8]]).flatten()
    est_link_trq = phi_link.dot(est_link_param_vector)
    est_shoulder_link_trq = est_link_trq[::2]
    est_elbow_link_trq = est_link_trq[1::2]                                    # joint torques from link inertia
    est_fric_param_vector = np.array([p1[2:4], p1[8:10]]).flatten()
    est_fric_trq = phi_fric.dot(est_fric_param_vector)
    est_shoulder_fric_trq = est_fric_trq[::2]
    est_elbow_fric_trq = est_fric_trq[1::2]                                    # joint torques from friction terms
    '''
    # assuming the rotor inertia params are known
    n1 = 6
    n2 = 6
    Ir1 = 0.000060719
    Ir2 = 0.000060719
    rotor_param_vec = np.array([n1**2*Ir1, n2**2*Ir2])
    '''
    rotor_param_vec = np.array([p1[4]])
    est_rotor_trq = phi_rotor.dot(rotor_param_vec)
    est_shoulder_rotor_trq = est_rotor_trq[::2]
    est_elbow_rotor_trq = est_rotor_trq[1::2]                                  # joint torques from rotor inertia
    
    return est_shoulder_trq, est_elbow_trq, \
           initial_shoulder_trq, initial_elbow_trq, \
           est_link_trq, est_shoulder_link_trq, est_elbow_link_trq, \
           est_shoulder_fric_trq, est_elbow_fric_trq, \
           est_shoulder_rotor_trq, est_elbow_rotor_trq


# def inverse_dynamics(phi, xb):
#    return phi.dot(xb)

'''
Run the script
'''
g, m1, m2, I1, I2, L1, L2, Lc1, Lc2, Fc1, Fv1, Fc2, Fv2, Ir, n1, n2, Irr1, Irr2 = \
    prep_params.load_from_yaml(params_filepath)                                # load params from yaml file

t, shoulder_pos, shoulder_vel, shoulder_acc_num_diff, shoulder_trq, \
    elbow_pos, elbow_vel, elbow_acc_num_diff, elbow_trq \
    = prep_data.import_measured_data(trajectory_filepath)                            # load data from csv file

ref_t, ref_shoulder_pos, ref_shoulder_vel, ref_shoulder_acc, \
    ref_shoulder_trq, ref_elbow_pos, ref_elbow_vel, ref_elbow_acc, \
    ref_elbow_trq, shoulder_vel1, shoulder_vel2, elbow_vel1, elbow_vel2, \
    shoulder_trq1, elbow_trq1, elbow_trq2, shoulder_acc, shoulder_acc2, \
    elbow_acc, elbow_acc2, shoulder_vel3, elbow_vel3, \
    shoulder_trq2, shoulder_acc3, elbow_acc3 \
= prep_data.smooth_data(t, shoulder_pos, shoulder_vel, shoulder_trq,
                            elbow_pos, elbow_vel, elbow_trq, ret='all')                   # smooth noisy measurements

plot_data.plot_measured_data(output_filepath, t,
                                 shoulder_pos,
                                 shoulder_vel,
                                 shoulder_acc,
                                 shoulder_trq,
                                 elbow_pos,
                                 elbow_vel,
                                 elbow_acc,
                                 elbow_trq)                                    # plot measurements

plot_data.plot_smoothed_data(output_filepath, t,
                                 shoulder_vel, shoulder_vel2, shoulder_vel3,
                                 elbow_vel, elbow_vel2, elbow_vel3,
                                 shoulder_trq, shoulder_trq2,
                                 elbow_trq, elbow_trq2,
                                 shoulder_acc, shoulder_acc3,
                                 elbow_acc, elbow_acc3)                        # plot filtered measurements

xb0, bounds = xb_vector_ref_rotor(m1, m2, I1, I2, Lc1, Lc2, Fc1, Fc2,
                         Fv1, Fv2, Ir)                               # state vector with bounds

Q, phi, phi_link, phi_fric, phi_rotor, phi_SE3\
    = build_identification_matrices(t, g, n1, n2, L1, xb0, yb_matrix)

p1 = solve_least_squares(Q, phi, xb0, bounds, errfunc_with_friction)           # solve least-squares optimization problem

est_shoulder_trq, est_elbow_trq, initial_shoulder_trq, initial_elbow_trq, \
    est_link_trq, est_shoulder_link_trq, est_elbow_link_trq, \
    est_shoulder_fric_trq, est_elbow_fric_trq, \
    est_shoulder_rotor_trq, est_elbow_rotor_trq = \
    get_joint_torques_from_state_vector(phi,
                                        phi_link,
                                        phi_fric,
                                        phi_rotor,
                                        p1,
                                        xb0)                                   # calculate torques from state vector

plot_data.plot_least_squares_optimization(output_filepath, t,
                                              est_shoulder_trq,
                                              initial_shoulder_trq,
                                              ref_shoulder_trq,
                                              est_elbow_trq,
                                              initial_elbow_trq,
                                              ref_elbow_trq)                   # plot measured vs. estimated torques from least-squares optimization

plot_data.plot_torque_contributions(output_filepath, t,
                                        est_shoulder_trq,
                                        est_shoulder_link_trq,
                                        est_shoulder_fric_trq,
                                        est_shoulder_rotor_trq,
                                        est_elbow_trq,
                                        est_elbow_link_trq,
                                        est_elbow_fric_trq,
                                        est_elbow_rotor_trq)                   # plot torque contributions of link inertia, rotor inertia and friction

y = est_link_trq[np.newaxis].T
Phi_prior = np.array([[m1, m1*Lc1, 0, 0, 0, 0, I1, 0, 0, 0],
                      [m2, m2*Lc2, 0, 0, 0, 0, I2, 0, 0, 0]])
mdic = {"A": phi_SE3, "b": Q, "b_link": y, "Phi_prior": Phi_prior.T}           # where
                                                                                        # phi_SE3 = concatenation of yb_SE3 matrix (only link inertia) for all time steps
                                                                                        # Q = measured joint torques
                                                                                        # y = least-squares estimation of joint torqes (only link inertia)
                                                                                        # Phi_prior = initial yb matrix (only link inertia)

savemat(output_filepath + "/Train_data_acrobot.mat", mdic)                     # save matrices to .mat files
print("p1 =", p1)
print("xb0 =", xb0)

MAE1, MAE2 = error_calculation.mae_sys_id(Q, phi, p1, xb0)                     # return mean absolute error
RMSE1, RMSE2 = error_calculation.rmse_sys_id(Q, phi, p1, xb0)                  # return root mean square error

param_names = ["Lc1*m1", "I1", "Fc1", "Fv1", "Ir", "Lc2*m2", "m2", "I2", "Fc2", "Fv2",]
print()
print('Dynamically Identified Parameters:')
for i in range(len(param_names)):
    print("{:10s} = {:+.3e}".format(param_names[i], p1[i]))                    # print identified parameters

print_to_textfile.dynamically_identified_parameters(output_filepath,
                                                        param_names, p1,
                                                        MAE1, MAE2,
                                                        RMSE1, RMSE2)








