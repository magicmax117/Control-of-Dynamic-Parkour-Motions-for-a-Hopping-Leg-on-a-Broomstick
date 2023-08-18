import os
import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
from cycler import cycler

"""
    In order to use latex font styles for your plots you need to install 
    'texlive' and the font 'cm-super'
        $ sudo apt update
        $ sudo apt install texlive-full
        $ sudo apt-get install cm-super
"""
mpl.rc('font', **{'family': 'serif', 'serif': ['Computer Modern']})            # font style
mpl.rc('text', usetex=True)                                                    # use latex
mpl.rcParams['axes.prop_cycle'] = cycler(color=['#20639B', '#3CAEA3', 'orange',
                                                '#ED553B', '#F6D55C',
                                                'dimgrey'])                    # color scheme
av = 1.0                                                                       # alpha value
fs = 10                                                                        # font size


def torque_mae_rmse(output_directory, mae_dict, rmse_dict):
    """
    Shows mean absolute error and root mean square error between estimated
    and measured joint torques
    """
    fig, axs = plt.subplots(nrows=1, ncols=2, sharex='col', sharey='row',
                            figsize=[5, 3])
    fig.canvas.set_window_title('MEA/RMSE measured vs estimated')
    axs[0].set_title('shoulder joint', fontsize=fs)
    axs[1].set_title('elbow joint', fontsize=fs)
    # shoulder torque
    labels = ['MAE', 'RMSE']
    shoulder_cad = [mae_dict[7], rmse_dict[7]]
    shoulder_sys_id = [mae_dict[6], rmse_dict[6]]
    x = np.arange(len(labels))
    width = 0.35
    rects1 = axs[0].bar(x - width/2, shoulder_cad, width, label='cad estimate')
    rects2 = axs[0].bar(x + width/2, shoulder_sys_id, width, label='sys id estimate')
    axs[0].set_ylabel('torque error', fontsize=fs)
    axs[0].set_xticks(x)
    axs[0].set_xticklabels(labels)
    axs[0].legend()
    # elbow torque
    elbow_cad = [mae_dict[9], rmse_dict[9]]
    elbow_sys_id = [mae_dict[8], rmse_dict[8]]
    rects1 = axs[1].bar(x - width/2, elbow_cad, width, label='cad estimate')
    rects2 = axs[1].bar(x + width/2, elbow_sys_id, width, label='sys id estimate')
    axs[1].set_xticks(x)
    axs[1].set_xticklabels(labels)
    fig.subplots_adjust(top=0.93, bottom=0.09, left=0.105, right=0.95,
                        hspace=0.15, wspace=0.1)
    plt.savefig(output_directory + "/mae_rsme_torque_error.svg", format='svg')
    plt.savefig(output_directory + "/mae_rsme_torque_error.pdf", format='pdf')
    plt.savefig(output_directory + "/mae_rsme_torque_error.png", format='png',
                dpi=300)
    plt.show()
    plt.close(fig)


def position_mae(output_directory, mae_kp, mae_kd, mae_shoulder_pos_cad,
                 mae_shoulder_pos_sys_id, mae_elbow_pos_cad,
                 mae_elbow_pos_sys_id):
    """
    Shows mean absolute error between CAD/Sys ID estimated
    and measured joint position
    """
    fig, axs = plt.subplots(nrows=1, ncols=2, sharex='col', sharey='row',
                            figsize=[10, 3])
    fig.canvas.set_window_title('MEA in position sys id vs cad estimate')
    axs[0].set_title('shoulder joint', fontsize=fs)
    axs[1].set_title('elbow joint', fontsize=fs)
    labels = ['kp = {} \n kd = {}'.format(mae_kp[0], mae_kd[0]),
              'kp = {} \n kd = {}'.format(mae_kp[1], mae_kd[1]),
              'kp = {} \n kd = {}'.format(mae_kp[2], mae_kd[2]),
              'kp = {} \n kd = {}'.format(mae_kp[3], mae_kd[3]),
              'kp = {} \n kd = {}'.format(mae_kp[4], mae_kd[4])]
    # shoulder torque
    s_cad = mae_shoulder_pos_cad
    s_sys_id = mae_shoulder_pos_sys_id
    shoulder_cad = [s_cad[0], s_cad[1], s_cad[2], s_cad[3], s_cad[4]]
    shoulder_sys_id = [s_sys_id[0], s_sys_id[1], s_sys_id[2], s_sys_id[3],
                       s_sys_id[4]]
    x = np.arange(len(labels))
    width = 0.35
    rects1 = axs[0].bar(x - width/2, shoulder_cad, width, label='cad estimate')
    rects2 = axs[0].bar(x + width/2, shoulder_sys_id, width, label='sys id estimate')
    axs[0].set_ylabel('MAE position', fontsize=fs)
    axs[0].set_xticks(x)
    axs[0].set_xticklabels(labels,  multialignment='left')
    # elbow torque
    e_cad = mae_elbow_pos_cad
    e_sys_id = mae_elbow_pos_sys_id
    elbow_cad = [e_cad[0], e_cad[1], e_cad[2], e_cad[3], e_cad[4]]
    elbow_sys_id = [e_sys_id[0], e_sys_id[1], e_sys_id[2], e_sys_id[3],
                    e_sys_id[4]]
    rects1 = axs[1].bar(x - width/2, elbow_cad, width, label='cad estimate')
    rects2 = axs[1].bar(x + width/2, elbow_sys_id, width, label='sys id estimate')
    axs[1].set_xticks(x)
    axs[1].set_xticklabels(labels)
    axs[1].legend()
    fig.subplots_adjust(top=0.895, bottom=0.150, left=0.070, right=0.980,
                        hspace=0.15, wspace=0.080)
    plt.savefig(output_directory + "/mae_postion_error.svg", format='svg')
    plt.savefig(output_directory + "/mae_postion_error.pdf", format='pdf')
    plt.savefig(output_directory + "/mae_postion_error.png", format='png',
                dpi=300)
    plt.show()
    plt.close(fig)


def position_rmse(output_directory, rmse_kp, rmse_kd, rmse_shoulder_pos_cad,
                  rmse_shoulder_pos_sys_id, rmse_elbow_pos_cad,
                  rmse_elbow_pos_sys_id):
    """
    Shows root mean square error between CAD/Sys ID estimated
    and measured joint position
    """
    fig, axs = plt.subplots(nrows=1, ncols=2, sharex='col', sharey='row',
                            figsize=[10, 3])
    fig.canvas.set_window_title('RMSE in position sys id vs cad estimate')
    axs[0].set_title('shoulder joint', fontsize=fs)
    axs[1].set_title('elbow joint', fontsize=fs)
    labels = ['kp = {} \n kd = {}'.format(rmse_kp[0], rmse_kd[0]),
              'kp = {} \n kd = {}'.format(rmse_kp[1], rmse_kd[1]),
              'kp = {} \n kd = {}'.format(rmse_kp[2], rmse_kd[2]),
              'kp = {} \n kd = {}'.format(rmse_kp[3], rmse_kd[3]),
              'kp = {} \n kd = {}'.format(rmse_kp[4], rmse_kd[4])]
    # shoulder torque
    s_cad = rmse_shoulder_pos_cad
    s_sys_id = rmse_shoulder_pos_sys_id
    shoulder_cad = [s_cad[0], s_cad[1], s_cad[2], s_cad[3], s_cad[4]]
    shoulder_sys_id = [s_sys_id[0], s_sys_id[1], s_sys_id[2], s_sys_id[3],
                       s_sys_id[4]]
    x = np.arange(len(labels))
    width = 0.35
    rects1 = axs[0].bar(x - width/2, shoulder_cad, width, label='cad estimate')
    rects2 = axs[0].bar(x + width/2, shoulder_sys_id, width, label='sys id estimate')
    axs[0].set_ylabel('RMSE position', fontsize=fs)
    axs[0].set_xticks(x)
    axs[0].set_xticklabels(labels,  multialignment='left')
    # elbow torque
    e_cad = rmse_elbow_pos_cad
    e_sys_id = rmse_elbow_pos_sys_id
    elbow_cad = [e_cad[0], e_cad[1], e_cad[2], e_cad[3], e_cad[4]]
    elbow_sys_id = [e_sys_id[0], e_sys_id[1], e_sys_id[2], e_sys_id[3],
                    e_sys_id[4]]
    rects1 = axs[1].bar(x - width/2, elbow_cad, width, label='cad estimate')
    rects2 = axs[1].bar(x + width/2, elbow_sys_id, width, label='sys id estimate')
    axs[1].set_xticks(x)
    axs[1].set_xticklabels(labels)
    axs[1].legend()
    fig.subplots_adjust(top=0.895, bottom=0.150, left=0.070, right=0.980,
                        hspace=0.15, wspace=0.080)
    plt.savefig(output_directory + "/rmse_postion_error.svg", format='svg')
    plt.savefig(output_directory + "/rmse_postion_error.pdf", format='pdf')
    plt.savefig(output_directory + "/rmse_postion_error.png", format='png',
                dpi=300)
    plt.show()
    plt.close(fig)


def veloctiy_mae(output_directory, mae_kp, mae_kd, mae_shoulder_vel_cad,
    mae_shoulder_vel_sys_id, mae_elbow_vel_cad, mae_elbow_vel_sys_id):
    """
    Shows mean absolute error between CAD/Sys ID estimated
    and measured joint velocity
    """
    fig, axs = plt.subplots(nrows=1, ncols=2, sharex='col', sharey='row',
                            figsize=[10, 3])
    fig.canvas.set_window_title('MEA in velocity sys id vs cad estimate')
    axs[0].set_title('shoulder joint', fontsize=fs)
    axs[1].set_title('elbow joint', fontsize=fs)
    labels = ['kp = {} \n kd = {}'.format(mae_kp[0], mae_kd[0]),
              'kp = {} \n kd = {}'.format(mae_kp[1], mae_kd[1]),
              'kp = {} \n kd = {}'.format(mae_kp[2], mae_kd[2]),
              'kp = {} \n kd = {}'.format(mae_kp[3], mae_kd[3]),
              'kp = {} \n kd = {}'.format(mae_kp[4], mae_kd[4])]
    # shoulder torque
    s_cad = mae_shoulder_vel_cad
    s_sys_id = mae_shoulder_vel_sys_id
    shoulder_cad = [s_cad[0], s_cad[1], s_cad[2], s_cad[3], s_cad[4]]
    shoulder_sys_id = [s_sys_id[0], s_sys_id[1], s_sys_id[2], s_sys_id[3],
                       s_sys_id[4]]
    x = np.arange(len(labels))
    width = 0.35
    rects1 = axs[0].bar(x - width/2, shoulder_cad, width, label='cad estimate')
    rects2 = axs[0].bar(x + width/2, shoulder_sys_id, width, label='sys id estimate')
    axs[0].set_ylabel('MAE velocity', fontsize=fs)
    axs[0].set_xticks(x)
    axs[0].set_xticklabels(labels,  multialignment='left')
    # elbow torque
    e_cad = mae_elbow_vel_cad
    e_sys_id = mae_elbow_vel_sys_id
    elbow_cad = [e_cad[0], e_cad[1], e_cad[2], e_cad[3], e_cad[4]]
    elbow_sys_id = [e_sys_id[0], e_sys_id[1], e_sys_id[2], e_sys_id[3],
                    e_sys_id[4]]
    rects1 = axs[1].bar(x - width/2, elbow_cad, width, label='cad estimate')
    rects2 = axs[1].bar(x + width/2, elbow_sys_id, width, label='sys id estimate')
    axs[1].set_xticks(x)
    axs[1].set_xticklabels(labels)
    axs[1].legend()
    fig.subplots_adjust(top=0.895, bottom=0.150, left=0.070, right=0.980,
                        hspace=0.15, wspace=0.080)
    plt.savefig(output_directory + "/mae_velocity_error.svg", format='svg')
    plt.savefig(output_directory + "/mae_velocity_error.pdf", format='pdf')
    plt.savefig(output_directory + "/mae_velocity_error.png", format='png',
                dpi=300)
    plt.show()
    plt.close(fig)


def veloctiy_rmse(output_directory, rmse_kp, rmse_kd, rmse_shoulder_vel_cad,
    rmse_shoulder_vel_sys_id, rmse_elbow_vel_cad, rmse_elbow_vel_sys_id):
    """
    Shows root mean square error between CAD/Sys ID estimated
    and measured joint velocity
    """
    fig, axs = plt.subplots(nrows=1, ncols=2, sharex='col', sharey='row',
                            figsize=[10, 3])
    fig.canvas.set_window_title('RMSE in velocity sys id vs cad estimate')
    axs[0].set_title('shoulder joint', fontsize=fs)
    axs[1].set_title('elbow joint', fontsize=fs)
    labels = ['kp = {} \n kd = {}'.format(rmse_kp[0], rmse_kd[0]),
              'kp = {} \n kd = {}'.format(rmse_kp[1], rmse_kd[1]),
              'kp = {} \n kd = {}'.format(rmse_kp[2], rmse_kd[2]),
              'kp = {} \n kd = {}'.format(rmse_kp[3], rmse_kd[3]),
              'kp = {} \n kd = {}'.format(rmse_kp[4], rmse_kd[4])]
    # shoulder torque
    s_cad = rmse_shoulder_vel_cad
    s_sys_id = rmse_shoulder_vel_sys_id
    shoulder_cad = [s_cad[0], s_cad[1], s_cad[2], s_cad[3], s_cad[4]]
    shoulder_sys_id = [s_sys_id[0], s_sys_id[1], s_sys_id[2], s_sys_id[3],
                       s_sys_id[4]]
    x = np.arange(len(labels))
    width = 0.35
    rects1 = axs[0].bar(x - width/2, shoulder_cad, width, label='cad estimate')
    rects2 = axs[0].bar(x + width/2, shoulder_sys_id, width, label='sys id estimate')
    axs[0].set_ylabel('RMSE velocity', fontsize=fs)
    axs[0].set_xticks(x)
    axs[0].set_xticklabels(labels,  multialignment='left')
    # elbow torque
    e_cad = rmse_elbow_vel_cad
    e_sys_id = rmse_elbow_vel_sys_id
    elbow_cad = [e_cad[0], e_cad[1], e_cad[2], e_cad[3], e_cad[4]]
    elbow_sys_id = [e_sys_id[0], e_sys_id[1], e_sys_id[2], e_sys_id[3],
                    e_sys_id[4]]
    rects1 = axs[1].bar(x - width/2, elbow_cad, width, label='cad estimate')
    rects2 = axs[1].bar(x + width/2, elbow_sys_id, width, label='sys id estimate')
    axs[1].set_xticks(x)
    axs[1].set_xticklabels(labels)
    axs[1].legend()
    fig.subplots_adjust(top=0.895, bottom=0.150, left=0.070, right=0.980,
                        hspace=0.15, wspace=0.080)
    plt.savefig(output_directory + "/rmse_velocity_error.svg", format='svg')
    plt.savefig(output_directory + "/rmse_velocity_error.pdf", format='pdf')
    plt.savefig(output_directory + "/rmse_velocity_error.png", format='png',
                dpi=300)
    plt.show()
    plt.close(fig)


def import_mae_data(input_directory, mae_file):
    filepath = input_directory + mae_file
    data = pd.read_csv(filepath)
    mae_kp = data["kp"].tolist()
    mae_kd = data["kd"].tolist()
    mae_shoulder_pos_cad = data["mae_shoulder_pos_cad"].tolist()
    mae_shoulder_pos_sys_id = data["mae_shoulder_pos_sys_id"].tolist()
    mae_elbow_pos_cad = data["mae_elbow_pos_cad"].tolist()
    mae_elbow_pos_sys_id = data["mae_elbow_pos_sys_id"].tolist()
    mae_shoulder_vel_cad = data["mae_shoulder_vel_cad"].tolist()
    mae_shoulder_vel_sys_id = data["mae_shoulder_vel_sys_id"].tolist()
    mae_elbow_vel_cad = data["mae_elbow_vel_cad"].tolist()
    mae_elbow_vel_sys_id = data["mae_elbow_vel_sys_id"].tolist()
    return mae_kp, mae_kd, mae_shoulder_pos_cad, mae_shoulder_pos_sys_id, \
           mae_elbow_pos_cad, mae_elbow_pos_sys_id, mae_shoulder_vel_cad, \
           mae_shoulder_vel_sys_id, mae_elbow_vel_cad, mae_elbow_vel_sys_id


def import_rmse_data(input_directory, rmse_file):
    filepath = input_directory + rmse_file
    data = pd.read_csv(filepath)
    rmse_kp = data["kp"].tolist()
    rmse_kd = data["kd"].tolist()
    rmse_shoulder_pos_cad = data["rmse_shoulder_pos_cad"].tolist()
    rmse_shoulder_pos_sys_id = data["rmse_shoulder_pos_sys_id"].tolist()
    rmse_elbow_pos_cad = data["rmse_elbow_pos_cad"].tolist()
    rmse_elbow_pos_sys_id = data["rmse_elbow_pos_sys_id"].tolist()
    rmse_shoulder_vel_cad = data["rmse_shoulder_vel_cad"].tolist()
    rmse_shoulder_vel_sys_id = data["rmse_shoulder_vel_sys_id"].tolist()
    rmse_elbow_vel_cad = data["rmse_elbow_vel_cad"].tolist()
    rmse_elbow_vel_sys_id = data["rmse_elbow_vel_sys_id"].tolist()
    return rmse_kp, rmse_kd, rmse_shoulder_pos_cad, rmse_shoulder_pos_sys_id, \
           rmse_elbow_pos_cad, rmse_elbow_pos_sys_id, rmse_shoulder_vel_cad, \
           rmse_shoulder_vel_sys_id, rmse_elbow_vel_cad, rmse_elbow_vel_sys_id


if __name__ == "__main__":
    experiment_folder = "20211230_param_id_excitation_traj_202406/"
    trajectory_folder = "swingup_500Hz/"
    input_directory = "../../../results/" + experiment_folder + \
                       "test_data/" + trajectory_folder
    mae_file = "swingup_500Hz_mae_pos_vel.csv"
    rmse_file = "swingup_500Hz_rmse_pos_vel.csv"
    output_directory = input_directory + "plots/"
    os.makedirs(output_directory)

    mae_kp, mae_kd, mae_shoulder_pos_cad, mae_shoulder_pos_sys_id, \
    mae_elbow_pos_cad, mae_elbow_pos_sys_id, mae_shoulder_vel_cad, \
    mae_shoulder_vel_sys_id, mae_elbow_vel_cad, mae_elbow_vel_sys_id = \
    import_mae_data(input_directory, mae_file)                                # import mae data

    rmse_kp, rmse_kd, rmse_shoulder_pos_cad, rmse_shoulder_pos_sys_id, \
    rmse_elbow_pos_cad, rmse_elbow_pos_sys_id, rmse_shoulder_vel_cad, \
    rmse_shoulder_vel_sys_id, rmse_elbow_vel_cad, rmse_elbow_vel_sys_id = \
    import_rmse_data(input_directory, rmse_file)                              # import rmse data

    position_mae(output_directory, mae_kp, mae_kd, mae_shoulder_pos_cad,
                 mae_shoulder_pos_sys_id, mae_elbow_pos_cad,
                 mae_elbow_pos_sys_id)                                         # make mae position bar charts

    position_rmse(output_directory, rmse_kp, rmse_kd, rmse_shoulder_pos_cad,
                  rmse_shoulder_pos_sys_id, rmse_elbow_pos_cad,
                  rmse_elbow_pos_sys_id)                                       # make rsme position bar charts

    veloctiy_mae(output_directory, mae_kp, mae_kd, mae_shoulder_vel_cad,
                 mae_shoulder_vel_sys_id, mae_elbow_vel_cad,
                 mae_elbow_vel_sys_id)                                         # make mae velocity bar charts

    veloctiy_rmse(output_directory, rmse_kp, rmse_kd, rmse_shoulder_vel_cad,
                  rmse_shoulder_vel_sys_id, rmse_elbow_vel_cad,
                  rmse_elbow_vel_sys_id)                                       # make rsme velocity bar charts
