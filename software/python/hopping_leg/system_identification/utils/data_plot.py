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

# plot style settings
mpl.rc('font', **{'family': 'serif', 'serif': ['Computer Modern']})                                                     # font style
mpl.rc('text', usetex=True)                                                                                             # use latex
mpl.rcParams['axes.prop_cycle'] = cycler(color=['#20639B', '#3CAEA3', 'orange',
                                                '#ED553B', '#F6D55C',
                                                'dimgrey'])                    # color scheme
av = 1.0                                                                                                                # alpha value
fs = 10                                                                                                                 # font size


def plot_measured_data(output_filepath, t,
                            shoulder_pos,
                            shoulder_vel,
                            shoulder_acc,
                            shoulder_trq,
                            elbow_pos,
                            elbow_vel,
                            elbow_acc,
                            elbow_trq):
    """
    Shows measured joint positions, velocities, accelerations and torques
    """
    fig, axs = plt.subplots(nrows=2, ncols=4, sharey='col', figsize=[18, 6])   # constrained_layout=True
    fig.canvas.set_window_title('Raw Measurement Data')

    # position plots
    axs[0, 0].set_title(r'position [rad]', fontsize=fs)
    axs[0, 0].plot(t, shoulder_pos, alpha=av)
    axs[0, 0].set_xlim([0, t[-1]])
    axs[0, 0].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[0, 0].set_ylabel('shoulder joint', fontsize=fs)
    axs[1, 0].plot(t, elbow_pos, alpha=av)
    axs[1, 0].set_xlim([0, t[-1]])
    axs[1, 0].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[1, 0].set_xlabel('t[s]', fontsize=fs)
    axs[1, 0].set_ylabel('elbow joint', fontsize=fs)
    # velocity plots
    axs[0, 1].set_title('velocity [rad$\cdot$s$^{-1}$]', fontsize=fs)
    axs[0, 1].plot(t, shoulder_vel, color='#3CAEA3', alpha=av)
    axs[0, 1].set_xlim([0, t[-1]])
    axs[0, 1].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[1, 1].plot(t, elbow_vel,  color='#3CAEA3', alpha=av)
    axs[1, 1].set_xlim([0, t[-1]])
    axs[1, 1].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[1, 1].set_xlabel('t[s]', fontsize=fs)
    # acceleration plots
    axs[0, 2].set_title('acceleration [rad$\cdot$s$^{-2}$]', fontsize=fs)
    axs[0, 2].plot(t, shoulder_acc, color='orange', lw=0.5)
    axs[0, 2].set_xlim([0, t[-1]])
    axs[0, 2].axhline(y=0, linestyle='dashed', linewidth=0.8)
    axs[1, 2].plot(t, elbow_acc, color='orange', lw=0.5, alpha=av)
    axs[1, 2].set_xlim([0, t[-1]])
    axs[1, 2].axhline(y=0,  linestyle='dashed', linewidth=0.8)
    axs[1, 2].set_xlabel('t[s]', fontsize=fs)
    # torque plots
    axs[0, 3].set_title('torque [Nm]', fontsize=fs)
    axs[0, 3].plot(t, shoulder_trq, color='#ED553B', alpha=av)
    axs[0, 3].set_xlim([0, t[-1]])
    axs[0, 3].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[1, 3].plot(t, elbow_trq, color='#ED553B', alpha=av)
    axs[1, 3].set_xlim([0, t[-1]])
    axs[1, 3].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[1, 3].set_xlabel('t[s]', fontsize=fs)
    fig.subplots_adjust(top=0.925, bottom=0.11, left=0.045, right=0.96,
                        hspace=0.2, wspace=0.2)
    plt.savefig(output_filepath + "/raw_data_measured.svg", format='svg')
    plt.savefig(output_filepath + "/raw_data_measured.pdf", format='pdf')
    plt.savefig(output_filepath + "/raw_data_measured.png", format='png',
                dpi=300)
    plt.show()
    plt.close(fig)


def plot_smoothed_data(output_filepath, t,
                             shoulder_vel, shoulder_vel2, shoulder_vel3,
                             elbow_vel, elbow_vel2, elbow_vel3,
                             shoulder_trq, shoulder_trq2,
                             elbow_trq, elbow_trq2,
                             shoulder_acc, shoulder_acc3,
                             elbow_acc, elbow_acc3):
    """
    Shows filtered joint positions, velocities, accelerations and torques
    """
    fig, axs = plt.subplots(nrows=3, ncols=2, sharex='col', sharey='row',
                            figsize=[10, 8])
    fig.canvas.set_window_title('Filtered Measurement Data')
    fig.align_ylabels(axs[:, 0])
    axs[0, 0].set_title('shoulder joint', fontsize=fs)
    axs[0, 1].set_title('elbow joint', fontsize=fs)
    # velocity plots
    axs[0, 0].plot(t, shoulder_vel, alpha=av)
    axs[0, 0].plot(t, shoulder_vel3, alpha=av)
    axs[0, 0].plot(t, shoulder_vel2, alpha=av)
    axs[0, 0].set_xlim([0, t[-1]])
    axs[0, 0].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[0, 0].set_ylabel('velocity [rad$\cdot$s$^{-1}$]', fontsize=fs)
    axs[0, 1].plot(t, elbow_vel, alpha=av)
    axs[0, 1].plot(t, elbow_vel3, alpha=av)
    axs[0, 1].plot(t, elbow_vel2, alpha=av)
    axs[0, 1].set_xlim([0, t[-1]])
    axs[0, 1].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[0, 1].legend(["raw data", "gradient pos/dt", "butterworth filter"],
                     loc='lower right')
    # acceleration plots
    axs[1, 0].plot(t, shoulder_acc, lw=0.5, alpha=av)
    axs[1, 0].plot(t, shoulder_acc3, color='orange', lw=0.5, alpha=av)
    axs[1, 0].set_xlim([0, t[-1]])
    axs[1, 0].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[1, 0].set_ylabel('acceleration [rad$\cdot$s$^{-2}$]', fontsize=fs)
    axs[1, 1].plot(t, elbow_acc, lw=0.5, alpha=av)
    axs[1, 1].plot(t, elbow_acc3, color='orange', lw=0.5, alpha=av)
    axs[1, 1].set_xlim([0, t[-1]])
    axs[1, 1].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[1, 1].legend(["raw data", "butterworth filter"], loc='lower right')
    # torque plots
    axs[2, 0].plot(t, shoulder_trq, alpha=av)
    axs[2, 0].plot(t, shoulder_trq2, color='orange', alpha=av)
    axs[2, 0].set_xlim([0, t[-1]])
    axs[2, 0].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[2, 0].set_xlabel('t[s]', fontsize=fs)
    axs[2, 0].set_ylabel('torque [Nm]', fontsize=fs)
    axs[2, 1].plot(t, elbow_trq, alpha=av)
    axs[2, 1].plot(t, elbow_trq2, color='orange', alpha=av)
    axs[2, 1].set_xlim([0, t[-1]])
    axs[2, 1].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[2, 1].set_xlabel('t[s]', fontsize=fs)
    axs[2, 1].legend(["raw data", "butterworth filter"], loc='lower right')
    fig.subplots_adjust(top=0.93, bottom=0.09, left=0.105, right=0.95,
                        hspace=0.15, wspace=0.1)
    plt.savefig(output_filepath + "/raw_data_filtered.svg", format='svg')
    plt.savefig(output_filepath + "/raw_data_filtered.pdf", format='pdf')
    plt.savefig(output_filepath + "/raw_data_filtered.png", format='png',
                dpi=300)
    plt.show()
    plt.close(fig)


def sys_id_and_cad_estimate(output_directory, meas_t,
                            des_shoulder_pos, meas_shoulder_pos,
                            des_elbow_pos, meas_elbow_pos,
                            des_shoulder_vel, meas_shoulder_vel, filt_shoulder_vel,
                            des_elbow_vel, meas_elbow_vel, filt_elbow_vel,
                            des_shoulder_acc, meas_shoulder_acc, filt_shoulder_acc,
                            des_elbow_acc, meas_elbow_acc, filt_elbow_acc,
                            des_shoulder_trq_id, des_shoulder_trq_cad, meas_shoulder_trq, filt_shoulder_trq,
                            des_elbow_trq_id, des_elbow_trq_cad, meas_elbow_trq, filt_elbow_trq):
    """
    Shows comparison between commanded and measured joint positions,
    velocities, accelerations and torques
    """
    fig, axs = plt.subplots(nrows=4, ncols=2, sharex='col', sharey='row',
                            figsize=[10, 10])
    fig.canvas.set_window_title('Desired and Measured Data')
    fig.align_ylabels(axs[:, 0])
    axs[0, 0].set_title('shoulder joint', fontsize=fs)
    axs[0, 1].set_title('elbow joint', fontsize=fs)
    # position plots
    axs[0, 0].plot(meas_t, meas_shoulder_pos, alpha=av)
    axs[0, 0].plot(meas_t, des_shoulder_pos[:-2], color='orange', alpha=av)
    axs[0, 0].set_xlim([0, meas_t[-1]])
    axs[0, 0].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[0, 0].set_ylabel('position [rad]', fontsize=fs)
    axs[0, 1].plot(meas_t, meas_elbow_pos, alpha=av)
    axs[0, 1].plot(meas_t, des_elbow_pos[:-2], color='orange', alpha=av)
    axs[0, 1].set_xlim([0, meas_t[-1]])
    axs[0, 1].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[0, 1].legend(["raw data", "desired"],
                     loc='lower right')
    # velocity plots
    axs[1, 0].plot(meas_t, meas_shoulder_vel, alpha=av)
    axs[1, 0].plot(meas_t, filt_shoulder_vel, alpha=av)
    axs[1, 0].plot(meas_t, des_shoulder_vel[:-2], alpha=av)
    axs[1, 0].set_xlim([0, meas_t[-1]])
    axs[1, 0].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[1, 0].set_ylabel('velocity [rad$\cdot$s$^{-1}$]', fontsize=fs)
    axs[1, 1].plot(meas_t, meas_elbow_vel, alpha=av)
    axs[1, 1].plot(meas_t, filt_elbow_vel, alpha=av)
    axs[1, 1].plot(meas_t, des_elbow_vel[:-2], alpha=av)
    axs[1, 1].set_xlim([0, meas_t[-1]])
    axs[1, 1].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[1, 1].legend(["raw data", "filtered data", "desired"],
                     loc='lower right')
    # acceleration plots
    #axs[2, 0].plot(t, meas_shoulder_acc, alpha=av)
    axs[2, 0].plot(meas_t, filt_shoulder_acc, alpha=av)
    axs[2, 0].plot(meas_t, des_shoulder_acc[:-2], color='orange', alpha=av)
    axs[2, 0].set_xlim([0, meas_t[-1]])
    axs[2, 0].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[2, 0].set_ylabel('acceleration [rad$\cdot$s$^{-2}$]', fontsize=fs)
    #axs[2, 1].plot(t, meas_elbow_acc, alpha=av)
    axs[2, 1].plot(meas_t, filt_elbow_acc, alpha=av)
    axs[2, 1].plot(meas_t, des_elbow_acc[:-2], color='orange', alpha=av)
    axs[2, 1].set_xlim([0, meas_t[-1]])
    axs[2, 1].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[2, 1].legend(["filtered data", "desired"], loc='lower right')
    # torque plots
    axs[3, 0].plot(meas_t, meas_shoulder_trq, alpha=av)
    axs[3, 0].plot(meas_t, filt_shoulder_trq, alpha=av)
    axs[3, 0].plot(meas_t, des_shoulder_trq_cad[:-2], color='orange', alpha=av)
    axs[3, 0].plot(meas_t, des_shoulder_trq_id[:-2], color='#ED553B', alpha=av)
    axs[3, 0].set_xlim([0, meas_t[-1]])
    axs[3, 0].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[3, 0].set_xlabel('t[s]', fontsize=fs)
    axs[3, 0].set_ylabel('torque [Nm]', fontsize=fs)
    axs[3, 1].plot(meas_t, meas_elbow_trq, alpha=av)
    axs[3, 1].plot(meas_t, filt_elbow_trq, alpha=av)
    axs[3, 1].plot(meas_t, des_elbow_trq_cad[:-2], color='orange', alpha=av)
    axs[3, 1].plot(meas_t, des_elbow_trq_id[:-2], color='#ED553B', alpha=av)
    axs[3, 1].set_xlim([0, meas_t[-1]])
    axs[3, 1].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[3, 1].set_xlabel('t[s]', fontsize=fs)
    axs[3, 1].legend(["raw data", "filtered data", "cad estimated",
                      "sys id estimated"], loc='lower right')
    fig.subplots_adjust(top=0.93, bottom=0.09, left=0.105, right=0.95,
                        hspace=0.15, wspace=0.1)
    plt.savefig(output_directory + "/measured_desired_comparison.svg", format='svg')
    plt.savefig(output_directory + "/measured_desired_comparison.pdf", format='pdf')
    plt.savefig(output_directory + "/measured_desired_comparison.png", format='png',
                dpi=300)
    plt.show()
    plt.close(fig)


def plot_least_squares_optimization(output_filepath, t,
                                        est_shoulder_trq,
                                        initial_shoulder_trq,
                                        ref_shoulder_trq,
                                        est_elbow_trq,
                                        initial_elbow_trq,
                                        ref_elbow_trq):
    """
    Shows measured, initial and least-squares estimation of joint torques
    """
    fig, axs = plt.subplots(nrows=1, ncols=2, sharey='row', figsize=[10, 4])
    fig.canvas.set_window_title('Dynamic Parameter Identification')
    axs[0].plot(t, ref_shoulder_trq, alpha=av)
    axs[0].plot(t, initial_shoulder_trq, alpha=av)
    axs[0].plot(t, est_shoulder_trq, alpha=av)
    axs[0].set_xlim([0, t[-1]])
    axs[0].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[0].set_xlabel('t[s]', fontsize=fs)
    axs[0].set_ylabel('torque [Nm]', fontsize=fs)
    axs[0].set_title('shoulder joint', fontsize=fs)
    axs[1].plot(t, ref_elbow_trq, alpha=av)
    axs[1].plot(t, initial_elbow_trq, alpha=av)
    axs[1].plot(t, est_elbow_trq, alpha=av)
    axs[1].set_xlim([0, t[-1]])
    axs[1].axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    axs[1].set_xlabel('t[s]', fontsize=fs)
    axs[1].set_title('elbow joint', fontsize=fs)
    axs[1].legend(["measured data", "initial parameter estimation",
                   "least-squares optimization"], loc='lower right')
    fig.subplots_adjust(top=0.875, bottom=0.160, left=0.085, right=0.945,
                        hspace=0.1, wspace=0.1)
    plt.savefig(output_filepath + "/dynamic_parameter_identification.svg",
                format='svg')
    plt.savefig(output_filepath + "/dynamic_parameter_identification.pdf",
                format='pdf')
    plt.savefig(output_filepath + "/dynamic_parameter_identification.png",
                format='png', dpi=300)
    plt.show()
    plt.close(fig)


def plot_torque_contributions(output_filepath, t,
                                    est_shoulder_trq,
                                    est_shoulder_link_trq,
                                    est_shoulder_fric_trq,
                                    est_shoulder_rotor_trq,
                                    est_elbow_trq,
                                    est_elbow_link_trq,
                                    est_elbow_fric_trq,
                                    est_elbow_rotor_trq):
    """
    Shows contribution of friction, rotor and link inertia to overall
    joint torques
    """
    fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2, sharey='row',
                                   figsize=[10, 4])
    fig.canvas.set_window_title('Friction, Link and Rotor Inertia Contribution')
    ax1.plot(t, est_shoulder_trq, alpha=av)
    ax1.plot(t, est_shoulder_link_trq, alpha=av)
    ax1.plot(t, est_shoulder_rotor_trq, alpha=av)
    ax1.plot(t, est_shoulder_fric_trq, alpha=av)
    ax1.set_xlim([0, t[-1]])
    ax1.axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    ax1.set_xlabel('t[s]', fontsize=fs)
    ax1.set_ylabel('torque [Nm]', fontsize=fs)
    ax1.set_title('shoulder joint', fontsize=fs)
    ax2.plot(t, est_elbow_trq, alpha=av)
    ax2.plot(t, est_elbow_link_trq, alpha=av)
    ax2.plot(t, est_elbow_rotor_trq, alpha=av)
    ax2.plot(t, est_elbow_fric_trq, alpha=av)
    ax2.set_xlim([0, t[-1]])
    ax2.axhline(y=0, color='black',  linestyle='dashed', linewidth=0.8)
    ax2.set_xlabel('t[s]', fontsize=fs)
    ax2.set_title('elbow joint', fontsize=fs)
    ax2.legend(["total", "link inertia", "rotor inertia", "friction"],
                loc='lower right')
    fig.subplots_adjust(top=0.875, bottom=0.160, left=0.085, right=0.945,
                        hspace=0.1, wspace=0.1)
    plt.savefig(output_filepath +
                "/torque_contribution_link_rotor_friction.svg", format='svg')
    plt.savefig(output_filepath +
                "/torque_contribution_link_rotor_friction.pdf", format='pdf')
    plt.savefig(output_filepath +
                "/torque_contribution_link_rotor_friction.png", format='png',
                dpi=300)
    plt.show()
    plt.close(fig)
