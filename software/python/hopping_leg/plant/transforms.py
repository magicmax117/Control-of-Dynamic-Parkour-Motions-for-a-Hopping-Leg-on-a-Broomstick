import numpy as np
import pandas as pd

from scipy.interpolate import UnivariateSpline as spline

from spine.data.record import record


def TS_to_JS(plant, x, xd, xdd, y, yd, ydd, feedforward_torque, knee_direction):

    s_pos, e_pos = plant.inverse_kinematics(     x,
                                                 y,
                                                 knee_direction=knee_direction)
            
    s_vel, e_vel = plant.inverse_velocity(       s_pos,
                                                 e_pos,
                                                 xd,
                                                 yd)
    
    s_acc, e_acc = plant.inverse_acceleration(   s_pos,
                                                 e_pos,
                                                 s_vel,
                                                 e_vel,
                                                 xdd,
                                                 ydd)
    if feedforward_torque == 'gravity':
        s_tau, e_tau, _ = plant.gravity_vector(  s_pos,
                                                 e_pos)
    elif feedforward_torque == 'full':
        s_tau, e_tau    = plant.inverse_dynamics(s_pos,
                                                 e_pos,
                                                 s_vel,
                                                 e_vel,
                                                 0.0,
                                                 s_acc,
                                                 e_acc)
    elif feedforward_torque == 'zero':
        s_tau = e_tau = 0
    else:
        raise ValueError('the *feedforward_torque* argument to *Controller_TrajectoryFollowing_TaskSpace_Offline* must be either "gravity" or "full"')
    
    return s_pos, s_vel, s_tau, e_pos, e_vel, e_tau


def JS_to_TS(plant, t1, t1d, t1dd, t2, t2d, t2dd):

    x, y       = plant.forward_kinematics(  t1,
                                            t2)

    xd, yd     = plant.forward_velocity(    t1,
                                            t2,
                                            t1d,
                                            t2d)

    xdd, ydd   = plant.forward_acceleration(t1,
                                            t2,
                                            t1d,
                                            t2d,
                                            t1dd,
                                            t2dd)

    return x, xd, xdd, y, yd, ydd


def trajectoryTS_to_JS(plant, trajectory, feedforward_torque, knee_direction):
    """
    Transform a task space trajectory into a joint space trajectory.
    """

    n = len(trajectory)

    # actuator space trajectory
    ast = pd.DataFrame({
        's_pos': np.zeros(n),
        's_vel': np.zeros(n),
        's_tau': np.zeros(n),
        'e_pos': np.zeros(n),
        'e_vel': np.zeros(n),
        'e_tau': np.zeros(n)
    })

    for i in range(n):

        x   = trajectory['pos_x'][i]
        y   = trajectory['pos_y'][i]
        xd  = trajectory['vel_x'][i]
        yd  = trajectory['vel_y'][i]
        xdd = trajectory['acc_x'][i]
        ydd = trajectory['acc_y'][i]
        
        ast.loc[i, :] = TS_to_JS(plant, x, xd, xdd, y, yd, ydd, feedforward_torque, knee_direction)

    return ast


def trajectoryJS_to_TS(plant, trajectory):
    """
    Transform a joint space trajectory into a task space trajectory
    """

    trajectory['t1dd'] = spline(trajectory['t'], trajectory['t1d']).derivative()(trajectory['t'])
    trajectory['t2dd'] = spline(trajectory['t'], trajectory['t2d']).derivative()(trajectory['t'])

    n = len(trajectory)

    # task space trajectory
    tst = pd.DataFrame({
        'x':   np.zeros(n),
        'xd':  np.zeros(n),
        'xdd': np.zeros(n),
        'y':   np.zeros(n),
        'yd':  np.zeros(n),
        'ydd': np.zeros(n)
    })
    
    for i in range(n):

        t    = trajectory['t'][i]
        t1   = trajectory['t1'][i]
        t1d  = trajectory['t1d'][i]
        t1dd = trajectory['t1dd'][i]
        t2   = trajectory['t2'][i]
        t2d  = trajectory['t2d'][i]
        t2dd = trajectory['t2dd'][i]
        
        tst.loc[i, :] = JS_to_TS(plant,
                                 t1,
                                 t1d,
                                 t1dd,
                                 t2,
                                 t2d,
                                 t2dd)

    return tst

    
def recordJS_to_TS(plant, record):

    m1 = record.motor_names[0]
    m2 = record.motor_names[1]

    trajectory_des = pd.DataFrame({
        't':    record['t'],
        't1':   record[f'{m1}_pos_des'],
        't1d':  record[f'{m1}_vel_des'],
        't2':   record[f'{m2}_pos_des'],
        't2d':  record[f'{m2}_vel_des'],
    })
    
    trajectory_msr = pd.DataFrame({
        't':    record['t'],
        't1':   record[f'{m1}_pos_msr'],
        't1d':  record[f'{m1}_vel_msr'],
        't2':   record[f'{m2}_pos_msr'],
        't2d':  record[f'{m2}_vel_msr'],
    })

    tst_des = trajectoryJS_to_TS(plant, trajectory_des)
    tst_des.rename(columns={col: f'{col}_des' for col in tst_des.columns}, inplace=True)

    tst_msr = trajectoryJS_to_TS(plant, trajectory_msr)
    tst_msr.rename(columns={col: f'{col}_msr' for col in tst_msr.columns}, inplace=True)

    return pd.concat([record.to_df(), tst_des, tst_msr], axis=1)
