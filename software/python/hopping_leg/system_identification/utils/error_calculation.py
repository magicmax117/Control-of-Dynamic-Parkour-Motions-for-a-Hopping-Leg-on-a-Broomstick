from sklearn.metrics import mean_absolute_error as mae
from sklearn.metrics import mean_squared_error as rmse


def mae_sys_id(Q, phi, p1, xb0):
    Q_meas = Q
    Q_initial = phi.dot(xb0)
    Q_est= phi.dot(p1)
    MAE1 = mae(Q_meas, Q_initial)
    MAE2 = mae(Q_meas, Q_est)
    print()
    if MAE1 > MAE2:
        print("Mean absolute error decreases over time: MAE2 < MEA1")
    else:
        print("Mean absolute error doesn't decreases over time: MAE2 >= MEA1")
    print("MEA1 ||Q_meas - Q_initial|| =", MAE1)
    print("MEA2 ||Q_meas - Q_est|| =", MAE2)
    return MAE1, MAE2


def rmse_sys_id(Q, phi, p1, xb0):
    Q_meas = Q
    Q_initial_est = phi.dot(xb0)
    Q_final_est= phi.dot(p1)
    RMSE1 = rmse(Q_meas, Q_initial_est, squared=False)
    RMSE2 = rmse(Q_meas, Q_final_est, squared=False)
    print()
    if RMSE1 > RMSE2:
        print("Root mean square error decreases over time: RMSE2 < RMSE1")
    else:
        print("Root mean square error doesn't decreases over time: RMSE2 >= RMSE1")
    print("RMSE1 ||Q_meas - Q_initial|| =", RMSE1)
    print("RMSE2 ||Q_meas - Q_est|| =", RMSE2)
    return RMSE1, RMSE2


def mae_rmse_traj_tracking(des_shoulder_pos, meas_shoulder_pos,
                           des_elbow_pos, meas_elbow_pos,
                           des_shoulder_vel, meas_shoulder_vel,
                           des_elbow_vel, meas_elbow_vel,
                           des_shoulder_acc, filt_shoulder_acc,
                           des_elbow_acc, filt_elbow_acc,
                           des_shoulder_trq_id, des_shoulder_trq_cad, meas_shoulder_trq,
                           des_elbow_trq_id, des_elbow_trq_cad, meas_elbow_trq):

    shoulder_pos_mae = mae(des_shoulder_pos[:-2], meas_shoulder_pos)
    elbow_pos_mae = mae(des_elbow_pos[:-2], meas_elbow_pos)
    shoulder_vel_mae = mae(des_shoulder_vel[:-2], meas_shoulder_vel)
    elbow_vel_mae = mae(des_elbow_vel[:-2], meas_elbow_vel)
    shoulder_acc_mae = mae(des_shoulder_acc[:-2], filt_shoulder_acc)
    elbow_acc_mae = mae(des_elbow_acc[:-2], filt_elbow_acc)
    shoulder_tau_id_mae = mae(des_shoulder_trq_id[:-2], meas_shoulder_trq)
    elbow_tau_id_mae = mae(des_elbow_trq_id[:-2], meas_elbow_trq)
    shoulder_tau_cad_mae = mae(des_shoulder_trq_cad[:-2], meas_shoulder_trq)
    elbow_tau_cad_mae = mae(des_elbow_trq_cad[:-2], meas_elbow_trq)

    shoulder_pos_rmse = rmse(des_shoulder_pos[:-2], meas_shoulder_pos)
    elbow_pos_rmse = rmse(des_elbow_pos[:-2], meas_elbow_pos)
    shoulder_vel_rmse = rmse(des_shoulder_vel[:-2], meas_shoulder_vel)
    elbow_vel_rmse = rmse(des_elbow_vel[:-2], meas_elbow_vel)
    shoulder_acc_rmse = rmse(des_shoulder_acc[:-2], filt_shoulder_acc)
    elbow_acc_rmse = rmse(des_elbow_acc[:-2], filt_elbow_acc)
    shoulder_tau_id_rmse = rmse(des_shoulder_trq_id[:-2], meas_shoulder_trq)
    elbow_tau_id_rmse = rmse(des_elbow_trq_id[:-2], meas_elbow_trq)
    shoulder_tau_cad_rmse = rmse(des_shoulder_trq_cad[:-2], meas_shoulder_trq)
    elbow_tau_cad_rmse = rmse(des_elbow_trq_cad[:-2], meas_elbow_trq)

    mae_dict = [shoulder_pos_mae, elbow_pos_mae, shoulder_vel_mae,
                elbow_vel_mae, shoulder_acc_mae, elbow_acc_mae,
                shoulder_tau_id_mae, shoulder_tau_cad_mae, elbow_tau_id_mae,
                elbow_tau_cad_mae]

    rmse_dict = [shoulder_pos_rmse, elbow_pos_rmse, shoulder_vel_rmse,
                 elbow_vel_rmse, shoulder_acc_rmse, elbow_acc_rmse,
                 shoulder_tau_id_rmse, shoulder_tau_cad_rmse, elbow_tau_id_rmse,
                 elbow_tau_cad_rmse]
    return mae_dict, rmse_dict


