import sys


def dynamically_identified_parameters(output_filepath, param_names, p1,
                                                MAE1, MAE2, RMSE1, RMSE2):
    print_path = output_filepath + "/dynamic_parameter_identification.txt"     # save print outs in .txt file

    sys.stdout = open(print_path, "w")
    print('Dynamically Identified Parameters:')
    for i in range(len(param_names)):
        print("{:10s} = {:+.3e}".format(param_names[i], p1[i]))
    print()
    print('Mean Absolute Error:')
    print("MEA1 ||Q_meas - Q_initial|| =", MAE1)
    print("MEA2 ||Q_meas - Q_est|| =", MAE2)
    if MAE1 > MAE2:
        print("Mean absolute error decreases over time: MAE2 < MEA1")
    else:
        print("Mean absolute error doesn't decreases over time: MAE2 >= MEA1")
    print()
    print('Root Mean Square Error:')
    print("RMSE1 ||Q_meas - Q_initial|| =", RMSE1)
    print("RMSE2 ||Q_meas - Q_est|| =", RMSE2)
    if RMSE1 > RMSE2:
        print("Root mean square error decreases over time: RMSE2 < RMSE1")
    else:
        print("Root mean square error doesn't decreases over time: RMSE2 >= RMSE1")
    sys.stdout.close()


def mae_rsme_tracking(output_directory, mae_dict, rmse_dict):
    print_path = output_directory + "/error_calculation.txt"                   # save print outs in .txt file
    sys.stdout = open(print_path, "w")
    print('Mean Absolute Error and Root Mean Square Error \n'
          'between measured and desired Trajectory Data:')
    print()
    print("Position:")
    print("shoulder_pos_mae: ", mae_dict[0])
    print("shoulder_pos_rmse: ",  rmse_dict[0])
    print("elbow_pos_mae: ", mae_dict[1])
    print("elbow_pos_rmse: ", rmse_dict[1])
    print()
    print("Velocity:")
    print("shoulder_vel_mae: ", mae_dict[2])
    print("shoulder_vel_rmse: ", rmse_dict[2])
    print("elbow_vel_mae: ", mae_dict[3])
    print("elbow_vel_rmse: ", rmse_dict[3])
    print()
    print("Acceleration:")
    print("shoulder_acc_mae: ", mae_dict[4])
    print("shoulder_acc_rmse: ", rmse_dict[4])
    print("elbow_acc_mae: ", mae_dict[5])
    print("elbow_acc_rmse: ", rmse_dict[5])
    print()
    print("Torque:")
    print("shoulder_tau_id_mae: ", mae_dict[6])
    print("shoulder_tau_cad_mae: ", mae_dict[7])
    print("shoulder_tau_id_rmse: ", rmse_dict[6])
    print("shoulder_tau_cad_rmse: ", rmse_dict[7])
    print("elbow_tau_id_mae: ", mae_dict[8])
    print("elbow_tau_cad_mae: ", mae_dict[9])
    print("elbow_tau_id_rmse: ", rmse_dict[8])
    print("elbow_tau_cad_rmse: ", rmse_dict[9])
    sys.stdout.close()
