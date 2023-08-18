import sys
import time
import moteus
import asyncio
import os
import matplotlib.pyplot as plt
import numpy as np
from state_vector import xb_vector_id_mjbots
from regressor_matrix import yb_matrix


def running_mean(x, N):
    cumsum = np.cumsum(np.insert(x, 0, 0))
    return (cumsum[N:] - cumsum[:-N]) / float(N)

def rev2rad(angle_in_revolution):
    return angle_in_revolution * (2 * np.pi)

def rad2rev(angle_in_radians):
    return angle_in_radians * (1 / (2 * np.pi))

async def main():
    # Set motor parameters
    motor_shoulder_controller = moteus.Controller(id=shoulder_id)
    # Elbow motor
    motor_elbow_controller = moteus.Controller(id=elbow_id)

    # stop both motors
    await motor_shoulder_controller.set_stop()
    await motor_elbow_controller.set_stop()

    print("Enabling Motors..")
    print("Start")

    shoulder_pos = 0.0
    elbow_pos = 0.0
    shoulder_vel = 0.0
    elbow_vel = 0.0
    alpha = 0.3
    shoulder_velocity_filtered = 0
    elbow_velocity_filtered = 0
    dp_shoulder = 0.0
    dp_elbow = 0.0
    start_time = time.time()
    tau = np.zeros(2)
    try:
        for i in range(numSteps):

            dt = time.time()
            traj_time = dt - start_time
            time_vec[i] = traj_time

            state1 = await motor_shoulder_controller.set_position(
                position=None,
                velocity=None,
                kp_scale=kp_scale_sh,
                kd_scale=kd_scale_sh,
                stop_position=None,
                feedforward_torque=tau[0],
                maximum_torque=max_torque,
                watchdog_timeout=None,
                query=True)
            state2 = await motor_elbow_controller.set_position(
                position=None,
                velocity=None,
                kp_scale=kp_scale_el,
                kd_scale=kd_scale_el,
                stop_position=None,
                feedforward_torque=tau[1],
                maximum_torque=max_torque,
                watchdog_timeout=None,
                query=True)

            # Compute friction torques with identified model
            # tau = np.dot(yb_matrix(-9.81, 6, 6, 0.15, [shoulder_pos, elbow_pos], [0., 0.], [0., 0.]), xb_vector_id_mjbots())
            tau = np.dot(yb_friction_matrix([shoulder_velocity_filtered, elbow_velocity_filtered]), est_fric_param_vector)

            # store the measured sensor data of position, velocity and torque in each time step
            shoulder_position[i] = rev2rad(state1.values[moteus.Register.POSITION])
            #shoulder_velocity[i] = rev2rad(state1.values[moteus.Register.VELOCITY])
            shoulder_velocity[i] = shoulder_velocity_filtered
            shoulder_torque[i] = state1.values[moteus.Register.TORQUE]
            elbow_position[i] = rev2rad(state2.values[moteus.Register.POSITION])
            #elbow_velocity[i] = rev2rad(state2.values[moteus.Register.VELOCITY])
            elbow_velocity[i] = elbow_velocity_filtered
            elbow_torque[i] = state2.values[moteus.Register.TORQUE]
            desired_shoulder_torque[i] = tau[0]
            desired_elbow_torque[i] = tau[1]
            print('commanded shoulder torque: ', tau[0])
            print('commanded elbow torque: ', tau[1])
            print()

            dt = time.time() - t0
            dp_shoulder = (shoulder_position[i] - shoulder_position[i-1]) / dt
            dp_elbow = (elbow_position[i] - elbow_position[i-1]) / dt

            elbow_velocity_filtered = alpha*dp_elbow + \
                           (1.-alpha)*elbow_velocity_filtered
            shoulder_velocity_filtered = alpha*dp_shoulder + \
                           (1.-alpha)*shoulder_velocity_filtered

    except Exception as e:
        print(e)
    finally:
        print("Disabling Motors...")
        os.system(f"sudo moteus_tool --stop -t{shoulder_id},{elbow_id}")
        await motor_shoulder_controller.set_stop()
        await motor_elbow_controller.set_stop()

        plt.plot(time_vec, shoulder_position)
        plt.plot(time_vec, elbow_position)
        plt.xlabel("Time (s)")
        plt.ylabel("Position (rad)")
        plt.title("Position (rad) vs Time (s)")
        plt.legend(['Shoulder', 'Elbow'])
        plt.savefig(results_dir + 'pos.pdf')
        plt.show()
        plt.plot(time_vec, shoulder_velocity)
        plt.plot(time_vec, elbow_velocity)
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (rad/s)")
        plt.legend(['Shoulder', 'Elbow'])
        plt.title("Velocity (rad/s) vs Time (s)")
        plt.savefig(results_dir + 'vel.pdf')
        plt.show()
        plt.plot(time_vec, shoulder_torque)
        plt.plot(time_vec, elbow_torque)
        plt.plot(time_vec, desired_shoulder_torque)
        plt.plot(time_vec, desired_elbow_torque)
        plt.xlabel("Time (s)")
        plt.ylabel("Torque (Nm)")
        plt.title("Torque (Nm) vs Time (s)")
        plt.legend(['Measured Shoulder', 'Measured Elbow', 'Desired Shoulder', 'Desired Elbow'])
        plt.savefig(results_dir + 'torque.pdf')
        plt.show()
        filtered_shoulder_torque = running_mean(np.array(shoulder_torque), 10)
        filtered_elbow_torque = running_mean(np.array(elbow_torque), 10)
        time_vec_filtered = running_mean(np.array(time_vec), 10)

        filtered_desired_shoulder_torque = running_mean(np.array(desired_shoulder_torque), 10)
        filtered_desired_elbow_torque = running_mean(np.array(desired_elbow_torque), 10)

        plt.plot(time_vec_filtered, filtered_shoulder_torque)
        plt.plot(time_vec_filtered, filtered_elbow_torque)
        plt.plot(time_vec_filtered, filtered_desired_shoulder_torque)
        plt.plot(time_vec_filtered, filtered_desired_elbow_torque)
        plt.xlabel("Time (s)")
        plt.ylabel("Torque (Nm)")
        plt.title("Filtered Torque (Nm) vs Time (s) with moving average filter (window = 100)")
        plt.legend(['Measured Shoulder', 'Measured Elbow', 'Desired Shoulder', 'Desired Elbow'])
        plt.savefig(results_dir + 'filtered_torque.pdf')
        plt.show()

        measured_csv_data = np.array([np.array(time_vec),
                                      np.array(shoulder_position),
                                      np.array(shoulder_velocity),
                                      np.array(shoulder_torque),
                                      np.array(elbow_position),
                                      np.array(elbow_velocity),
                                      np.array(elbow_torque)]).T
        np.savetxt(results_dir + "measured_data.csv", measured_csv_data, delimiter=',', header="time,shoulder_pos,shoulder_vel,shoulder_torque,elbow_pos,elbow_vel,elbow_torque", comments="")

if __name__ == "__main__":
    working_dir = os.path.dirname(os.path.abspath(__file__))
    results_dir = os.join(working_dir, '/data/hopping_leg_v2_mjbots/experiments/DC-R1/friction_compensation/')
    if not os.path.isdir(results_dir):
        os.makedirs (results_dir)

    shoulder_id = 1
    elbow_id = 2
    os.system(f"sudo moteus_tool --zero-offset -t{shoulder_id},{elbow_id}")
    kp_scale_sh = 0
    kd_scale_sh = 0
    kp_scale_el = 0
    kd_scale_el = 0
    max_torque = 8
    numSteps = 10000
    time_vec = np.zeros(numSteps)

    shoulder_position = np.zeros(numSteps)
    elbow_position = np.zeros(numSteps)
    shoulder_velocity = np.zeros(numSteps)
    elbow_velocity = np.zeros(numSteps)
    shoulder_torque = np.zeros(numSteps)
    elbow_torque = np.zeros(numSteps)
    desired_shoulder_torque = np.zeros(numSteps)
    desired_elbow_torque = np.zeros(numSteps)

    est_fric_param_vector = np.array([xb_vector_id_mjbots()[2:4], xb_vector_id_mjbots()[8:10]]).flatten()

    if input("When ready press y:") == 'y':
        asyncio.run(main())
