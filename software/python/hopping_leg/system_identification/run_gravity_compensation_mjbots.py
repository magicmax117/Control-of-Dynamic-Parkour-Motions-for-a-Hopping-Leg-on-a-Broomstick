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

            # Compute gravity torques with identified model

            # store the measured sensor data of position, velocity and torque in each time step
            shoulder_position[i] = rev2rad(state1.values[moteus.Register.POSITION])
            shoulder_velocity[i] = rev2rad(state1.values[moteus.Register.VELOCITY])
            shoulder_torque[i] = state1.values[moteus.Register.TORQUE]
            elbow_position[i] = rev2rad(state2.values[moteus.Register.POSITION])
            elbow_velocity[i] = rev2rad(state2.values[moteus.Register.VELOCITY])
            elbow_torque[i] = state2.values[moteus.Register.TORQUE]
            shoulder_pos = shoulder_position[i]
            elbow_pos = elbow_position[i]
            tau = np.dot(yb_matrix(-9.81, 6, 6, 0.15, [shoulder_pos, elbow_pos], [0., 0.], [0., 0.]), xb_vector_id_mjbots())
            desired_shoulder_torque[i] = tau[0]
            desired_elbow_torque[i] = tau[1]
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
        plt.savefig('gravity_compensation/pos.pdf')
        plt.show()
        plt.plot(time_vec, shoulder_velocity)
        plt.plot(time_vec, elbow_velocity)
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (rad/s)")
        plt.legend(['Shoulder', 'Elbow'])
        plt.title("Velocity (rad/s) vs Time (s)")
        plt.savefig('gravity_compensation/vel.pdf')
        plt.show()
        plt.plot(time_vec, shoulder_torque)
        plt.plot(time_vec, elbow_torque)
        plt.plot(time_vec, desired_shoulder_torque)
        plt.plot(time_vec, desired_elbow_torque)
        plt.xlabel("Time (s)")
        plt.ylabel("Torque (Nm)")
        plt.title("Torque (Nm) vs Time (s)")
        plt.legend(['Measured Shoulder', 'Measured Elbow', 'Desired Shoulder', 'Desired Elbow'])
        plt.savefig('gravity_compensation/torque.pdf')
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
        plt.savefig('gravity_compensation/filtered_torque.pdf')
        plt.show()

        measured_csv_data = np.array([np.array(time_vec),
                                      np.array(shoulder_position),
                                      np.array(shoulder_velocity),
                                      np.array(shoulder_torque),
                                      np.array(elbow_position),
                                      np.array(elbow_velocity),
                                      np.array(elbow_torque)]).T
        np.savetxt("gravity_compensation/measured_data.csv", measured_csv_data, delimiter=',', header="time,shoulder_pos,shoulder_vel,shoulder_torque,elbow_pos,elbow_vel,elbow_torque", comments="")

if __name__ == "__main__":
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

    if input("When ready press y:") == 'y':
        asyncio.run(main())
