import sys
import time
from motor_driver.canmotorlib import CanMotorController
import matplotlib.pyplot as plt
import numpy as np
from state_vector import xb_vector_id_tmotors
from regressor_matrix import yb_matrix


def setZeroPosition(motor, initPos):
    pos = initPos
    while abs(np.rad2deg(pos)) > 0.5:
        pos, vel, curr = motor.set_zero_position()
        print("Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel),
                                                                curr))


# Motor ID
motor_shoulder_id = 0x08
motor_elbow_id = 0x09
# CAN port
can_port = 'can0'

# if len(sys.argv) != 2:
#     print('Provide CAN device name (can0, slcan0 etc.)')
#     sys.exit(0)

# print("Using Socket {} for can communication".format(sys.argv[1],))

# Create motor controller objects
motor_shoulder_controller = CanMotorController(can_port, motor_shoulder_id)
motor_elbow_controller = CanMotorController(can_port, motor_elbow_id)
print("Enabling Motors..")

shoulder_pos, shoulder_vel, shoulder_torque = motor_shoulder_controller.enable_motor()
print("Shoulder Motor Status: Pos: {}, Vel: {}, Torque: {}".format(shoulder_pos, shoulder_vel, shoulder_torque))

elbow_pos, elbow_vel, elbow_torque = motor_elbow_controller.enable_motor()
print("Elbow Motor Status: Pos: {}, Vel: {}, Torque: {}".format(elbow_pos, elbow_vel, elbow_torque))

print("Setting Shoulder Motor to Zero Position...")
setZeroPosition(motor_shoulder_controller, shoulder_pos)
print("Setting Elbow Motor to Zero Position...")
setZeroPosition(motor_elbow_controller, elbow_pos)

print("Start")
numSteps = 50000
time_vec = np.zeros(numSteps)

shoulder_position = np.zeros(numSteps)
elbow_position = np.zeros(numSteps)
shoulder_velocity = np.zeros(numSteps)
elbow_velocity = np.zeros(numSteps)
shoulder_torque = np.zeros(numSteps)
elbow_torque = np.zeros(numSteps)
desired_shoulder_torque = np.zeros(numSteps)
desired_elbow_torque = np.zeros(numSteps)
tau = np.zeros(2)

start_time = time.time()

shoulder_pos = 0.0
elbow_pos = 0.0
shoulder_vel = 0.0
elbow_vel = 0.0

for i in range(numSteps):

    dt = time.time()
    traj_time = dt - start_time
    time_vec[i] = traj_time

    # Send only the tau_ff command and use the in-built low level controller
    shoulder_pos, shoulder_vel, shoulder_tau = motor_shoulder_controller.send_rad_command(0.0,0.0,0.0,0.0,tau[0])
    elbow_pos, elbow_vel, elbow_tau = motor_elbow_controller.send_rad_command(0.0,0.0,0.0,0.0,tau[1])
    
    # Compute gravity torques with identified model: Q = Yb * xb
    tau = np.dot(yb_matrix(-9.81, 6, 0.3, [shoulder_pos, elbow_pos], [0., 0.], [0., 0.]), xb_vector_id_tmotors())

    # Store data in lists
    shoulder_position[i] = shoulder_pos
    shoulder_velocity[i] = shoulder_vel
    shoulder_torque[i] = shoulder_tau
    elbow_position[i] = elbow_pos
    elbow_velocity[i] = elbow_vel
    elbow_torque[i] = elbow_tau
    
    desired_shoulder_torque[i] = tau[0]
    desired_elbow_torque[i] = tau[1]

print("Disabling Motors...")
shoulder_pos, shoulder_vel, shoulder_tau = motor_shoulder_controller.disable_motor()

print("Shoulder Motor Status: Pos: {}, Vel: {}, Torque: {}".format(shoulder_pos, shoulder_vel, shoulder_tau))
elbow_pos, elbow_vel, elbow_tau = motor_elbow_controller.disable_motor()

print("Elbow Motor Status: Pos: {}, Vel: {}, Torque: {}".format(elbow_pos, elbow_vel, elbow_tau))

plt.plot(time_vec, shoulder_position)
plt.plot(time_vec, elbow_position)
plt.xlabel("Time (s)")
plt.ylabel("Position (rad)")
plt.title("Position (rad) vs Time (s)")
plt.legend(['Shoulder', 'Elbow'])
plt.show()
plt.plot(time_vec, shoulder_velocity)
plt.plot(time_vec, elbow_velocity)
plt.xlabel("Time (s)")
plt.ylabel("Velocity (rad/s)")
plt.legend(['Shoulder', 'Elbow'])
plt.title("Velocity (rad/s) vs Time (s)")
plt.show()
plt.plot(time_vec, shoulder_torque)
plt.plot(time_vec, elbow_torque)
plt.plot(time_vec, desired_shoulder_torque)
plt.plot(time_vec, desired_elbow_torque)
plt.xlabel("Time (s)")
plt.ylabel("Torque (Nm)")
plt.title("Torque (Nm) vs Time (s)")
plt.legend(['Measured Shoulder', 'Measured Elbow', 'Desired Shoulder', 'Desired Elbow'])
plt.show()


def running_mean(x, N):
    cumsum = np.cumsum(np.insert(x, 0, 0))
    return (cumsum[N:] - cumsum[:-N]) / float(N)


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
plt.show()

measured_csv_data = np.array([np.array(time_vec),
                    np.array(shoulder_position),
                    np.array(shoulder_velocity),
                    np.array(shoulder_torque),
                    np.array(elbow_position),
                    np.array(elbow_velocity),
                    np.array(elbow_torque)]).T
np.savetxt("measured_data.csv", measured_csv_data, delimiter=',', header="time,shoulder_pos,shoulder_vel,shoulder_torque,elbow_pos,elbow_vel,elbow_torque", comments="")
