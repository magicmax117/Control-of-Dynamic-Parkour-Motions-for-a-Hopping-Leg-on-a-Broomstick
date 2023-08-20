"""
PARCOUR for Pybullet
========
"""
import os
import toml
import numpy as np
from functools import reduce
# SPINE
from spine.controller.abstract import AbstractController

# HOPPING LEG
# from controllers.PD_JS.ctrl import Controller_Stiffness_Joint
# from controllers.PD_CS.ctrl import Controller_Stiffness_Cartesian
#from hopping_leg.controllers.PD_CJS.ctrl import Controller_Stiffness_Combined
from hopping_leg.controllers.PD_JS.ctrl import Controller_Stiffness_Joint
from hopping_leg.controllers.Exertion.ctrl import Controller_Stiffness_Cartesian
#from hopping_leg.controllers.TESTER.ctrl import test_controller
#from hopping_leg.controllers.Flight.ctrl import Controller_Stiffness_Joint as flight

from hopping_leg.state_estimation.contact import contact_effort_threshold
from hopping_leg.state_estimation.pybullet import contact_pybullet

from hopping_leg.parcour_functions.ballistic import edit
from hopping_leg.parcour_functions.ballistic import force

class StateMachine(AbstractController):
    """
        STAGING    ->  Initialized       ->  TOUCHDOWN
        TOUCHDOWN  ->  Launch Clearance  ->  EXERTION
        EXERTION   ->  Thrust Stop       ->  FLIGHT
        FLIGHT     ->  Contact           ->  TOUCHDOWN
    """

    flight_counter = 0
    q_backflip = np.asarray([0, 0], dtype=float)

    def save(self, path):
        shutil.copyfile(self.gains_file, os.path.join(path, 'gains.toml'))

    def __init__(self,
                 duration,
                 control_freq,
                 plant,
                 knee_direction,
                 qd_star,
                 controller,
                 gains_file,
                 initial_phase,
                 contact_est,
                 contact_threshold,
                 jumping_dict,
                 ENCODER,
                 FEEDBACK,
                 path):
        """
        **Arguments**

        ``plant`` [``hopping_leg.plant.plant.HopperPlant`` instance]
          hopping leg plant instance

        ``knee_direction`` [int]
          either 0 or 1
        """

        self.duration = duration
        self.control_freq = control_freq
        self.plant = plant
        self.controller = controller
        self.gains_file = gains_file
        self.initial_phase = initial_phase
        self.gains = toml.load(gains_file)
        self.jumping_dict = jumping_dict
        self.ENCODER = ENCODER
        self.FEEDBACK = FEEDBACK
        self.path = path
        if ENCODER:
            from hopping_leg.state_estimation.encoder import Encoder
            self.encoder = Encoder(self.duration * self.control_freq)
        else:
            self.encoder = None
        self.state_machine = {
            'TOUCHDOWN': lambda state, r, theta, theta_vel: _controller(state,
                                                                      **self.gains['TOUCHDOWN'],
                                                                      q_star=self.q_star_touchdown,
                                                                      qd_star=qd_star,
                                                                      T_ff=[0, 0],
                                                                      plant=plant,
                                                                      knee_direction=knee_direction),

            'REPOSITION': lambda state, r, theta, theta_vel: _controller(state,
                                                                        **self.gains['REPOSITION'],
                                                                        q_star=self.q_star_reposition,
                                                                        qd_star=qd_star,
                                                                        T_ff=[0, 0],
                                                                        plant=plant,
                                                                        knee_direction=knee_direction),

            'STAGING': lambda state, r, theta, theta_vel: _controller(state,
                                                                        **self.gains['STAGING'],
                                                                        q_star=self.q_star_staging,
                                                                        qd_star=qd_star,
                                                                        T_ff=[0, 0],
                                                                        plant=plant,
                                                                        knee_direction=knee_direction),

            'EXERTION': lambda state, r, theta, theta_vel: Controller_Stiffness_Cartesian(state,
                                                                                          **self.gains['EXERTION'],
                                                                                          X_star=[self.theta_go],
                                                                                          Xd_star=[0],
                                                                                          F_ff=[self.Fx, self.Fy],
                                                                                          plant=plant,
                                                                                          knee_direction=knee_direction),

            'FLIGHT': lambda state, r, theta, theta_vel: _controller(state,
                                                                     **self.gains['FLIGHT'],
                                                                     q_star=self.q_star_flight,
                                                                     qd_star=qd_star,
                                                                     T_ff=[0, 0],
                                                                     plant=plant,
                                                                     knee_direction=knee_direction),

            'TESTER': lambda state, r, theta, theta_vel: test_controller(state,
                                                                         r,
                                                                         theta,
                                                                         theta_vel,
                                                                         **self.gains['TESTER'],
                                                                         theta_star=self.tester_theta,
                                                                         thetad_star=0)
        }
        self.transition = {
            'TOUCHDOWN': lambda state, r, theta, count, r_go: "REPOSITION" if
            r <= self.jumping_dict["touchdown_r"] and
            np.linalg.norm(plant.forward_velocity(*state[0, :], *state[1, :])) < 0.1
            # and self.jumping_dict["staging_theta"] - np.radians(5) <= theta
            # <= self.jumping_dict["staging_theta"] + np.radians(5)
            else "TOUCHDOWN",

            'REPOSITION': lambda state, r, theta, count, r_go: "STAGING" if
            np.linalg.norm(plant.forward_velocity(*state[0, :], *state[1, :])) < 0.1
            and self.jumping_dict["staging_theta"][count] - np.radians(5) <= theta
            <= self.jumping_dict["staging_theta"][count] + np.radians(5)
            else "REPOSITION",

            'STAGING': lambda state, r, theta, count, r_go: "EXERTION" if
            np.linalg.norm(plant.forward_velocity(*state[0, :], *state[1, :])) < 0.003
            and self.jumping_dict["staging_r"] >= r
            >= self.jumping_dict["staging_r"] - 0.02
            and self.jumping_dict["staging_theta"][count] - np.radians(2) <= theta
            <= self.jumping_dict["staging_theta"][count] + np.radians(2)
            and self.jumping_dict["number_of_jumps"] > count
            else "STAGING",

            'EXERTION': lambda state, r, theta, count, r_go: "FLIGHT" if
            # plant.calculate_r(*state[0, :]) >= self.jumping_dict["exertion_r_extended"][count] - 0.01
            r >= 0.2
            # r_go + jumping_dict["exertion_d"]
            # or not _contact_estimation(state[2, :], contact_threshold)
            else "EXERTION",

            'FLIGHT': lambda state, r, theta, count, r_go: "TOUCHDOWN" if
            _contact_estimation(state[2, :], contact_threshold[count - 1])
            # and plant.calculate_r(*state[0, :]) <= self.jumping_dict["flight_r"][count - 1] - 0.003
            and r <= self.jumping_dict["flight_r"]
            else "FLIGHT",

            'TESTER': lambda state, r, theta, count, r_go: "TESTER"}

        _controller = {
            #'JOINT': Controller_Stiffness_Joint,
            #'CARTESIAN': Controller_Stiffness_Cartesian,
            #'COMBINED': Controller_Stiffness_Combined,
            'PARCOUR': Controller_Stiffness_Joint,
        }[controller]

        _contact_estimation = {
            'ET': contact_effort_threshold,
            'PB': lambda *args, **kwargs: contact_pybullet(1, 0)
        }[contact_est]

    def get_control_output(self, n):
        self.n = n
        count = self.flight_counter
        # print(f'{n} -> t = {self.RECORD.t[n]}')

        # initial phase
        if n == 0:
            self.RECORD.phase[0] = self.initial_phase
            self.q_star_touchdown = self.plant.kinematics_new_frame(self.jumping_dict["touchdown_r"],
                                                                    self.jumping_dict["touchdown_theta"])
            # self.q_star_stageing = self.plant.kinematics_new_frame(self.jumping_dict["stage_r"][count],
            #                                                        self.jumping_dict["stage_theta"][count])
            # # self.q_star_stageing = [0, 0]
            # self.tester_theta = np.radians(70)
            theta_vel = 0
            self.r_go = 0

        # state
        state = self.RECORD.state(n)
        r, theta = self.plant.inverse_kinematics_new_frame(*state[0, :])
        # print(np.linalg.norm(self.plant.forward_velocity(*state[0, :], *state[1, :])))
        self.theta = theta
        if n > 0:
            theta_vel = (theta - self.theta_old) * 200
        self.theta_old = theta

        # phase
        phase = self.transition[self.RECORD.phase[max(n - 1, 0)]](state, r, theta, count, self.r_go)
        # if n > 100:
        #     phase = self.transition[self.RECORD.phase[max(n - 1, 0)]](state, r, theta, count, self.r_go)
        # else:
        #     phase = self.initial_phase
        # if phase == 'EXERTION' and n == 3:
        #     print('-- DESIRED POSITION REACHED --')
        #     phase = 'TOUCHDOWN'
        self.RECORD.phase[n] = phase
        trans = phase != self.RECORD.phase[n - 1]

        # print(f'Theta: {np.round(np.degrees(theta), 2)} deg ------------- Radius: {np.round(r, 5)} m'
        #       f'----------- {phase}')

        if self.ENCODER:
            stick_state = self.encoder.read(n)
            # print(stick_state)

        # initialization - touchdown phase
        if trans and phase == 'TOUCHDOWN':
            print(f'{count}--------Theta: {np.round(np.degrees(theta), 2)} deg--------TOUCHDOWN---{n}')
            self.q_star_touchdown = self.plant.kinematics_new_frame(self.jumping_dict["touchdown_r"],
                                                                    self.jumping_dict["staging_theta"][count])

        # initialization - reposition phase
        if trans and phase == 'REPOSITION':
            print(f'{count}--------Theta: {np.round(np.degrees(theta), 2)} deg--------REPOSITION---{n}')
            self.q_star_reposition = self.plant.kinematics_new_frame(self.jumping_dict["staging_r"],
                                                                     self.jumping_dict["staging_theta"][count])

        # initialization - staging phase
        if trans and phase == 'STAGING':
            print(f'{count}--------Theta: {np.round(np.degrees(theta), 2)} deg--------STAGING---{n}')
            if self.FEEDBACK and 0 < count < self.jumping_dict["number_of_jumps"]:
                current_x = 7.2 * stick_state[0, 2] / (2 * np.pi) - np.cos(theta) * r
                print(f'Yaw angle: {np.degrees(stick_state[0, 2])}')
                print(f'current x: {current_x}')
                self.dx = abs(((self.jumping_dict["contact_sequence_x"][count + 1] - current_x) -
                      (-r * np.cos(self.jumping_dict["staging_theta"][count]) + self.jumping_dict["flight_r"] *
                       np.cos(self.jumping_dict["flight_theta"][count]))))
                # print(f'dx: {self.dx}')
                self.dz = ((self.jumping_dict["contact_sequence_z"][count + 1]
                      - self.jumping_dict["contact_sequence_z"][count]) -
                      (-r * np.sin(self.jumping_dict["staging_theta"][count]) + self.jumping_dict["flight_r"] *
                       np.sin(self.jumping_dict["flight_theta"][count])))
                # print(f'dz: {self.dz}')
            self.q_star_staging = self.plant.kinematics_new_frame(self.jumping_dict["staging_r"],
                                                                  self.jumping_dict["staging_theta"][count])

        # initialization - exertion phase
        if trans and phase == 'EXERTION':
            print(f'{count}--------Theta: {np.round(np.degrees(theta), 2)} deg--------EXERTION---{n}---{np.degrees(theta)}')
            # if self.ENCODER:
            #     U = 0.2 * 36
            #     r = U / (2 * np.pi)
            #     x = U * stick_state[0, 0] / (2 * np.pi) - np.cos(theta) * r
            #     x_diff = x - contact_sequence_x[count]
            # chnages log
            # self.theta_go = theta
            # self.r_go = r
            # m = 1.4
            # v_des = self.jumping_dict["launch_velocity"][count]
            # v_rea = np.sqrt(v_des ** 2 * (np.sin(2 * self.jumping_dict["staging_theta"][count]) / np.sin(2 * theta)))
            # F = m * v_rea**2 / (2 * self.jumping_dict["exertion_d"])
            # self.Fx = np.sin(self.theta_go) * F
            # self.Fy = -np.cos(self.theta_go) * F
            # v_rea = np.sqrt(v_des ** 2 * (np.sin(2 * self.jumping_dict["staging_theta"][count]) / np.sin(2 * theta)))
            self.theta_go = theta  # self.jumping_dict["staging_theta"][count]
            self.r_go = r
            exertion_d = 0.2 - r
            m = 1.12
            if self.FEEDBACK and count > 0:
                jump_time = np.sqrt((self.dx * np.tan(np.radians(180) - self.jumping_dict["staging_theta"][count]) - self.dz) /
                                    (0.5 * 9.80665))
                # print(f'jump time: {jump_time}')
                v_des = self.dx / (jump_time * np.cos(np.radians(180) - self.jumping_dict["staging_theta"][count]))
                v_des = max(1.5, v_des)
                print(f'v_feedback: {v_des}')
                print(f'v_OMP: {self.jumping_dict["launch_velocity"][count]}')
            else:
                v_des = self.jumping_dict["launch_velocity"][count]
            v_rea = np.sqrt(v_des ** 2 * (np.sin(2 * self.jumping_dict["staging_theta"][count]) / np.sin(2 * theta)))
            F = m * v_rea**2 / (2 * exertion_d)
            self.Fx = np.sin(self.theta_go) * F
            self.Fy = -np.cos(self.theta_go) * F

        # initialization - flight phase
        if trans and phase == 'FLIGHT':
            print(f'{count}--------Theta: {np.round(np.degrees(theta), 2)} deg--------FLIGHT---{n}')
            self.q_star_flight = self.plant.kinematics_new_frame(self.jumping_dict["flight_r"],
                                                                 self.jumping_dict["flight_theta"][count])
            self.flight_counter += 1

        # control input
        tau = self.state_machine[phase](state, r, theta, theta_vel)
        # print(f'TAU: {tau}')
        # print(f'real tau: {state[2, :]}')
        if tau[0] >= 11 or tau[1] >= 11:
            print(f'LIMIT EXCEEDED -> tau = {tau}')
        # print(F'{phase} - theta: {np.degrees(theta)}deg')
        # print(f'tau: {tau}')
        # print(f'acceleration: {state[2, :]}')
        # tau = [0, 0]
        u = np.array([[0, 0],
                      [0, 0],
                      tau])
        # pm, vm, tm = self.RECORD.matrix_msr(0)
        # print(pm)

        if self.ENCODER and self.RECORD.t[n] >= self.duration - 1:
            self.encoder.save(self.path, self.RECORD, n)
            print('+++++++++++++++++++++ SAVED +++++++++++++++++++++')
            print('+++++++++++++++++++++ SAVED +++++++++++++++++++++')
            print('+++++++++++++++++++++ SAVED +++++++++++++++++++++')
            self.ENCODER = False
            # print(self.RECORD.state)
            # print(f'TIME VECTOR: {self.RECORD.columns}')
        return u
