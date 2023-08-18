"""
=======
PARCOUR
=======
"""

import toml
import numpy as np
from functools import reduce
# SPINE
from spine.controller.abstract import AbstractController

# HOPPING LEG
# from controllers.PD_JS.ctrl import Controller_Stiffness_Joint
# from controllers.PD_CS.ctrl import Controller_Stiffness_Cartesian
from hopping_leg.controllers.PD_CJS.ctrl import Controller_Stiffness_Combined
from hopping_leg.controllers.PD_JS.ctrl import Controller_Stiffness_Joint
from hopping_leg.controllers.Exertion.ctrl import Controller_Stiffness_Cartesian
from hopping_leg.controllers.TESTER.ctrl import test_controller
from hopping_leg.controllers.Flight.ctrl import Controller_Stiffness_Joint as flight

from hopping_leg.state_estimation.contact import contact_effort_threshold
from hopping_leg.state_estimation.pybullet import contact_pybullet

from hopping_leg.parcour_functions.ballistic import launching_velocity
from hopping_leg.parcour_functions.ballistic import force

class StateMachine(AbstractController):
    """
    something informative
    """

    flight_counter = 0
    fc_aux = 0

    q_backflip = np.asarray([0, 0], dtype=float)

    def save(self, path):
        shutil.copyfile(self.gains_file, os.path.join(path, 'gains.toml'))

    def __init__(self,
                 plant,
                 knee_direction,
                 qd_star,

                 controller,
                 gains_file,
                 initial_phase,

                 contact_est,
                 contact_threshold,

                 r_crouched,
                 r_extended,
                 jumping_dict):
        """
        **Arguments**

        ``plant`` [``hopping_leg.plant.plant.HopperPlant`` instance]
          hopping leg plant instance

        ``knee_direction`` [int]
          either 0 or 1

        ``x_star`` [float]
          in the leg's BODY frame of reference, the Cartesian x coordinate of the desired FOOT position

        ``y_star`` [float]
          in the leg's BODY frame of reference, the Cartesian y coordinate of the desired FOOT position

        ``q_star`` [list]
          in the leg's joint space, the position vector of the desired joint space state

        ``qd_star`` [list]
          in the leg's joint space, the velocity vector of the desired joint space state

        ``controller`` [str]
          either ``"JOINT"``, ``"CARTESIAN"`` or ``"COMBINED"``

        ``contact_est`` [str]
          either ``"ET"`` (contact threshold contact estimation) or ``"PB"`` (pybullet contact estimation)
        """

        self.plant = plant
        self.controller = controller
        self.gains_file = gains_file
        self.initial_phase = initial_phase

        self.gains = toml.load(gains_file)

        self.x_extended = 0.7 * (plant.L1 + plant.L2)
        self.x_crouched = 3 / 2 * 0.15

        self.r_crouched = r_crouched
        self.state_machine = {

            'FLIGHT': lambda state, r, theta, theta_vel: _controller(state,

                                                          **self.gains['FLIGHT'],

                                                          # joint
                                                          q_star=self.q_star_flight,
                                                          qd_star=qd_star,
                                                          T_ff=[0, 0],

                                                          plant=plant,
                                                          knee_direction=knee_direction),

            'TOUCHDOWN': lambda state, r, theta, theta_vel: _controller(state,

                                                             **self.gains['TOUCHDOWN'],

                                                             # joint
                                                             q_star=self.q_star_touchdown,
                                                             qd_star=qd_star,
                                                             T_ff=[0, 0],
                                                             plant=plant,
                                                             knee_direction=knee_direction),

            'EXERTION': lambda state, r, theta, theta_vel: Controller_Stiffness_Cartesian(state,

                                                           **self.gains['EXERTION'],

                                                           # cartesian
                                                           X_star=[self.theta_go],
                                                           Xd_star=[0],
                                                           F_ff=[self.Fx, self.Fy],

                                                           plant=plant,
                                                           knee_direction=knee_direction)
        }
        self.r_extended = r_extended
        self.exertion_distance = r_extended-r_crouched
        self.jumping_dict = jumping_dict

        _controller = {
            'JOINT': Controller_Stiffness_Joint,
            'CARTESIAN': Controller_Stiffness_Cartesian,
            'COMBINED': Controller_Stiffness_Combined,
            'PARCOUR': Controller_Stiffness_Joint,
        }[controller]

        _contact_estimation = {
            'ET': contact_effort_threshold,
            'PB': lambda *args, **kwargs: contact_pybullet(1, 0)
        }[contact_est]

        # LIFTOFF   -> no contact                   -> FLIGHT
        # FLIGHT    -> contact and below x_extended -> TOUCHDOWN
        # TOUCHDOWN -> 0 vertical velocity          -> EXERTION
        # EXERTION  -> leg extended                 -> LIFTOFF

        self.transition = {
            'FLIGHT': lambda state, r, theta, count: "TOUCHDOWN" if _contact_estimation(state[2, :], contact_threshold[count-1])
            and plant.calculate_r(*state[0, :]) <= self.jumping_dict["flight_r"][count-1] - 0.005
            else "FLIGHT",

            'TOUCHDOWN': lambda state, r, theta, count: "EXERTION" if
            np.linalg.norm(plant.forward_velocity(*state[0, :], *state[1, :])) < 0.01
            and r <= self.jumping_dict["exertion_r_crouched"][count] + 0.01
            and theta >= self.jumping_dict["des_theta"][count] - np.radians(5)
            and count < 1
            else "TOUCHDOWN",

            'EXERTION': lambda state, r, theta, count: "FLIGHT" if
            plant.calculate_r(*state[0, :]) >= self.jumping_dict["exertion_r_extended"][count] - 0.01
            # or not _contact_estimation(state[2, :], contact_threshold)
            else "EXERTION"}

    def get_control_output(self, n):
        self.n = n
        count = self.flight_counter
        # initial phase
        if n == 0:
            self.RECORD.phase[0] = self.initial_phase
            self.q_star_touchdown = self.plant.kinematics_new_frame(self.r_crouched,
                                                                    self.jumping_dict["des_theta"][self.flight_counter])
            self.tester_theta = np.radians(70)
            theta_vel = 0
        # ---------------------------------------------

        # state
        state = self.RECORD.state(n)
        # print(np.degrees(state[0, :]))
        r, theta = self.plant.inverse_kinematics_new_frame(*state[0, :])
        self.theta = theta
        if n > 0:
            theta_vel = (theta - self.theta_old) * 200
        self.theta_old = theta

        # phase
        phase = self.transition[self.RECORD.phase[max(n - 1, 0)]](state, r, theta, count)
        self.RECORD.phase[n] = phase
        trans = phase != self.RECORD.phase[n - 1]

        # initialization - exertion phase
        if trans and phase == 'EXERTION':
            print(f'{self.flight_counter}----------------------------------------------------EXERTION---{n}---{np.degrees(theta)}')
            self.theta_go = theta
            m = 1.26
            r_crouched = self.jumping_dict["exertion_r_crouched"][self.flight_counter]
            r_extended = self.jumping_dict["exertion_r_extended"][self.flight_counter]
            v_des = self.jumping_dict["launching_velocity"][self.flight_counter]
            theta_des = self.jumping_dict["des_theta"][self.flight_counter]
            v_rea = np.sqrt(v_des**2 * (np.sin(2 * theta_des) / np.sin(2 * theta)))
            F = m * v_rea**2 / (2 * (r_extended - r_crouched))
            print(f'FORCE = {F}')
            self.Fx = np.sin(self.theta_go) * F
            self.Fy = -np.cos(self.theta_go) * F

        # initialization - flight phase
        if trans and phase == 'FLIGHT':
            self.q_star_flight = self.plant.kinematics_new_frame(self.jumping_dict["flight_r"][self.flight_counter],
                                                                 self.jumping_dict["flight_theta"][self.flight_counter])
            if self.flight_counter == 3:
                self.flight_counter = 3
            else:
                self.flight_counter += 1
            print(f'{self.flight_counter}----------------------------------------------------FLIGHT---{n}')

        # initialization - touchdown phase
        if trans and phase == 'TOUCHDOWN':
            print(f'{self.flight_counter}----------------------------------------------------TOUCHDOWN---{n}')
            self.q_star_touchdown = self.plant.kinematics_new_frame(self.jumping_dict["exertion_r_crouched"][self.flight_counter],
                                                                    self.jumping_dict["des_theta"][self.flight_counter])

        tau = self.state_machine[phase](state, r, theta, theta_vel)
        print(f'tau: {tau}')
        # control input
        if tau[0] >= 10 or tau[1] >= 10:
            print(f'LIMIT EXEDED -> tau = {tau}')
        # print(F'theta: {np.degrees(theta)}deg')
        # tau = [0, 0]
        u = np.array([[0, 0],
                      [0, 0],
                      tau])

        return u
