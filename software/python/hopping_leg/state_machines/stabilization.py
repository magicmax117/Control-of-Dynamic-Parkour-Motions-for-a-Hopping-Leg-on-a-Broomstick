"""
Stabilization Framework
=======================

DFKI Stabilization framework state machine class.
"""

import os
import time
import shutil

import numpy as np

# SPINE
from spine.controller.abstract import AbstractController

# STABILIZATION FRAMEWORK
from stabilization.main import init_stapylize

# HOPPING LEG
from hopping_leg.plant.transforms import JS_to_TS

from hopping_leg.state_estimation.contact     import contact_effort_threshold
from hopping_leg.state_estimation.energy      import energy_kinetic, energy_potential
from hopping_leg.state_estimation.base_height import base_height_hybrid, base_height_peak_last
from hopping_leg.state_estimation.pybullet    import contact_pybullet, base_height_pybullet

from hopping_leg.utils.calc import qdd


class PybulletSM:
    """
    Pybullet method container class.
    """

    def contact_PB(self, n):
        """
        ``pybullet`` contact detection. The body IDs are follow from which object
        was instantiated before, the ground plane or the robot.

        * [1]: robot
        * [0]: ground plane

        .. admonition: Assumption
           :class: warning
        
           As the ``spine.sim.pybullet.simulation`` must be called before
           instantiating new ``pybullet`` bodies, and the ground plane is created
           within it, the ground plane is assumed to have ID 0
        """
        return contact_pybullet(1, 0)
    
    def base_height_PB(self, n):
        """
        ``pybullet`` base height estimation. The ID of the robot follows from order
        of instantiation. The ID of the prismatic joint is 0 as it constrains the
        base of the monoped.
        
        * [1]: robot
        * [0]: prismatic joint
        """
        self.RECORD.base_height_msr[n] = base_height_pybullet(1, 0)


class StateMachine(AbstractController, PybulletSM):
    """
    Monoped state machine

    **Nomenclature**
    
    ``phase``
      equivalent to the *state of the state machine*

    ``state``
      the present attitude, velocity and acceleration of the monoped
      at any given point in time
    """
    
    def __init__(self,
                 source,
                 plant,
                 contact_effort_threshold,
                 contact_estimation_method='effort_threshold',
                 base_height_estimation_method='hybrid',
                 base_height_measurement_method=None):
        
        # save configuration
        self.config = source
        
        # stabilization framework
        self.update_func, self.statev, self.cmdv, self.inputv = init_stapylize(source)
        
        # plant
        self.plant = plant
        
        # contact threshold
        self.contact_effort_threshold = contact_effort_threshold

        # transition function constants
        self.x_extended  = 0.7 * (plant.L1 + plant.L2)
        self.x_crouched  = 1/2 * (plant.L1 + plant.L2)

        # contact estimation method
        # it may be desireable to split this between estimation/measurement methods as is done below
        # if there were interest in comparing estimated and measured contact data
        self.contact_estimation_method = {
            'effort_threshold': self.contact_ET,
            'pybullet':         self.contact_PB
        }[contact_estimation_method]
        
        # base height estimation method
        self.base_height_estimation_method = {
            'hybrid':           self.base_height_HY
        }[base_height_estimation_method]
        
        # base height measurement method
        if base_height_measurement_method is not None:
            self.base_height_measurement_method = {
                'pybullet':         self.base_height_PB
            }[base_height_measurement_method]
        
        # energy shaping
        self.statev.mg        = self.plant.m * self.plant.g
        self.statev.x_liftoff = lambda q_liftoff: self.plant.forward_kinematics(*q_liftoff)[0]
        
    def save(self, path):
        """
        Copy state machine configuration file to record directory. This function
        is called from a low level ``motor_control_loop`` of the ``spine``
        library.
        """
        shutil.copyfile(self.config, os.path.join(path, 'state_machine.toml'))
        
    def contact_ET(self, n):
        """
        Effort threshold contact detection
        """
        return contact_effort_threshold(self.RECORD.state(n)[2, :], self.contact_effort_threshold)

    def base_height_HY(self, n):
        """
        Estimate base height using the ``base_height_hybrid`` method.
        The velocity of the base at liftoff is calculated using a moving
        average (``base_height_velocity``) with a window of 3 timesteps.
        """
        
        self.RECORD.base_height_est[n] = base_height_hybrid(self.plant,
                                                            self.RECORD,
                                                            n,
                                                            window=10,
                                                            delay=0,
                                                            flight_phases=['FLIGHT'])

    def update_state_vector(self, n):
        """
        Update the state machine's state vector (``statev``)
        with the different representations of the state of the
        hopping leg, as follows.

        **State variables**

        ``q`` [numpy.ndarray]
          vector containing the angular positions of the motors in the robot

        ``qd`` [numpy.ndarray]
          vector containing the angular velocities of the motors in the robot

        ``qdd`` [numpy.ndarray]
          vector containing the angular accelerations of the motors in the robot
        
        ``q1`` [numpy.ndarray]
          vector containing the angular position, velocity and acceleration of the
          first motor in your robot configuration TOML file

        ``q2`` [numpy.ndarray]
          vector containing the angular position, velocity and acceleration of the
          first motor in your robot configuration TOML file
        
        ``x`` [numpy.ndarray]
          vector containing the Cartesian state, velocity and acceleration of the
          hopping leg's foot along the ``x`` axis

        ``y`` [numpy.ndarray]
          vector containing the Cartesian state, velocity and acceleration of the
          hopping leg's foot along the ``y`` axis
        """

        state = self.RECORD.state(n)
        
        # obtain angular accelerations
        qdd1 = qdd(self.RECORD, n, 0) if n > 2 else 0
        qdd2 = qdd(self.RECORD, n, 1) if n > 2 else 0

        # UPDATE STATE VARIABLES
        # ======================
        
        # joint space state
        self.statev.q   = state[0, :]
        self.statev.qd  = state[1, :]
        self.statev.qdd = np.array([qdd1, qdd2])
        
        # motor states
        self.statev.q1  = state[:, 0]
        self.statev.q2  = state[:, 1]
        self.statev.q1[2] = qdd1
        self.statev.q2[2] = qdd2
        
        # cartesian state
        cartesian_state = JS_to_TS(self.plant, *self.statev.q1, *self.statev.q2)
        self.statev.x   = cartesian_state[:3]
        self.statev.y   = cartesian_state[3:]
        
        # base height
        self.base_height_estimation_method(n)
        if hasattr(self, 'base_height_measurement_method'):
            self.base_height_measurement_method(n)

        # energies
        self.statev.Ek = energy_kinetic(self.plant, self.RECORD, n)
        self.statev.Ep = energy_potential(self.plant, self.RECORD, n)
        
        # last peak height
        h_base = self.RECORD.base_height_est
        self.statev.last_peak = base_height_peak_last(self.RECORD.t,
                                                      h_base,
                                                      h_peak_min=self.plant.L1 + self.plant.L2)
        
    def calc_transition_functions(self, n):

        # LIFTOFF   -> no contact                   -> FLIGHT
        # FLIGHT    -> contact and below x_extended -> TOUCHDOWN
        # TOUCHDOWN -> 0 vertical velocity          -> EXERTION
        # EXERTION  -> leg extended                 -> LIFTOFF
        
        state   = self.RECORD.state(n)
        contact = self.contact_estimation_method(n)

        self.statev.contact               = contact
        
        self.statev.LIFTOFF_to_FLIGHT     = not contact
        
        self.statev.FLIGHT_to_TOUCHDOWN   = contact and \
                                            self.plant.forward_kinematics(*state[0, :])[0] <= self.x_extended - 0.005
        
        self.statev.TOUCHDOWN_to_EXERTION = np.linalg.norm(self.plant.forward_velocity(*state[0, :], *state[1, :])) < 0.1 and \
                                            self.plant.forward_kinematics(*state[0, :])[0] <= self.x_crouched
        
        self.statev.EXERTION_to_LIFTOFF   = self.plant.forward_kinematics(*state[0, :])[0] >= self.x_extended
        
    def get_control_output(self, n):
        """
        State machine control output
        """
        
        # STATE
        self.update_state_vector(n)
        
        # TRANSITION FUNCTIONS
        self.calc_transition_functions(n)
        
        # CONTROL (and transition model call)
        self.inputv = self.update_func(self.statev, self.cmdv, self.RECORD.t[n])
        
        # PHASE
        # self.RECORD.phase[n] = self.state_machine.transition_model.state
        
        # CARTESIAN FORCE -> TORQUE
        if hasattr(self.inputv, 'spaces'):
            
            if 'cartesian' in self.inputv.spaces:
                J = self.plant.jacobian(*self.statev.q)
                self.inputv.tau += np.matmul(J.T, self.inputv.F)
        
        # control signal
        u = np.array([[0, 0],
                      [0, 0],
                      self.inputv.tau])
        
        return u
