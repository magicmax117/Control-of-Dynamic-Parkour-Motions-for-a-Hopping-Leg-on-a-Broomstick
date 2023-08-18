"""
Implementation
==============
"""

import numpy as np

# STABILIZATION FRAMEWORK
from stabilization.control.controllers import AbstractController

# HOPPING LEG
from hopping_leg.controllers.PD.ctrl import Ctrl as PD
    

class Ctrl(AbstractController):
    """
    .. warning:

       This implementation is for use with the DFKI Underactuated Lab ``stabilization``
       controller scheduling framework.

    Joint space stiffness controller. Internally it applies the general stiffness
    controller (``hopping_leg.controllers.PD.ctrl.Ctrl``) to the joints of the leg,
    with the desired joint position and velocity of each joint provided by the user
    in the state machine configuration file.
    """
    
    def __init__(self,inputv, conf, key):
        
        super().__init__(inputv, conf, key)
        print("Initializing joint space PD controller")
        print(f"Kp (diag):     {self.conf.Kp}")
        print(f"Kd (diag):     {self.conf.Kd}")
        print(f"desired q:     {self.conf.q_star}")
        print(f"desired qd:    {self.conf.qd_star}")
        print(f"torque limits: {self.conf.tau_max}")

        # gain matrices
        self.conf.Kp      = np.diag(self.conf.Kp)
        self.conf.Kd      = np.diag(self.conf.Kd)
        
        # desired state vectors
        self.conf.q_star  = np.array(self.conf.q_star)
        self.conf.qd_star = np.array(self.conf.qd_star)

        # zero feedforward torque
        self.tau_ff = [0, 0]
        
    def calc_u(self, state, cmd, t):
        
        unclipped = PD(x    = state.q,
                       xd   = state.qd,
                       u    = self.conf.q_star,
                       ud   = self.conf.qd_star,
                       Kp   = self.conf.Kp,
                       Kd   = self.conf.Kd,
                       F_ff = self.tau_ff)
        
        self.u.tau = np.clip(unclipped,
                             [-self.conf.tau_max, -self.conf.tau_max],
                             [ self.conf.tau_max,  self.conf.tau_max])


def Controller_Stiffness_Joint(state,
                               q_star,
                               qd_star,
                               Kp_JS,
                               Kd_JS,
                               T_ff,
                               **kwargs):
    """
    .. warning:

       This implementation is for standalone use, and is **not** suited for use with the DFKI
       ``stabilization`` controller scheduling framework.

    Joint space stiffness controller. Internally it applies the general stiffness
    controller (``hopping_leg.controllers.PD.ctrl.Ctrl``) to the joints of the leg,
    with the desired joint position and velocity of each joint being ``q_star`` and ``qd_star``.

    Returns a control **torques** vector.
    
    **Arguments**

    ``state`` [np.ndarray]
      ``3`` by ``m`` vector, containing the angular position, velocity and acceleration of each motor in the system
    
    ``q_star`` [list]
      definition above
    
    ``qd_star`` [list]
      definition above
    
    ``Kp_JS`` [list]
      diagonal of proportional gain matrix
    
    ``Kd_JS`` [list]
      diagonal of derivative gain matrix
    
    ``T_ff`` [list]
      feedforward torques
    
    **Output**
    
    ``T`` [np.ndarray]
      Control torque vector
    """

    a = lambda l, **kwargs: np.asarray(l, **kwargs)
    d = lambda l, **kwargs: np.diag(l, **kwargs)
    
    # input validation
    q_star  = a(q_star)
    qd_star = a(qd_star)
    T_ff    = a(T_ff)
    Kp_JS   = d(Kp_JS)
    Kd_JS   = d(Kd_JS)
    
    T = PD(x    = state[0, :],
           xd   = state[1, :],
           u    = q_star,
           ud   = qd_star,
           Kp   = Kp_JS,
           Kd   = Kd_JS,
           F_ff = T_ff)

    return T
