"""
Implementation
==============

This implementation is for use with the DFKI Underactuated Lab ``stabilization`` controller scheduling framework.
"""

import numpy as np

# STABILIZATION FRAMEWORK
from stabilization.control.controllers import AbstractController

# HOPPING LEG
from hopping_leg.controllers.PD.ctrl import Ctrl as PD
    

class Ctrl(AbstractController):
    """
    Cartesian stiffness controller
    """
    
    def __init__(self, inputv, conf, key):
        
        super().__init__(inputv, conf, key)
        print("Initializing Cartesian PD controller")
        print(f"Kp (diag):     {self.conf.Kp}")
        print(f"Kd (diag):     {self.conf.Kd}")
        print(f"desired x:     {self.conf.x_star}")
        print(f"desired xd:    {self.conf.xd_star}")
        print(f"force limit:   {self.conf.F_max}")

        # gain matrices
        self.conf.Kp           = np.diag(self.conf.Kp)
        self.conf.Kd           = np.diag(self.conf.Kd)
        
        # desired state vectors
        self.conf.x_star       = np.array(self.conf.x_star)
        self.conf.xd_star      = np.array(self.conf.xd_star)
        
        # zero feedforward force
        self.F_ff = [0, 0]
        
    def calc_u(self, state, cmd, t):
        
        X  = np.array([state.x[0], state.y[0]])
        Xd = np.array([state.x[1], state.y[1]])
        
        unclipped = PD(x    = X,
                       xd   = Xd,
                       u    = self.conf.x_star,
                       ud   = self.conf.xd_star,
                       Kp   = self.conf.Kp,
                       Kd   = self.conf.Kd,
                       F_ff = self.F_ff)

        self.u.F = np.clip(unclipped,
                           [-self.conf.F_max, -self.conf.F_max],
                           [ self.conf.F_max,  self.conf.F_max])
        
        self.u.spaces = ['cartesian']


def Controller_Stiffness_Cartesian(state,
                                   X_star,
                                   Xd_star,
                                   Kp_CS,
                                   Kd_CS,
                                   F_ff,
                                   plant,
                                   knee_direction,
                                   **kwargs):
    """
    .. warning:

       This implementation is for standalone use, and is **not** suited for use with the DFKI
       ``stabilization`` controller scheduling framework.
    
    Cartesian space stiffness controller. Returns a control **torques** vector.

    This vector is obtained by transforming the resulting Cartesian forces
    using the plant's Jacobian.

    .. note:

       The Jacobian is calculated taking as **reference** the joint space position vector
       corresponding to the **desired Cartesian position vector**.
    
    **Arguments**

    ``state`` [np.ndarray]
      ``3`` by ``m`` vector, containing the angular position, velocity and acceleration of each motor in the system
    
    ``X_star`` [list]
      definition above
    
    ``Xd_star`` [list]
      definition above
    
    ``Kp_CS`` [list]
      diagonal of proportional gain matrix
    
    ``Kd_CS`` [list]
      diagonal of derivative gain matrix
    
    ``F_ff`` [list]
      feedforward forces

    ``plant`` [HopperPlant instance]
      due instance of the HopperPlant class

    ``knee_direction`` [int]
      either 0 or 1

    **Ouput**
    
    ``T`` [np.ndarray]
      Control torque vector
    """

    a = lambda l, **kwargs: np.asarray(l, **kwargs)
    d = lambda l, **kwargs: np.diag(l, **kwargs)

    # input validation
    Xd_star = a(Xd_star)
    F_ff    = F_ff
    Kp   = d(Kp_CS)
    Kd   = d(Kd_CS)
    
    X  = np.array(plant.forward_kinematics(*state[0, :]))
    Xd = np.array(plant.forward_velocity(*state[0, :], *state[1, :]))
    r, theta = np.array(plant.kinematics_inversion(*state[0, :]))
    # print(f'Theta: {np.degrees(theta)}deg')
    thetad = 0

    p = Kp_CS * (X_star - theta)
    # d = Kd.dot(ud - xd)

    tau = p
    Fx = - np.cos(theta) * tau / r
    Fy = - np.sin(theta) * tau / r
    F_ff = a(F_ff)
    # F = np.array([[F_ff[0]], [F_ff[1]]]) + np.array([[Fx], [Fy]])
    F = F_ff + a([Fx[0], Fy[0]])
    # print(F)
    # calculate Jacobian taking as reference the joint space position vector
    # corresponding to the desired Cartesian position vector
    J = plant.jacobian(*state[0, :])
    T = np.matmul(J.T, F)
    # if T.any > 10:
    #     print(f'OVERLOADED TORQUE ON ACTUATORS: {T}Nm')
    # print(abs(F[1])/F[0])
    return T
