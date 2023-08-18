"""
Backflip
========
"""


import toml
import numpy as np

# SPINE
from spine.controller.abstract import AbstractController

# HOPPING LEG
from hopping_leg.controllers.PD_JS.ctrl    import Controller_Stiffness_Joint
from hopping_leg.controllers.PD_CS.ctrl    import Controller_Stiffness_Cartesian
from hopping_leg.controllers.PD_CJS.ctrl   import Controller_Stiffness_Combined

from hopping_leg.state_estimation.contact  import contact_effort_threshold
from hopping_leg.state_estimation.pybullet import contact_pybullet


class StateMachine(AbstractController):
    """
    .. warning:

       This state machine is designed for use directly with the DFKI Underactuated
       Lab ``spine`` library. It is **not** suited for use with the ``stabilization``
       controller scheduling framework.
    
    State machine to achieve backflips with the hopping leg.

    It internally uses a joint, Cartesian or combined joint and Cartesian
    stiffness controller, scheduling its gains to achieve jumping.
    
    While in ``FLIGHT`` phase, and with a frequency of ``backflip_freq`` (so once every
    ``backflip_freq`` jumps), the state machine will add :math:`A*2\pi` to the desired
    hip joint position, where :math:`A` stands for the ``backflip_ampl`` parameter.
    This causes the leg to backflip while in flight.

    .. note:

       **Nomenclature**

       "desired state", "desired", etc.: Cartesian or joint space equilibrium state for a stiffness controller -ergo, desired

    """
    
    flight_counter = 0
    fc_aux         = 0
    
    q_backflip     = np.asarray([0, 0], dtype=float)
    
    def save(self, path):
        shutil.copyfile(self.gains_file, os.path.join(path, 'gains.toml'))
    
    def __init__(self,
                 plant,
                 knee_direction,
                 
                 x_star,
                 y_star,
                 q_star,
                 qd_star,
                 
                 controller,
                 gains_file,
                 initial_phase,
                 backflip_ampl,
                 backflip_freq,
                 
                 contact_est,
                 contact_threshold):
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

        self.controller    = controller
        self.gains_file    = gains_file
        self.initial_phase = initial_phase
        self.backflip_ampl = backflip_ampl
        self.backflip_freq = backflip_freq
        
        self.gains = toml.load(gains_file)
        
        self.x_extended  = 0.7 * (plant.L1 + plant.L2)
        self.x_crouched  = 3/2 * x_star
        
        _controller = {
            'JOINT':     Controller_Stiffness_Joint,
            'CARTESIAN': Controller_Stiffness_Cartesian,
            'COMBINED':  Controller_Stiffness_Combined
        }[controller]
        
        _contact_estimation = {
            'ET': contact_effort_threshold,
            'PB': lambda *args, **kwargs: contact_pybullet(1, 0)
        }[contact_est]
        
        self.state_machine = {
            
            'FLIGHT' :   lambda state: _controller(state,

                                                   **self.gains['FLIGHT'],
                                                   
                                                   # joint
                                                   q_star  = q_star + self.q_backflip,
                                                   qd_star = qd_star,
                                                   T_ff    = [0, 0],
                                                   
                                                   # cartesian
                                                   X_star  = [x_star, y_star],
                                                   Xd_star = [0,     0],
                                                   F_ff    = [0, 0],
                                                   
                                                   plant=plant,
                                                   knee_direction=knee_direction),

            'TOUCHDOWN': lambda state: _controller(state,

                                                   **self.gains['TOUCHDOWN'],
                                                   
                                                   # joint
                                                   q_star  = q_star + self.q_backflip,
                                                   qd_star = qd_star,
                                                   T_ff    = [0, 0],
                                                   
                                                   # cartesian
                                                   X_star  = [x_star, y_star],
                                                   Xd_star = [0,     0],
                                                   F_ff    = [0, 0],
                                                   
                                                   plant=plant,
                                                   knee_direction=knee_direction),

            'EXERTION':  lambda state: _controller(state,

                                                   **self.gains['EXERTION'],
                                                   
                                                   # joint
                                                   q_star  = np.asarray([0, 0]) + self.q_backflip,
                                                   qd_star = qd_star,
                                                   T_ff=[0, 0],
                                                   
                                                   # cartesian
                                                   X_star  = [self.x_extended, y_star],
                                                   Xd_star = [0,           0],
                                                   F_ff    = [0, 0],
                                                   
                                                   plant=plant,
                                                   knee_direction=knee_direction),
            
            'LIFTOFF':   lambda state: _controller(state,

                                                   **self.gains['LIFTOFF'],

                                                   # joint
                                                   q_star  = np.asarray([0, 0]) + self.q_backflip,
                                                   qd_star = qd_star,
                                                   T_ff    = [0, 0],
                                                   
                                                   # cartesian
                                                   X_star  = [x_star, y_star],
                                                   Xd_star = [0,     0],
                                                   F_ff    = [0, 0],
                                                   
                                                   plant=plant,
                                                   knee_direction=knee_direction),

        }

        # LIFTOFF   -> no contact                   -> FLIGHT
        # FLIGHT    -> contact and below x_extended -> TOUCHDOWN
        # TOUCHDOWN -> 0 vertical velocity          -> EXERTION
        # EXERTION  -> leg extended                 -> LIFTOFF

        self.transition = {

            'LIFTOFF'   : lambda state: "FLIGHT"    if not _contact_estimation(state[2, :], contact_threshold) \
                            else "LIFTOFF",

            'FLIGHT'    : lambda state: "TOUCHDOWN" if  _contact_estimation(state[2, :], contact_threshold) \
                                                    and plant.forward_kinematics(*state[0, :])[0] <= self.x_extended - 0.005 \
                            else "FLIGHT",

            'TOUCHDOWN' : lambda state: "EXERTION"  if  np.linalg.norm(plant.forward_velocity(*state[0, :], *state[1, :])) < 0.1 \
                                                    and plant.forward_kinematics(*state[0, :])[0] <= self.x_crouched \
                            else "TOUCHDOWN",

            'EXERTION'  : lambda state: "LIFTOFF"   if plant.forward_kinematics(*state[0, :])[0]  >= self.x_extended \
                            else "EXERTION",

        }
    
    def get_control_output(self, n):
        
        # initial phase
        if n == 0: self.RECORD.phase[0] = self.initial_phase
        # ---------------------------------------------
        
        # state
        state = self.RECORD.state(n)
        
        # phase
        phase = self.transition[self.RECORD.phase[max(n-1, 0)]](state)
        self.RECORD.phase[n] = phase
        
        # flight counter
        trans = phase != self.RECORD.phase[n-1]
        if trans and phase == 'FLIGHT':
            
            self.flight_counter += 1
            self.fc_aux         += 1
        
            # backflip
            if self.fc_aux > self.backflip_freq:
                self.q_backflip  += np.asarray([self.backflip_ampl*2*np.pi, 0])
                if self.controller in ['JOINT', 'COMBINED']:
                    self.final_state += np.asarray([self.backflip_ampl*2*np.pi, 0])
                self.fc_aux       = 0
        
        # control input
        u = np.array([[0, 0],
                      [0, 0],
                      self.state_machine[phase](state)])
        
        return u
