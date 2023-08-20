"""
Implementation
==============
"""

def Ctrl(x, xd, u, ud, Kp, Kd, F_ff):
    """
    General Proportional Derivative (PD) controller.
    
    **Arguments**

    ``x`` [``n``x``1`` numpy.ndarray of float]
      system state vector

    ``xd`` [``n``x``1`` numpy.ndarray of float]
      system state derivative vector

    ``u`` [``n``x``1`` numpy.ndarray of float]
      desired state vector

    ``ud`` [``n``x``1`` numpy.ndarray of float]
      desired state derivative vector

    ``Kp`` [``n``x``n`` numpy.ndarray of float]
      proportional gain matrix

    ``Kd`` [``n``x``n`` numpy.ndarray of float]
      derivative gain matrix

    ``F_ff`` [``n``x``1`` numpy.ndarray of float]
      feed-forward force array

    **Output**

    ``F`` [``n``x``1`` numpy.ndarray of float]
      control input force array
    """
    
    p = Kp.dot(u  - x)
    d = Kd.dot(ud - xd)
    
    F = p + d + F_ff
    
    return F
