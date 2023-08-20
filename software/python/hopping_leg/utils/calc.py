"""
Numerical
=========
"""

import numpy as np


def qdd(record, n, i):
    """
    Backward differentiation to obtain angular accelerations

    **Arguments**

    ``record`` [``spine.data.record.record`` instance]
      experiment ``record`` object, containing the time vector of the
      experiment as well as the desired and measured state vectors
      of all motors in the experiment.
    
    ``n`` [int]
      current iteration number
    
    ``i`` [int]
      index in the ``record.motor_names`` list of the motor the
      angular acceleration of which you want to obtain. So usually
      0 for the hip motor, 1 for the knee motor
    """

    t_m1 = record.t[n-1]
    t_0  = record.t[n]

    qd_m1, qd_0 = getattr(record, f'{record.motor_names[i]}_vel_msr')[n-2:n]

    qdd_0 = (qd_0 - qd_m1)/(t_0 - t_m1)

    return qdd_0


def derivative_diff_backward(f, g):
    """
    Calculate the derivative of ``f`` with respect to ``g``.

    **Arguments**

    ``f`` [numpy.ndarray]
      function of ``g``
    
    ``g`` [numpy.ndarray]
      signal (eg: time)
    
    **Output**
    
    ``v`` [float]
      derivative of signal ``f`` with respect to ``g``
    """
    
    v = np.array([(f[i] - f[i-1])/(g[i] - g[i-1]) for i in range(len(f))])
    
    return v
