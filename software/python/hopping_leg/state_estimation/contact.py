"""
Contact
=======
"""


def contact_effort_threshold(taus, contact_effort_threshold):
    """
    Contact detection using an effort threshold.

    If any of the torques being exerted by the motors exceed the
    ``contact_effort_threshold``, the leg is assumed to be in contact
    with the ground.

    **Arguments**

    ``taus`` [list or numpy.ndarray]
      array containing the torque applied by each of the motors
      at a given point in time

    ``contact_effort_threshold`` [float]
      torque threshold for any of the motors in the system. If
      any of the torque values in ``taus`` exceeds this threshold,
      the system is considered to be in contact with the ground
    """
    
    return any([abs(tau) > contact_effort_threshold for tau in taus])
