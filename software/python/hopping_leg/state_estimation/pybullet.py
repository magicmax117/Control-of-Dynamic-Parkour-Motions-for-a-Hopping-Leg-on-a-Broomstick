"""
``pybullet``
============
"""

import pybullet as pb


def contact_pybullet(robot, ground):
    """
    Determine whether the robot model is in contact with the
    ground plane by retrieving their contact points.

    **Arguments**

    ``robot`` [int]
      numerical identifier of robot body created
      by ``pybullet.loadURDF`` from the provided ``urdf``

    ``ground`` [int]
      numerical identifier of the ground plane body
    
    **Ouput**

    ``contact`` [bool]
      ``True`` if the list of contact points between the robot
      and ground plane is not empty
    """

    n = pb.getNumJoints(robot)
    contact_points = pb.getContactPoints(robot,
                                         ground,
                                         max(n-1, 2),
                                         -1)
    contact_points2 = pb.getContactPoints(robot, 2)
    contact_points3 = pb.getContactPoints(robot, 3)
    contact_points4 = pb.getContactPoints(robot, 4)
    contact = len(contact_points) > 0
    contact2 = len(contact_points2) > 0
    contact3 = len(contact_points3) > 0
    contact4 = len(contact_points4) > 0
    if contact or contact2 or contact3 or contact4:
        contact = True
    return contact


def base_height_pybullet(robot, prismatic_joint):
    """
    Retrieve the base height of the monoped by calling
    ``pybullet.getJointState`` on the vertical prismatic joint

    **Arguments**

    ``robot`` [int]
      numerical identifier of robot body created
      by ``pybullet.loadURDF`` from the provided ``urdf``

    ``prismatic_joint`` [int]
      numerical identifier of the vertical prismatic joint
      constraining the movement of the monoped's hip
    
    **Ouput**

    ``jump_height`` [float]
      monoped jump height measured at the hip
    """
    
    # jump_height = pb.getJointState(robot, prismatic_joint)[0]
    jump_height = pb.getBasePositionAndOrientation(robot)[0][2]
    
    return jump_height
