"""
``pybullet``
============
"""

import time

import numpy as np
import pybullet as pb


def propagate(client, t):
    """
    Propagate the simulation for an amount of time t

    **Arguments**

    ``client`` [int]
      numerical identifier of the pybullet physics client
      running the current simulation

    ``t`` [float]
      time in seconds for which to propagate the simulation
    """
    
    t0 = time.time()
    dt = pb.getPhysicsEngineParameters(client)['fixedTimeStep']
    
    while time.time() - t0 < t:

        _t = time.time()
        
        pb.stepSimulation(physicsClientId=client)

        while time.time() - _t < dt:
            pass


def legV1(client,
          urdf,
          free_joints=[],
          drop_height=0.5):
    """
    
    .. admonition:: NOT IMPLEMENTED
       :class: danger
    
    Create ``pybullet`` robot from a model of the V1 hopping leg.
    
    **Arguments**

    ``client`` [int]
      numerical identifier of the pybullet physics client
      running the current simulation
    
    ``urdf`` [str]
      path to the URDF model of the system, relative to the directory from
      which Python is being run

    ``free_joints`` [list of int]
      list containing the indices of the joints which will be free
      to move
    
    ``drop_height`` [float]
      initial height at which the model will be spawned

    **Output**

    ``robot`` [int]
      numerical identifier of robot object created
      by ``pybullet.loadURDF`` from the provided ``urdf``
    """
    
    # camera position
    pb.resetDebugVisualizerCamera(cameraDistance=1.0,
                                  cameraYaw=180,
                                  cameraPitch=0,
                                  cameraTargetPosition=[0, 0, 0.25],
                                  physicsClientId=client)
    
    robot = pb.loadURDF(urdf,
                        [0, 0, drop_height],
                        useFixedBase=0,
                        baseOrientation=pb.getQuaternionFromEuler([0, 0, 0]),
                        physicsClientId=client)
    
    # free joints
    for joint in free_joints:
        
        _joint = pb.getJointInfo(robot,
                                 joint,
                                 physicsClientId=client)
        
        pb.setJointMotorControl2(robot,
                                 joint,
                                 pb.VELOCITY_CONTROL,
                                 force=0,
                                 physicsClientId=client)
    
    # prismatic constraint
    pb.createConstraint(robot,
                        -1,
                        -1,
                        -1,
                        pb.JOINT_PRISMATIC,
                        [0, 0, 1],
                        [0, 0, 0],
                        [0, 0, 0],
                        pb.getQuaternionFromEuler([0, 0, 0]),
                        physicsClientId=client)
    
    # let the model drop to the ground
    propagate(client, 2)

    return robot


def legV1_rail(client,
               urdf,
               free_joints=[],
               drop_height=0.5):
    """
    Create ``pybullet`` robot from a model of the V1 hopping leg on the
    vertical test stand.

    **Arguments**

    ``client`` [int]
      numerical identifier of the pybullet physics client
      running the current simulation
    
    ``urdf`` [str]
      path to the URDF model of the system, relative to the directory from
      which Python is being run

    ``free_joints`` [list of int]
      list containing the indices of the joints which will be free
      to move

    ``drop_height`` [float]
      initial height at which the model will be spawned

    **Output**

    ``robot`` [int]
      numerical identifier of robot object created
      by ``pybullet.loadURDF`` from the provided ``urdf``
    """

    robot = pb.loadURDF(urdf,
                        [0, 0, 0],
                        useFixedBase=1,
                        physicsClientId=client)

    # free joints
    for joint in free_joints:
        
        _joint = pb.getJointInfo(robot,
                                 joint,
                                 physicsClientId=client)
        
        pb.setJointMotorControl2(robot,
                                 joint,
                                 pb.VELOCITY_CONTROL,
                                 force=0,
                                 physicsClientId=client)
    
    # set initial position of prismatic joint
    pb.resetJointState(robot,
                       jointIndex=0,
                       targetValue=drop_height,
                       physicsClientId=client)

    # let the model drop to the ground
    propagate(client, 2)

    return robot


def legV2(client,
          urdf,
          free_joints=[],
          drop_height=0.5):
    """
    Create ``pybullet`` robot from a model of the V1 hopping leg.

    **Arguments**

    ``client`` [int]
      numerical identifier of the pybullet physics client
      running the current simulation

    ``urdf`` [str]
      path to the URDF model of the system, relative to the directory from
      which Python is being run

    ``free_joints`` [list of int]
      list containing the indices of the joints which will be free
      to move

    ``drop_height`` [float]
      initial height at which the model will be spawned

    **Output**

    ``robot`` [int]
      numerical identifier of robot object created
      by ``pybullet.loadURDF`` from the provided ``urdf``
    """

    # camera position
    pb.resetDebugVisualizerCamera(cameraDistance=1.00,
                                  cameraYaw=-90,
                                  cameraPitch=0,
                                  cameraTargetPosition=[0, 0, 0.25],
                                  physicsClientId=client)

    robot = pb.loadURDF(urdf,
                        [0, 0, drop_height],
                        useFixedBase=0,
                        baseOrientation=pb.getQuaternionFromEuler([np.pi, 0, 0]),
                        physicsClientId=client)

    # free joints
    for joint in free_joints:
        
        _joint = pb.getJointInfo(robot,
                                 joint,
                                 physicsClientId=client)
        
        pb.setJointMotorControl2(robot,
                                 joint,
                                 pb.VELOCITY_CONTROL,
                                 force=0,
                                 physicsClientId=client)

    # prismatic constraint
    pb.createConstraint(robot,
                        -1,
                        -1,
                        -1,
                        pb.JOINT_PRISMATIC,
                        [0, 0, 1],
                        [0, 0, 0],
                        [0, 0, 0],
                        pb.getQuaternionFromEuler([np.pi, 0, 0]),
                        physicsClientId=client)
    
    # free prismatic joint
    pb.setJointMotorControl2(robot,
                             0,
                             pb.VELOCITY_CONTROL,
                             force=0,
                             physicsClientId=client)
    
    # let the model drop to the ground
    propagate(client, 2)
    
    return robot


def legV2_boomstick(client,
                    urdf,
                    free_joints=[],
                    drop_height=0):
    """

    .. admonition:: NOT IMPLEMENTED
       :class: danger
    
    Create ``pybullet`` robot from a model of the V2 hopping leg attached to
    the boomstick.

    **Arguments**

    ``client`` [int]
      numerical identifier of the pybullet physics client
      running the current simulation
    
    ``urdf`` [str]
      path to the URDF model of the system, relative to the directory from
      which Python is being run

    ``free_joints`` [list of int]
      list containing the indices of the joints which will be free
      to move

    ``drop_height`` [float]
      initial height at which the model will be spawned

    **Output**

    ``robot`` [int]
      numerical identifier of robot object created
      by ``pybullet.loadURDF`` from the provided ``urdf``
    """

    # camera position
    pb.resetDebugVisualizerCamera(cameraDistance=1.4,
                                  cameraYaw=27,
                                  cameraPitch=-25,
                                  cameraTargetPosition=[0.24999999999999997, -0.43301270189221935, 0],
                                  physicsClientId=client)

    robot = pb.loadURDF(urdf,
                        [0, 0, drop_height],
                        useFixedBase=0,
                        baseOrientation=pb.getQuaternionFromEuler([0, -np.pi/2, 0]),
                        physicsClientId=client)

    box1 = pb.loadURDF('../../model/obstacles/urdf/obstacle_1.urdf',
                      [0.24810195909806773, -1.143392066568463, 0.15],
                      pb.getQuaternionFromEuler([0, 0, 0.21367521367521372]))
    pb.changeDynamics(box1,
                      -1,
                      lateralFriction=1,
                      physicsClientId=client)

    box2 = pb.loadURDF('../../model/obstacles/urdf/obstacle_2.urdf',
                      [0.7771774735307185, -0.8745828575020259, 0.075],
                      pb.getQuaternionFromEuler([0, 0, 0.7264957264957266]))
    pb.changeDynamics(box2,
                      -1,
                      lateralFriction=2,
                      physicsClientId=client)

    box3 = pb.loadURDF('../../model/obstacles/urdf/obstacle_3.urdf',
                      [1.0004047203630166, -0.6067045372134565, 0.2],
                      pb.getQuaternionFromEuler([0, 0, 1.0256410256410258]))
    pb.changeDynamics(box3,
                      -1,
                      lateralFriction=1,
                      physicsClientId=client)

    marker0 = pb.loadURDF('../../model/obstacles/urdf/marker.urdf',
                       [0, -1.17, 0],
                       pb.getQuaternionFromEuler([0, 0, 0]))

    marker20 = pb.loadURDF('../../model/obstacles/urdf/marker.urdf',
                       [0.1990274040090956, -1.1529475670876799, 0],
                       pb.getQuaternionFromEuler([0, 0, 0.17094017094017097]))

    marker40 = pb.loadURDF('../../model/obstacles/urdf/marker.urdf',
                       [0.39225326707019403, -1.1022873375271798, 0],
                       pb.getQuaternionFromEuler([0, 0, 0.34188034188034194]))

    marker60 = pb.loadURDF('../../model/obstacles/urdf/marker.urdf',
                       [0.5740451600093218, -1.0194960295508129, 0],
                       pb.getQuaternionFromEuler([0, 0, 0.5128205128205129]))

    marker80 = pb.loadURDF('../../model/obstacles/urdf/marker.urdf',
                       [0.7391039476840034, -0.9069869649106991, 0],
                       pb.getQuaternionFromEuler([0, 0, 0.6837606837606839]))

    marker100 = pb.loadURDF('../../model/obstacles/urdf/marker.urdf',
                       [0.8826182559003733, -0.7680397218577846, 0],
                       pb.getQuaternionFromEuler([0, 0, 0.8547008547008549]))

    # free joints
    for joint in free_joints:
        
        _joint = pb.getJointInfo(robot,
                                 joint,
                                 physicsClientId=client)
        
        pb.setJointMotorControl2(robot,
                                 joint,
                                 pb.VELOCITY_CONTROL,
                                 force=0,
                                 physicsClientId=client)

    # # prismatic constraint
    # pb.createConstraint(robot,
    #                     -1,
    #                     -1,
    #                     -1,
    #                     pb.JOINT_PRISMATIC,
    #                     [0, 0, 1],
    #                     [0, 0, 0],
    #                     [0, 0, 0],
    #                     pb.getQuaternionFromEuler([np.pi, 0, 0]),
    #                     physicsClientId=client)

    # # free prismatic joint
    # pb.setJointMotorControl2(robot,
    #                          0,
    #                          pb.VELOCITY_CONTROL,
    #                          force=0,
    #                          physicsClientId=client)
    
    # let the model drop to the ground
    propagate(client, 2)

    # pb.setRealTimeSimulation(0)
    # pb.startStateLogging(pb.STATE_LOGGING_VIDEO_MP4, '/home/dfki.uni-bremen.de/malbracht/videos/nonopt_parcour_simulation_v5.mp4')

    return robot
