# global imports
import math
import numpy as np


def yb_SE3_matrix(g, L1, q_vec=None, dq_vec=None, ddq_vec=None):
    s1_yb = g*math.sin(q_vec[0] + q_vec[1])                                    # sigma1 in the regressor matrix

    y_11 = 0    # m1
    y_12 = -g*math.sin(q_vec[0])    # m1*cx1
    y_13 = 0    # m1*cy1
    y_14 = 0    # m1*cz1
    y_15 = 0    # I1xx
    y_16 = 0    # I1yy
    y_17 = ddq_vec[0]    # I1zz
    y_18 = 0    # I1xy
    y_19 = 0    # I1yz
    y_110 = 0   # I1zx

    y_111 = ddq_vec[0]*L1**2 - g*math.sin(q_vec[0])*L1      # m2
    y_112 = -L1*math.sin(q_vec[1])*dq_vec[1]**2 \
           - 2*L1*dq_vec[0]*math.sin(q_vec[1])*dq_vec[1] - s1_yb \
           + 2*L1*ddq_vec[0]*math.cos(q_vec[1]) \
           + L1*ddq_vec[1]*math.cos(q_vec[1])    # m2*cx2
    y_113 = 0    # m2*cy2
    y_114 = 0    # m2*cz2
    y_115 = 0    # I2xx
    y_116 = 0    # I2yy
    y_117 = ddq_vec[0] + ddq_vec[1]    # I2zz
    y_118 = 0    # I2xy
    y_119 = 0    # I2yz
    y_120 = 0   # I2zx

    y_21 = 0    # m1
    y_22 = 0    # m1*cx1
    y_23 = 0    # m1*cy1
    y_24 = 0    # m1*cz1
    y_25 = 0    # I1xx
    y_26 = 0    # I1yy
    y_27 = 0    # I1zz
    y_28 = 0    # I1xy
    y_29 = 0    # I1yz
    y_210 = 0   # I1zx

    y_211 = 0    # m2
    y_212 = L1*math.sin(q_vec[1])*dq_vec[0]**2 - s1_yb \
           + L1*ddq_vec[0]*math.cos(q_vec[1])    # m2*cx2
    y_213 = 0    # m2*cy2
    y_214 = 0    # m2*cz2
    y_215 = 0    # I2xx
    y_216 = 0    # I2yy
    y_217 = ddq_vec[0] + ddq_vec[1]    # I2zz
    y_218 = 0    # I2xy
    y_219 = 0    # I2yz
    y_220 = 0   # I2zx

    yb = np.array([[y_11, y_12, y_13, y_14, y_15, y_16, y_17, y_18, y_19,
                    y_110, y_111, y_112, y_113, y_114, y_115, y_116, y_117,
                    y_118, y_119, y_120],
                  [y_21, y_22, y_23, y_24, y_25, y_26, y_27, y_28, y_29,
                    y_210, y_211, y_212, y_213, y_214, y_215, y_216, y_217,
                    y_218, y_219, y_220]])
    return yb


def yb_matrix(g, n1, n2, L1, q_vec=None, dq_vec=None, ddq_vec=None):
    s1_yb = g*math.sin(q_vec[0] + q_vec[1])                                    # sigma1 in the regressor matrix

    y_11 = -g*math.sin(q_vec[0])
    y_12 = ddq_vec[0]
    y_13 = math.atan(50*dq_vec[0])
    y_14 = dq_vec[0]
    y_15 = ddq_vec[0]*(n1**2+1)+ddq_vec[1]*n2
    y_16 = -L1*math.sin(q_vec[1])*dq_vec[1]**2 \
           - 2*L1*dq_vec[0]*math.sin(q_vec[1])*dq_vec[1] - s1_yb \
           + 2*L1*ddq_vec[0]*math.cos(q_vec[1]) \
           + L1*ddq_vec[1]*math.cos(q_vec[1])
    y_17 = ddq_vec[0]*L1**2 - g*math.sin(q_vec[0])*L1
    y_18 = ddq_vec[0] + ddq_vec[1]
    y_19 = 0
    y_110 = 0

    y_21 = 0
    y_22 = 0
    y_23 = 0
    y_24 = 0
    y_25 = ddq_vec[0]*n1 + ddq_vec[1]*n2**2
    y_26 = L1*math.sin(q_vec[1])*dq_vec[0]**2 - s1_yb \
           + L1*ddq_vec[0]*math.cos(q_vec[1])
    y_27 = 0
    y_28 = ddq_vec[0] + ddq_vec[1]
    y_29 = math.atan(50*dq_vec[1])
    y_210 = dq_vec[1]

    yb = np.array([[y_11, y_12, y_13, y_14, y_15, y_16, y_17, y_18, y_19, y_110],
                  [y_21, y_22, y_23, y_24, y_25, y_26, y_27, y_28,  y_29,  y_210]])
    return yb


def yb_link_matrix(g, L1, q_vec=None, dq_vec=None, ddq_vec=None):
    s1_yb = g*math.sin(q_vec[0] + q_vec[1])                                    # sigma1 in the regressor matrix

    y_11 = -g*math.sin(q_vec[0])
    y_12 = ddq_vec[0]

    y_13 = -L1*math.sin(q_vec[1])*dq_vec[1]**2 \
           - 2*L1*dq_vec[0]*math.sin(q_vec[1])*dq_vec[1] - s1_yb \
           + 2*L1*ddq_vec[0]*math.cos(q_vec[1]) \
           + L1*ddq_vec[1]*math.cos(q_vec[1])
    y_14 = ddq_vec[0]*L1**2 - g*math.sin(q_vec[0])*L1
    y_15 = ddq_vec[0] + ddq_vec[1]

    y_21 = 0
    y_22 = 0
    y_23 = L1*math.sin(q_vec[1])*dq_vec[0]**2 - s1_yb \
           + L1*ddq_vec[0]*math.cos(q_vec[1])
    y_24 = 0
    y_25 = ddq_vec[0] + ddq_vec[1]

    yb = np.array([[y_11, y_12, y_13, y_14, y_15],
                  [y_21, y_22, y_23, y_24, y_25]])
    return yb


def yb_friction_matrix(dq_vec):

    y_11 = math.atan(50*dq_vec[0])
    y_12 = dq_vec[0]

    y_23 = math.atan(50*dq_vec[1])
    y_24 = dq_vec[1]

    yb_fric = np.array([[y_11, y_12, 0, 0],
                        [0, 0, y_23, y_24]])
    return yb_fric


def yb_rotor_matrix(ddq_vec, n1, n2):

    y_11 = ddq_vec[0]*(n1**2+1)+ddq_vec[1]*n2

    y_22 = ddq_vec[0]*n1 + ddq_vec[1]*n2**2

    yb_rotor = np.array([[y_11],
                        [y_22]])
    return yb_rotor

