# global imports
import numpy as np


def xb_vector(m1, m2, I1, I2, Lc1, Lc2, Fc1, Fc2, Fv1, Fv2, Irr1, Irr2):          # initial state vector with reflected rotor inertia as a konstant provided by t-motors
    xb0 = [Lc1 * m1, I1   , Fc1, Fv1, Irr1   , Lc2 * m2,  m2, I2    , Fc2, Fv2, Irr2]
    bounds = ([0.15 , 0.0  , 0.0, 0.0, 0.0   , 0.1    ,  0.5, 0.0   , 0.0, 0.0, 0.0],
              [0.3, np.Inf, 2.0, 2.0, 0.003,  0.4    ,  0.7, np.Inf, 2.0, 2.0, 0.003])
    return xb0, bounds


def xb_vector_ref_rotor(m1, m2, I1, I2, Lc1, Lc2, Fc1, Fc2, Fv1, Fv2, Ir):         # initial state vector with reflected rotor inertia in the regressor matris
    xb0 = [Lc1 * m1,     I1,  Fc1,  Fv1,      Ir, Lc2 * m2,   m2,      I2,  Fc2,  Fv2]
    bounds = ([0.01, 0.0005,  0.0,  0.0, 0.00001,    0.003,  0.1, 0.00005,  0.0,  0.0],
              [ 0.2,   0.01,  0.5,  0.5,   0.005,      0.2, 0.15,   0.001,  0.5,  0.5])
    return xb0, bounds


def xb_vector_id_mjbots():                                                           # latest sys id estimation with mjbots hopping leg v2
    Lc1m1      = 3.430e-02
    I1         = 5.000e-04
    Fc1        = 2.855e-02
    Fv1        = 2.209e-02
    Ir1        = 4.070e-04
    Lc2m2      = 4.000e-03
    m2         = 1.000e-01
    I2         = 1.000e-03
    Fc2        = 4.273e-02
    Fv2        = 3.263e-02
    Ir2        = 4.070e-04
    xb = [Lc1m1, I1, Fc1, Fv1, Ir1, Lc2m2, m2, I2, Fc2, Fv2, Ir2]
    return xb


def xb_vector_cad():                                                                  # estimation from cad data with mjbots hopping leg v2
    Lc1m1      = +0.011355
    I1         = +0.0012629
    Fc1        = +0.00001
    Fv1        = +0.00001
    Ir         = +0.000060719
    n1         =  6     
    n2         =  12
    Lc2m2      = +0.00531
    m2         = +0.118
    I2         = +0.0000983
    Fc2        = +0.00001
    Fv2        = +0.00001
    xb = [Lc1m1, I1, Fc1, Fv1, Ir, n1, n2, Lc2m2, m2, I2, Fc2, Fv2]
    return xb





