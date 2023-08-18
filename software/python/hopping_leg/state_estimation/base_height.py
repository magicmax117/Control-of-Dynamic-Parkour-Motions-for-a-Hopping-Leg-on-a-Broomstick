"""
Base Height
===========
"""


import numpy as np
from scipy.signal import find_peaks

# HOPPING LEG
from hopping_leg.utils.state_estimation import last_liftoff


def base_height_velocity_ma(plant,
                            record,
                            n,
                            window=10,
                            delay=0):
    """
    Estimate the vertical velocity of the base using a moving average
    spanning ``window`` timesteps.
    
    **Arguments**
    
    ``plant`` [``hopping_leg.plant.plant.HopperPlant`` instance]
      hopping leg plant instance
    
    ``record`` [``spine.data.record.record`` instance]
      experiment ``record`` object, containing the time vector of the
      experiment as well as the desired and measured state vectors
      of all motors in the experiment.
    
    ``n`` [int]
      iteration index at which the vertical velocity of the base will
      be calculated
    
    ``window`` [int]
      moving average kernel
    
    ``delay`` [int]
      moving average delay
    
    **Output**
    
    ``v`` [float]
      vertical velocity of the base of the hopping leg at timestep ``n``,
      calculated using a moving average with a kernel of ``window`` timesteps
    """
    
    v  = lambda n: plant.forward_velocity(*record.state(n)[0, :], *record.state(n)[1, :])[0]
    
    return np.array([v(i) for i in range(max(n-window, 0), n)]).mean()


def base_height_velocity_max(plant,
                             record,
                             n,
                             window=10,
                             delay=0):
    """
    Estimate the vertical velocity of the base by taking the maximum
    base velocity in a window of WINDOW timesteps.
    
    **Arguments**
    
    ``plant`` [``hopping_leg.plant.plant.HopperPlant`` instance]
      hopping leg plant instance
    
    ``record`` [``spine.data.record.record`` instance]
      experiment ``record`` object, containing the time vector of the
      experiment as well as the desired and measured state vectors
      of all motors in the experiment.
    
    ``n`` [int]
      iteration index at which the vertical velocity of the base will
      be calculated
    
    ``window`` [int]
      moving average kernel
    
    ``delay`` [int]
      moving average delay
    
    **Output**
    
    ``v`` [float]
      vertical velocity of the base of the hopping leg at timestep ``n``,
      calculated using a moving average with a kernel of ``window`` timesteps
    """
    
    v  = lambda n: plant.forward_velocity(*record.state(n)[0, :], *record.state(n)[1, :])[0]
    
    return max([v(i) for i in range(max(n-window, 0), n)])


def base_height_hybrid(plant,
                       record,
                       n,
                       window=10,
                       delay=0,
                       flight_phases=['FLIGHT']):
    """
    Estimate current base height by simple integration during flight
    phase and forward kinematics otherwise.
    
    **Arguments**
    
    ``plant`` [``hopping_leg.plant.plant.HopperPlant`` instance]
      hopping leg plant instance
    
    ``record`` [``spine.data.record.record`` instance]
      experiment ``record`` object, containing the time vector of the
      experiment as well as the desired and measured state vectors
      of all motors in the experiment.
    
    ``n`` [int]
      iteration index at which the vertical velocity of the base will
      be calculated
    
    ``window`` [int]
      moving average window

    ``delay`` [int]
      moving average delay
    
    ``flight_phases`` [list of str]
      list containing the phase names of all phases during which the
      leg is **not in contact with the ground**
    
    **Output**
    
    ``x`` [float]
      base height at time ``t``
    """
    
    if record.phase[n] not in flight_phases:
        
        return plant.forward_kinematics(*record.state(n)[0, :])[0]
    
    else:

        lo_n     = last_liftoff(record.phase[:n+1])
        lo_state = record.state(lo_n)
        
        t0 = record.t[lo_n]
        x0 = plant.forward_kinematics(*lo_state[0, :])[0]
        v0 = base_height_velocity_max(plant  = plant,
                                      record = record,
                                      n      = lo_n,
                                      window = window,
                                      delay  = delay)
        
        t  = record.t[n]
        g  = plant.g
        
        x  = x0 + v0*(t-t0) + g*(t-t0)**2/2
        
        return x


def base_height_peaks(t,
                      h_base,
                      h_peak_min):
    """
    **Arguments**
    
    ``t`` [float]
      time vector

    ``h_base`` [numpy.ndarray]
      array containing the measured or estimated base height of the hopping leg

    ``h_peak_min`` [float]
      minimum height of a peak for it to be identified as such

    **Output**

    ``peaks_t`` [numpy.ndarray]
      array containing the times at which the peaks in the ``base_height`` array are found

    ``peaks_h`` [numpy.ndarray]
      array containing the heights reached at each found peak
    """

    peaks, _ = find_peaks(h_base,
                          height=h_peak_min)
    
    peaks_t = t[peaks]
    peaks_h = h_base[peaks]
    
    return peaks_t, peaks_h


def base_height_peak_last(t,
                          h_base,
                          h_peak_min):
    """
    Retrieve the most recent base peak height. Refer to the ``base_height_peaks`` documentation.
    """
    
    _, peaks_h = base_height_peaks(t,
                                   h_base,
                                   h_peak_min)

    try:
        return peaks_h[-1]
    except IndexError:
        return False
