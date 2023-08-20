"""
State Estimation
================
"""

import numpy as np


def state_to_binary(state):
    """
    Construct a binary (0, 1) array indicating FLIGHT or NOT FLIGHT
    from a ``state`` array containing the state of the hopping leg
    at each time step.

    **Arguments**

    ``state`` [numpy.ndarray]
      array containing the state, either "FLIGHT", "TOUCHDOWN" or
      "LIFTOFF" of the hopping leg at each timestep

    **Output**

    ``binary`` [numpy.ndarray]
      array containing 1s for FLIGHT and 0s for NOT FLIGHT
    """

    binary = np.fromiter(({'FLIGHT':    1,
                           'TOUCHDOWN': 0,
                           'LIFTOFF':   0}[s] for s in state), np.int)
    
    return binary


def binary_structure(array):
    """
    Return the structure of a binary array. This function is used to
    find transitions between FLIGHT and NOT FLIGHT in the binary
    state array output by ``state_to_binary`` above.

    From `dan_fulea's answer in StackOverflow <https://stackoverflow.com/a/64267873>`_,
    
       We prepend and also append a zero to A, getting a vector ZA, then detect the 1 islands,
       and the 0 islands coming in alternating manner in the ZA by comparing the shifted versions
       ZA[1:] and ZA[-1]. (In the constructed arrays we take the even places, corresponding to the
       ones in A.)

    where A is the ``array`` argument.
    
    **Arguments**

    ``array`` [numpy.ndarray``
      array containing 1s for FLIGHT and 0s for NOT FLIGHT

    **Output**

    ``indices`` [numpy.ndarray]

    ``counts`` [numpy.ndarray]
    """
    
    ZA      = np.concatenate(([0], array, [0]))
    indices = np.flatnonzero( ZA[1:] != ZA[:-1] )
    counts  = indices[1:] - indices[:-1]
    
    return indices[::2], counts[::2]


def last_liftoff(state):
    """
    Retrieve the index of the last take-off, that is, the most recent
    transition from "LIFTOFF" to "FLIGHT" in the ``state`` array,
    by first creating a binary FLIGHT/NO FLIGHT representation of
    the state array, and then finding the index of the last
    "isle" of 1s (time steps in FLIGHT).
    
    **Arguments**

    ``state`` [numpy.ndarray]
      array containing the state, either "FLIGHT", "TOUCHDOWN" or
      "LIFTOFF" of the hopping leg at each timestep

    **Output**

    ``index`` [numpy.ndarray]
      index of the last take-off to flight transition in the ``state``
      array
    """

    indices, _ = binary_structure(state_to_binary(state))
    
    return indices[-1] if indices.size > 0 else -1
