"""
Statistics
==========
"""

import numpy as np


def moving_average(x, window):
    """
    Calculate a moving average of signal ``x`` with a kernel of
    ``window`` steps.

    **Arguments**

    ``x`` [numpy.ndarray]
      signal

    ``window`` [float]
      moving average kernel
    """
    cumsum = np.cumsum(np.insert(x, 0, 0))
    return (cumsum[window:] - cumsum[:-window]) / float(window)
