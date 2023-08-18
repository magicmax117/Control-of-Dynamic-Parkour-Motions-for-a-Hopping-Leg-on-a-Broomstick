"""
Energies
========
"""


def energy_kinetic(plant,
                   record,
                   n):
    """
    Vertical kinetic energy of the hopping leg at timestep ``n``.
    
    Assumptions:
    
    * All mass is at the hip
    
    **Arguments**
    
    ``plant`` [``hopping_leg.plant.plant.HopperPlant`` instance]
      hopping leg plant instance
    
    ``record`` [``spine.data.record.record`` instance]
      experiment ``record`` object, containing the time vector of the
      experiment as well as the desired and measured state vectors
      of all motors in the experiment.
    
    ``n`` [int]
      timestep at which the leg's vertical kinetic energy is calculated
    """

    state = record.state(n)
    
    xd_b = plant.forward_velocity(*state[0, :], *state[1, :])[0]
    
    return 1/2 * plant.m * xd_b**2


def energy_potential(plant,
                     record,
                     n):
    """
    Gravitational potential energy of the hopping leg at timestep ``n``.
    
    Assumptions:
    
    * All mass is at the hip
    
    **Arguments**
    
    ``plant`` [``hopping_leg.plant.plant.HopperPlant`` instance]
      hopping leg plant instance
    
    ``record`` [``spine.data.record.record`` instance]
      experiment ``record`` object, containing the time vector of the
      experiment as well as the desired and measured state vectors
      of all motors in the experiment.
    
    ``n`` [int]
      timestep at which the leg's potential energy is calculated
    """
    
    state = record.state(n)
    
    x_b = plant.forward_kinematics(*state[0, :])[0]
    
    return - plant.m * plant.g * x_b
