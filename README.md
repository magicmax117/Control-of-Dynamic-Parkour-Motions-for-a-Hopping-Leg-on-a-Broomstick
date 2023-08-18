# Hopping Leg

For practical reference, refer to the experiment main scripts in the `software/python/experiments` directory.

If you are the new maintainer of the project (starting after September 2022),
please check the [introduction to the project](https://git.hb.dfki.de/underactuated-robotics/hopping_leg/-/blob/master/docs/introduction/INTRODUCTION.md)! It's a better document to help you get started.

### IMPORTANT

The `main` branch of this repository contains **mature** software alone,
its documentation (`docs`), and the models of the different hopping legs
in the lab (`models`).

Currently only Python software is present in this repository. It can be found
in the `software/python/hopping_leg` directory, which is a Python package containing
the following modules:

- `controllers`

  Controllers for the system

- `state_estimation`

  State estimation algorithms

- `plant`

  Hopping leg plant

- `simulation`

  Per simulator, functions to load a hopping leg URDF into the simulator, as well as perform
  any necessary setup of the robot model

- `analysis`

  Data analysis and representation functions

- `utils`

  General utilities

-----

#### Table of Contents

[**Install**](#install)

[**Conventions**](#commit-name-conventions)

[**Parameters**](#parameters)

-----

## Install

**IMPORTANT:** this guide holds as of July 2022. In the near future, the [DFKI Underactuated Group Control
Loop library](https://git.hb.dfki.de/underactuated-robotics/control_loop) will **not** include

- motor driver libraries
- nor simulator packages

in its installation dependencies. This is to avoid causing potential installation failures, as some of these
packages are prone to buggy install processes, as well as avoid forcing large libraries on users who may not
have interest in them (eg: installing mjbots drivers for projects which do not use mjbots motors).

If this guide were not updated after the change takes place, and you find yourself missing motor driver or
simulator packages, proceed to install the packages you require and notify the maintainer of this project
and the control loop library.

### High level guide

1. Clone the `master` branch of this repository

   ```
   git clone -b <branch> --single-branch <repo>
   ```

2. Install the project's dependencies

   **pip**
   
   ```
   cd hopping_leg/software/python
   python3 -m pip install .
   ```

   **poetry**
   ```
   cd hopping_leg/software/python
   poetry install
   ```

3. Install the [DFKI Underactuated Group Control Loop library](https://git.hb.dfki.de/underactuated-robotics/control_loop)

4. Install the [DFKI Underactuated Group Stabilization library](https://git.hb.dfki.de/m-rock/stabilization)

### Detailed guide

It can be rather frustrating to be stuck in one of the previous steps without a guiding hand, so here it is.

I will use the regular shell and Poetry for Python package management in this guide. As long as you know how to

- create Python virtual environments
- and install local Python packages

with your tools of choice, the instructions apply with negligible difference.

This guide consists of a **single shell session** and has been tested in Ubuntu 18:04, 20.04 and 22.04.
All commands follow each other as described. Please follow this guide as intended.
Start from scratch if you cannot resolve errors during the process.

#### 1. Create a parent directory

You will download here the **four** repositories you will need. I hereby declare it to be ``repos``.

```
mkdir "repos"
cd repos
```

#### 2. Clone the four repositories you will need

SSH clone below, feel free to clone with HTTPS if you wish.

```

# hopping leg repo
git clone git@git.hb.dfki.de:underactuated-robotics/hopping_leg.git

# control loop library
git@git.hb.dfki.de:underactuated-robotics/control_loop.git

# stabilization library
git clone git@git.hb.dfki.de:m-rock/stabilization.git

# lcm-synced-object (dependency of the stabilization library)
git@git.hb.dfki.de:lmaywald/lcm-synced-object.git

```

Furthermore, install LCM, another dependency of the stabilization library.

```

sudo apt-get install liblcm-bin

```

#### 3. Create a Python virtual environment for the hopping leg project

In Poetry,

```

cd hopping_leg/software/python
poetry install
poetry shell

```

#### 4. Install the control loop and stabilization libraries in the same virtual environment

As follows for the control loop library,

```

cd ../../../control_loop
poetry install

```

and for the stabilization library.

```

cd ../stabilization
poetry install

```

#### 5. Confirm a successful install

This will run whichever state machine controller is currently selected in `experiments/state_machine/experiment.py`.
The `2` means you wish to run the experiment in **pybullet**, `0` and `1` indicating an mjbots or tmotor system
respectively.

```

cd ../hopping_leg/software/python
poetry run python experiments/state_machine/experiment.py 2

```

If the installation is correct, you will see a pybullet simulation window open and a simulation take place of
either the V1 or V2 hopping leg. Afterwards you will see the actuator space state plot, base height plot and task
space end effector plot.

**If you get errors:**

- In the case of "package missing" errors, please install the missing packages using `apt`/`apt-get` and submit
  a pull request to this guide stating which packages you needed to install, along with the operating system
  and version of your system.
- If other errors arise that you cannot resolve,
    1. Remove all installed software and start from scratch.
    2. If the error persists, contact somebody knowledgeable in the lab, and once the cause of the error is
       found please add a note with its cause and solution to this guide. Thank you.

# Commit name conventions

Commit names for commits not relevant enough to deserve a proper commit message. These are commits which you do not want cluttering your commit history, so you can immediately identify meaningful commits when scannig past changes.

If I correct a spelling mistake, I don't want a commit message saying so.

- **r**

  refactoring
  
- **m**

  minor changes
  
- **del**

  deletions
  
- **doc**

  documentation/docstrings


# Parameters

## Hopping leg V2 mjbots: URDF

*(URDF parameters 29.07.2022)*

```
g: -9.81                      # gravity vector
m1: 0.757                     # mass of femur (Link Hip Pitch)
m2: 0.118                     # mass of tibia (Link Knee Pitch) 
I1: 0.0012629                 # moment of inertia (izz)
I2: 0.0000983
L1: 0.15                      # link lengths
L2: 0.15
Lc1: 0.015                    # center of gravity of links
Lc2: 0.045
Fc1: 0.00001                  # coulomb friction
Fc2: 0.00001
Fv1: 0.00001                  # viscous friction
Fv2: 0.00001
Ir1: 0.008923                 # rotor inertia
Ir2: 0.008923
Irr1: 0.0002478               # reflected rotor inertia (Irr = n**2*Ir)
Irr2: 0.0002478
```

**Note:** Differences between Solidworks and URDF export
```
Hip CAD inertia: izz = 2288276.473 g*(mm²)
Hip URDF inertia: izz = 0.0012629  kg*(m²)
Knee CAD inertia: izz = 698351.856  g*(mm²)
Knee URDF inertia: izz = 0.0000983  kg*(m²)
```

## Hopping leg V2 mjbots: Least-Squares

*(Sys id LS 30.07.2022)*

```
m2:  1.250e-01          # mass of tibia
I1:  3.293e-03          # moment of inertia
I2:  1.000e-03 
Lc1: 2.8309115e-02      # center of gravity of links
Lc2: 2.4e-02
Fc1: 4.124e-02          # coulomb friction
Fc2: 2.324e-02 
Fv1: 9.449e-03          # viscous friction
Fv2: 3.962e-02  
Ir:  1.043e-04          # rotor inertia 
```

-----

[Back to the top](#hopping-leg)
