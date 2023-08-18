import yaml


def load_from_yaml(params_filepath):
        """
        Load the pendulum parameters from a yaml file.
        Parameters
        ----------
        params_filepath : string
            path to yaml file
        """
        n1 = 6
        n2 = 6
        with open(params_filepath, 'r') as yaml_file:
            params = yaml.safe_load(yaml_file)
        g = params["g"]               # gravity vector
        m1 = params["m1"]             # mass of links
        m2 = params["m2"]
        I1 = params["I1"]             # moment of inertia
        I2 = params["I2"]
        L1 = params["L1"]             # link lengths
        L2 = params["L2"]
        Lc1 = params["Lc1"]           # center of gravity of links
        Lc2 = params["Lc2"]
        Fc1 = params["Fc1"]           # coulomb friction
        Fv1 = params["Fv1"]           # viscous friction
        Fc2 = params["Fc2"]           
        Fv2 = params["Fv2"]          
        Ir = params["Ir"]             # rotor inertia
        n1 = params["n1"]             # gear ratio
        n2 = params["n2"]             
        Irr1 = params["Irr1"]         # reflected rotor inertia
        Irr2 = params["Irr2"]             
        return g, m1, m2, I1, I2, L1, L2, Lc1, Lc2, Fc1, Fv1, Fc2, Fv2, Ir, n1, n2, Irr1, Irr2
