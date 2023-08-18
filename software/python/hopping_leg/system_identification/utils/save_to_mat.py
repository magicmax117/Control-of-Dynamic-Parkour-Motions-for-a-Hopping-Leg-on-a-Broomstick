import numpy as np
from varname.helpers import Wrapper
from scipy.io import savemat


def convert(mat_dict=None):
    if mat_dict is None:
        a = np.arange(20)
        mat_dict = {"a": a, "label": "experiment"}
    label = Wrapper(mat_dict)
    dict_label = str(mat_dict)
    print(label)
    savemat("matlab/" + label.name + ".mat", mat_dict)
    #savemat("/matlab/" + label.name + ".mat", mat_dict)


if __name__ == "__main__":
    convert()
