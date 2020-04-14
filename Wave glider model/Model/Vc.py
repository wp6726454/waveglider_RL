import numpy as np


def Vc(dir_, speed, eta):
    phi = eta.item(3)
    vc_x = speed * np.cos(dir_-phi)
    vc_y = speed * np.sin(dir_-phi)
    return np.array([[vc_x], [vc_y], [0], [0]], float)