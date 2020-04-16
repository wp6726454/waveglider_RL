import numpy as np


# vector of current velocity
def Vc(dir_, speed, eta):  # current direction, current speed, vessel's position and attitude vector
    phi = eta.item(3)  # vessel's heading of vessel
    vc_x = speed * np.cos(dir_-phi)
    vc_y = speed * np.sin(dir_-phi)
    return np.array([[vc_x], [vc_y], [0], [0]], float)
