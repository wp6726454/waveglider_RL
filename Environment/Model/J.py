import numpy as np


# transmission matrix
def J(eta):  # vessel's position and attitude vector
    phi = eta.item(3)  # vessel's heading of vessel
    c = np.cos(phi)
    s = np.sin(phi)
    trans_matrix = np.array([[c, -s, 0, 0],
                  [s, c, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]], float)
    return trans_matrix
