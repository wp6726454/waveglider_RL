import numpy as np


def J(eta):
    phi = eta.item(3)
    c = np.cos(phi)
    s = np.sin(phi)
    trans_matrix = np.array([[c, -s, 0, 0],
                  [s, c, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]], float)
    return trans_matrix