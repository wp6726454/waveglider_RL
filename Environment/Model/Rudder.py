】from math import *
from Model.Vc import Vc
import numpy as np
from Model.PID import PID

class Rudder():
    def __init__(self, eta2, V2, c_dir, c_speed):
        self.eta2 = eta2
        self.V2 = V2
        self.c_dir = c_dir
        self.c_speed = c_speed

    def force(self, angle):
        # states
        V2_r = self.V2 - Vc(self.c_dir, self.c_speed, self.eta2)
        ur_2 = V2_r.item(0)
        # parameters
        LCG = 0.94
        lamda = 2
        chi = 7 * pi / 180
        A = 0.04
        CDC = 0.8
        CD0 = 0.008
        rho = 1025
        # lift and drag forces
        m = sqrt(4 + lamda ** 2 / cos(chi) ** 4)
        CL = 1.8 * pi * lamda * angle / (1.8 + cos(chi) * m) + CDC * angle ** 2 / lamda
        CD = CD0 + CL ** 2 / 0.9 / lamda / pi
        FL = 0.5 * rho * CL * A * ur_2 ** 2
        FD = 0.5 * rho * CD * A * ur_2 ** 2
        return np.array([[-FD], [FL], [0], [- FL * LCG]])
