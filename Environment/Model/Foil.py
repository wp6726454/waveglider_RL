from math import *
from Model.Vc import Vc
import numpy as np


class Foil (object):

    def __init__(self, eta2, V2, c_dir, c_speed):
        self.rho = 1025
        self.A = 0.113
        self.lamda = 2
        V2_r = V2 - Vc(c_dir, c_speed, eta2)
        self.u2_r = V2_r.item(0)
        self.w2_r = V2_r.item(2)
        self.vf = sqrt(self.u2_r**2 + self.w2_r**2)
        self.alpha_f = atan2(abs(self.w2_r), abs(self.u2_r))

    def CL(self):
        chi = 7*pi/180
        alpha_k = 18 * pi/180
        alpha = self.alpha_f - alpha_k
        CDC = 0.6
        m = sqrt(4+self.lamda**2/cos(chi)**4)
        CL = 1.8*pi*self.lamda*alpha/(1.8+cos(chi)*m)+CDC*alpha**2/self.lamda
        return CL

    def CD(self):
        CD0 = 0.008
        return CD0 + self.CL()**2/0.9/self.lamda/pi

    def FL(self):
        return 0.5*self.rho*self.CL()*self.A*self.vf**2

    def FD(self):
        return 0.5*self.rho*self.CD()*self.A*self.vf**2

    def foilforce(self):
        x = self.FL() * sin(self.alpha_f) - self.FD() * cos(self.alpha_f)
        z = - np.sign(self.w2_r) * self.FL() * cos(self.alpha_f) - np.sign(self.w2_r) * self.FD() * sin(self.alpha_f)
        return np.array([[12 * x], [0], [12 * z], [0]])



