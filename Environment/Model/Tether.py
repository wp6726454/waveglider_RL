from math import *
import numpy as np


class Tether (object):
    def __init__(self, eta1, eta2):
        self.x = eta1.item(0) - eta2.item(0)
        self.y = eta1.item(1) - eta2.item(1)
        self.z = eta1.item(2) - eta2.item(2)
        self.phi1 = eta1.item(3)
        self.phi2 = eta2.item(3)

    # tether force
    def T(self):
        k = 1.7e6
        distance = sqrt(self.x**2+self.y**2+self.z**2) - 6.2
        if distance > 0.01:
            distance = 0.01
        F = k * distance + 200

        return F

    def alpha(self):
        alpha = 0
        if self.x > 0:
            alpha = atan(self.y / self.x)
        elif self.x < 0:
            alpha = atan(self.y / self.x) + pi
        elif self.x == 0:
            if self.y > 0:
                alpha = pi/2
            elif self.y == 0:
                alpha = 0
            else:
                alpha = -pi/2
        return alpha

    def beta(self):
        m = (self.x**2 + self.y**2)/(self.x**2+self.y**2+self.z**2)
        return asin(sqrt(m))

    # tether force exerted on float
    def Ftether_1(self):
        x = -self.T() * sin(self.beta()) * cos(self.alpha() - self.phi1)
        y = -self.T() * sin(self.beta()) * sin(self.alpha() - self.phi1)
        z = self.T() * cos(self.beta())
        return np.array([[x], [y], [z], [0]], float)

    # tether force exerted on glider
    def Ftether_2(self):
        x = self.T() * sin(self.beta()) * cos(self.alpha() - self.phi2)
        y = self.T() * sin(self.beta()) * sin(self.alpha() - self.phi2)
        z = - self.T() * cos(self.beta())
        return np.array([[x], [y], [z], [0]], float)
