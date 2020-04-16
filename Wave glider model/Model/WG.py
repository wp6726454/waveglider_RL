from Model.Vc import Vc
import numpy as np
from scipy import integrate


class WG (object):

    def __init__(self, eta1, eta2, V1, V2, c_dir, c_speed):
        # states
        V1_r = V1 - Vc(c_dir, c_speed, eta1)
        V2_r = V2 - Vc(c_dir, c_speed, eta2)
        self.u1_r = V1_r.item(0)
        self.v1_r = V1_r.item(1)
        self.r1 = V1_r.item(3)
        self.u2_r = V2_r.item(0)
        self.v2_r = V2_r.item(1)
        self.r2 = V2_r.item(3)
        # parameters
        self.rho = 1025
        self.L = np.array([[0, 0, 0, 0],
                      [0, 0, 0, 1],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0]])
    # float's inertia matrix
    def MRB_1(self):
        return np.diag((55, 55, 55, 13.75))

    # glider's inertia matrix
    def MRB_2(self):
        return np.diag((35, 35, 35, 5.6))

    # float's added mass matrix
    def MA_1(self):
        return np.diag((4.59, 50, 490, 15.8))

    # glider's added mass matrix
    def MA_2(self):
        return np.diag((3, 300, 20, 1))

    # Coriolis and centripetal matrix
    def CRB_1(self):
        return self.u1_r * np.dot(self.MRB_1(), self.L)

    def CRB_2(self):
        return self.u2_r * np.dot(self.MRB_2(), self.L)

    def CA_1(self):
        return self.u1_r * np.dot(self.MA_1(), self.L)

    def CA_2(self):
        return self.u2_r * np.dot(self.MA_2(), self.L)

    # float's linear damping matrix
    def D_1(self):
        return np.diag((0, 0, 966.17, 0))

    # integration items for quadratic damping calculation
    def I_y(self, x):
        return (self.v1_r + x * self.r1) * abs(self.v1_r + x * self.r1)

    def I_n(self, x):
        return x * self.I_y(x)

    # float's quadratic damping
    def d_1(self):
        # parameters
        Lpp = 2.1
        Cx = 0.6
        Cy = 0.7
        Ax = 0.09
        Ay = 0.3
        # integral items
        #x = symbols('x')
        #I_y = (self.v1_r + x * self.r1) * abs(self.v1_r + x * self.r1)
        #I_n = x * I_y
        dx = 0.5 * self.rho * Cx * Ax * self.u1_r * abs(self.u1_r)
        v1, err1 = integrate.quad(self.I_y, - 0.5 * Lpp, 0.5 * Lpp)
        v2, err2 = integrate.quad(self.I_n, - 0.5 * Lpp, 0.5 * Lpp)
        dy = 0.5 * self.rho * Cy * Ay / Lpp * v1
        dn = 0.5 * self.rho * Cy * Ay / Lpp * v2
        return np.array([[dx], [dy], [0], [dn]])

    # glider's quadratic damping
    def d_2(self):
        y = 0.5 * self.rho *0.7 * 0.2868 * self.v2_r * abs(self.v2_r)
        return np.array([[0], [y], [0], [0]])

    # float's gravitational matrix
    def G_1(self):
        return np.diag((0, 0, 11030.332, 0))

    # glider's gravitational matrix
    def g_2(self):
        return np.array([[0], [0], [-200], [0]])

