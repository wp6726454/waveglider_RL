import numpy as np
import time
import math
from math import *
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from Environment.WG_dynamics import WG_dynamics
from Environment.Model.Foil import Foil
from Environment.Model.Tether import Tether
from Environment.Model.Rudder import Rudder
from Environment.data_viewer import data_viewer
from Environment.data_process import data_storage, data_elimation

class Waveglider(object):
    # initialization of data storage lists
    def __init__(self):
        self.action_space = ['left', 'left_s', 'hold', 'right_s','right']
        self.n_actions = len(self.action_space)
        self.n_features = 3
        self._t = []
        self.time_step = 0.001
        # sea state
        self.H = 0.3
        self.omega = 1
        self.c_dir = 0
        self.c_speed = 0
        self.state_0 = np.zeros((16, 1))
        self.WG = WG_dynamics(self.H, self.omega, self.c_dir, self.c_speed)

        # float
        self.x1 = []
        self.y1 = []
        self.z1 = []
        self.phi1 = []
        self.u1 = []
        self.v1 = []
        self.w1 = []
        self.r1 = []
        # glider
        self.x2 = []
        self.y2 = []
        self.z2 = []
        self.phit = []
        self.u2 = []
        self.v2 = []
        self.w2 = []
        self.r2 = []
        # forces
        self.T = []
        self.Ffoil_x = []
        self.Ffoil_z = []
        self.Rudder_angle = []
        self.Frudder_x = []
        self.Frudder_y = []
        self.Frudder_n = []
        #target position
        self.target_position = np.array([100, 100])

    def reset(self):
        time.sleep(0.1)
        data_elimation()  # Turn on when previous data needs to be cleared
        self.t = 0
        self._t.clear()
        # float
        self.x1.clear()
        self.y1.clear()
        self.z1.clear()
        self.phi1.clear()
        self.u1.clear()
        self.v1.clear()
        self.w1.clear()
        self.r1.clear()
        # glider
        self.x2.clear()
        self.y2.clear()
        self.z2.clear()
        self.phit.clear()
        self.u2.clear()
        self.v2.clear()
        self.w2.clear()
        self.r2.clear()
        # forces
        self.T.clear()
        self.Ffoil_x.clear()
        self.Ffoil_z.clear()
        self.Rudder_angle.clear()
        self.Frudder_x.clear()
        self.Frudder_y.clear()
        self.Frudder_n.clear()
        # initial state
        self.state_0 = np.array([[0], [0], [0], [0],  # eta1
                            [0], [0], [0], [0],  # V1
                            [0], [0], [6.2], [0],  # eta2
                            [0], [0], [0], [0]], float)  # V2
        #self.rudder_angle = [0]

        return np.array([self.state_0.item(8), self.state_0.item(9), self.state_0.item(11)])


    def obser(self, rudder_angle):

        for _ in range(0, 1000, 1):
            # Runge-Kutta
            k1 = self.WG.f(self.state_0, rudder_angle, self.t)* self.time_step
            k2 = self.WG.f(self.state_0 + 0.5 * k1, rudder_angle, self.t + 0.5 * self.time_step)* self.time_step
            k3 = self.WG.f(self.state_0 + 0.5 * k2, rudder_angle, self.t + 0.5 * self.time_step)* self.time_step
            k4 = self.WG.f(self.state_0 + k3, rudder_angle, self.t + self.time_step)* self.time_step
            self.state_0 += (1 / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            #print(self.state_0.item(3))
            self.t += 0.001
        print(self.state_0.item(11))
        self._t.append(round(self.t, 1))
        self.x1.append(round(self.state_0.item(0),1))
        self.y1.append(round(self.state_0.item(1),1))
        self.z1.append(round(self.state_0.item(2),1))
        self.phi1.append(round(self.state_0.item(3),1))
        self.u1.append(round(self.state_0.item(4),1))
        self.v1.append(round(self.state_0.item(5),1))
        self.w1.append(round(self.state_0.item(6),1))
        self.r1.append(round(self.state_0.item(7),1))
        self.x2.append(round(self.state_0.item(8),1))
        self.y2.append(round(self.state_0.item(9),1))
        self.z2.append(round(self.state_0.item(10),1))
        self.phit.append(round(self.state_0.item(11),1))
        self.u2.append(round(self.state_0.item(12),1))
        self.v2.append(round(self.state_0.item(13),1))
        self.w2.append(round(self.state_0.item(14),1))
        self.r2.append(round(self.state_0.item(15),1))

        self.T.append(round(Tether(self.state_0[0:4], self.state_0[8:12]).T(),1))
        self.Ffoil_x.append(round(Foil(self.state_0[8:12], self.state_0[12:16], self.c_dir, self.c_speed).foilforce().item(0),1))
        self.Ffoil_z.append(round(Foil(self.state_0[8:12], self.state_0[12:16], self.c_dir, self.c_speed).foilforce().item(2),1))

        self.Rudder_angle.append(round(rudder_angle,1))
        self.Frudder_x.append(round(Rudder(self.state_0[8:12], self.state_0[12:16], self.c_dir, self.c_speed).force(rudder_angle).item(0),1))
        self.Frudder_y.append(round(Rudder(self.state_0[8:12], self.state_0[12:16], self.c_dir, self.c_speed).force(rudder_angle).item(1),1))
        self.Frudder_n.append(round(Rudder(self.state_0[8:12], self.state_0[12:16], self.c_dir, self.c_speed).force(rudder_angle).item(3),1))
        data_storage(self.x1, self.y1, self.phit, self.t, self.Rudder_angle)  # store data in local files

        observation = np.array([self.state_0.item(8), self.state_0.item(9), self.state_0.item(11)])

        return observation

    def step(self, action):
        s_ = np.array([0,0,0])
        a_1 = -5*pi/180
        a_2 = -2.5 * pi / 180
        a_3 = 0
        a_4 = 2.5 * pi / 180
        a_5 = 5 * pi / 180

        if action == 0:
            s_ = self.obser(a_1)
        elif action == 1:
            s_ = self.obser(a_2)
        elif action == 2:
            s_ = self.obser(a_3)
        elif action == 3:
            s_ = self.obser(a_4)
        elif action == 4:
            s_ = self.obser(a_5)

        # reward function
        real_position = s_[:2]
        distance_1 = self.target_position - real_position
        distance = math.hypot(distance_1[0], distance_1[1])

        if distance < 10:
            reward = 100
            done = True
        elif (s_[0] >= 110 or s_[0] <= -10) or (s_[1] >= 110 or s_[1] <= -10):
            reward = -100
            done = True
        elif self.t >= 1000:
            reward = -1
            done = True
        else:
            reward = -1
            done = False

        return s_, reward, done

    def render(self):

        data_viewer(self.x1, self.y1, u1=self.u1, phit=self.phit, rudder_angle=self.Rudder_angle, t=self._t, xlim_left=-100, xlim_right=200, ylim_left=-100, ylim_right=200,
                        goal_x=100, goal_y=100)

