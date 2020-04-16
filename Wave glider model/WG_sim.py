import numpy as np
from math import *
import matplotlib.pyplot as plt
from WG_dynamics import WG_dynamics
from Model.Foil import Foil
from Model.Tether import Tether
from Model.Rudder import Rudder
from data_viewer import data_viewer
from data_process import data_storage, data_elimation


# initialization of data storage lists
_t = []
# float
x1 = []; y1 = []; z1 = []; phi1 = []
u1 = []; v1 = []; w1 = []; r1 = []
# glider
x2 = []; y2 = []; z2 = []; phit = []
u2 = []; v2 = []; w2 = []; r2 = []
# forces
T = []; Ffoil_x = []; Ffoil_z = []
Rudder_angle = []; Frudder_x = []; Frudder_y = []; Frudder_n = []

def simulation_time(start_time=0,
                    terminal_time=1000,
                    time_step=0.001):
    return np.arange(start_time, terminal_time, time_step)


def simulation():
    # initial state
    state_0 = np.array([[0], [0], [0], [0],  # eta1
                      [0], [0], [0], [0],  # V1
                      [0], [0], [6.2], [0],  # eta2
                      [0], [0], [0], [0]], float)  # V2
    # sea state
    H = 0.5
    omega = 0.25
    c_dir = 0
    c_speed = 0

    WG = WG_dynamics(H, omega, c_dir, c_speed)
    data_elimation()  # Turn on when previous data needs to be cleared

    for t in simulation_time(terminal_time=1000):
        rudder_angle = 0.5 * sin(t)  # data storage
        _t.append(t)
        x1.append(state_0.item(0)); y1.append(state_0.item(1)); z1.append(state_0.item(2)); phi1.append(state_0.item(3))
        u1.append(state_0.item(4)); v1.append(state_0.item(5)); w1.append(state_0.item(6)); r1.append(state_0.item(7))
        x2.append(state_0.item(8)); y2.append(state_0.item(9)); z2.append(state_0.item(10)); phit.append(state_0.item(11))
        u2.append(state_0.item(12)); v2.append(state_0.item(13)); w2.append(state_0.item(14)); r2.append(state_0.item(15))

        T.append(Tether(state_0[0:4], state_0[8:12]).T())
        Ffoil_x.append(Foil(state_0[8:12], state_0[12:16], c_dir, c_speed).foilforce().item(0))
        Ffoil_z.append(Foil(state_0[8:12], state_0[12:16], c_dir, c_speed).foilforce().item(2))

        Rudder_angle.append(rudder_angle)
        Frudder_x.append(Rudder(state_0[8:12], state_0[12:16], c_dir, c_speed).force(rudder_angle).item(0))
        Frudder_y.append(Rudder(state_0[8:12], state_0[12:16], c_dir, c_speed).force(rudder_angle).item(1))
        Frudder_n.append(Rudder(state_0[8:12], state_0[12:16], c_dir, c_speed).force(rudder_angle).item(3))

        data_storage(x1, y1, phit, t, rudder_angle=Rudder_angle)  # store data in local files

        # Runge-Kutta
        time_step = 0.001
        k1 = time_step * WG.f(state_0, rudder_angle, t)
        k2 = time_step * WG.f(state_0 + 0.5 * k1, rudder_angle, t + 0.5 * time_step)
        k3 = time_step * WG.f(state_0 + 0.5 * k2, rudder_angle, t + 0.5 * time_step)
        k4 = time_step * WG.f(state_0 + k3, rudder_angle, t + time_step)
        state_0 += (1 / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
        print(t)
        # print(u1[-1], np.mean(u1))

        # interval of refreshing the monitor, default: 1s
        if t % 1 == 0:
            data_viewer(x1, y1, phit, _t, rudder_angle=Rudder_angle, u1=u1)


    return x1, y1, z1, phi1, \
           x2, y2, z2, phit, \
           u1, v1, w1, r1, \
           u2, v2, w2, r2, \
           _t, T, Ffoil_x, Ffoil_z, \
           Frudder_x, Frudder_y, Frudder_n, Rudder_angle



x1, y1, z1, phi1, \
x2, y2, z2, phit, \
u1, v1, w1, r1, \
u2, v2, w2, r2, \
_t, T, Ffoil_x, Ffoil_z, \
Frudder_x, Frudder_y, Frudder_n, Rudder_angle = simulation()

# plot the result figures after simulation
path = plt.figure(1)
plt.plot(y1, x1, '-r')
plt.title('Path')
plt.ylabel('x(m)')
plt.xlabel('y(m)')

heading = plt.figure(2)
plt.plot(_t, phit, '-r', label = 'phit')
plt.title('Heading')
plt.ylabel('Course(rad)')
plt.ylabel('Time(s)')

rudder_angle = plt.figure(3)
plt.plot(_t, Rudder_angle, '-r')
plt.title('Rudder angle')
plt.ylabel('Rudder angle(deg)')
plt.ylabel('Time(s)')
plt.show()


