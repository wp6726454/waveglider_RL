import numpy as np
import math
from math import *
import tensorflow as tf
import time
from Environment.Model.J import J
from Environment.Model.Vc import Vc
from Environment.Model.WG import WG
from Environment.Model.Rudder import Rudder
from Environment.data_viewer import data_viewer
from Environment.data_process import data_storage, data_elimation

tf.disable_v2_behavior()
np.random.seed(1)
tf.set_random_seed(1)


class Waveglider(object):
    # initialization of data storage lists
    def __init__(self):
        self.n_features = 5
        self._t = []
        self.time_step = 1
        # sea state
        self.H = 0.3
        self.omega = 1
        self.c_dir = 0
        self.c_speed = 0
        self.state_0 = np.zeros((8, 1))

        # float
        self.x1 = []
        self.y1 = []
        self.z1 = []
        self.phi1 = []
        self.u1 = []
        self.v1 = []
        self.w1 = []
        self.r1 = []
        # forces
        self.Thrust = []
        self.Rudder_angle = []
        self.Frudder_x = []
        self.Frudder_y = []
        self.Frudder_n = []
        #target position
        self.target_position = np.array([100, 100])
        '''
        self.obstacle_1 = np.array([50, 50])
        self.obst_R1 = 10
        self.obstacle_2 = np.array([50, 50])
        self.obst_R2 = 10
        self.obstacle_3 = np.array([50, 50])
        self.obst_R3 = 10
        self.obstacle_4 = np.array([50, 50])
        self.obst_R4 = 10
        self.obstacle_5 = np.array([50, 50])
        self.obst_R5 = 10
        '''
        self.obstacle_1 = np.array([50, 50])
        self.obst_R1 = 12
        self.obstacle_2 = np.array([70, 10])
        self.obst_R2 = 10
        self.obstacle_3 = np.array([90, 50])
        self.obst_R3 = 8
        self.obstacle_4 = np.array([70, 80])
        self.obst_R4 = 8
        self.obstacle_5 = np.array([20, 70])
        self.obst_R5 = 8


        self.xlim_left = -20
        self.xlim_right = 120
        self.ylim_left = -20
        self.ylim_right = 120

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

        # forces
        self.Thrust.clear()

        self.Rudder_angle.clear()
        self.Frudder_x.clear()
        self.Frudder_y.clear()
        self.Frudder_n.clear()
        # initial state
        self.state_0 = np.array([[0], [0], [0], [0],  # eta1
                            [0], [0], [0], [0]],  float)  # V1

        self.distance_target = math.hypot(self.state_0.item(0) - self.target_position[0],self.state_0.item(1) - self.target_position[1]) / 100
        self.distance_obstacle_1 = (math.hypot(self.state_0.item(0) - self.obstacle_1[0],self.state_0.item(1) - self.obstacle_1[1])-self.obst_R1) / 100
        self.distance_obstacle_2 = (math.hypot(self.state_0.item(0) - self.obstacle_2[0],self.state_0.item(1) - self.obstacle_2[1])-self.obst_R2) / 100
        self.distance_obstacle_3 = (math.hypot(self.state_0.item(0) - self.obstacle_3[0],self.state_0.item(1) - self.obstacle_3[1])-self.obst_R3) / 100
        self.distance_obstacle_4 = (math.hypot(self.state_0.item(0) - self.obstacle_4[0],self.state_0.item(1) - self.obstacle_4[1])-self.obst_R4) / 100
        self.distance_obstacle_5 = (math.hypot(self.state_0.item(0) - self.obstacle_5[0],self.state_0.item(1) - self.obstacle_5[1])-self.obst_R5) / 100

        self.course_error_target = self.state_0.item(3) - self.desired_course(self.target_position[0],self.target_position[1],self.state_0.item(0),self.state_0.item(1))
        self.course_error_obstacle_1 = self.state_0.item(3) - self.desired_course(self.obstacle_1[0],self.obstacle_1[1],self.state_0.item(0),self.state_0.item(1))
        self.course_error_obstacle_2 = self.state_0.item(3) - self.desired_course(self.obstacle_2[0],self.obstacle_2[1],self.state_0.item(0),self.state_0.item(1))
        self.course_error_obstacle_3 = self.state_0.item(3) - self.desired_course(self.obstacle_3[0],self.obstacle_3[1],self.state_0.item(0),self.state_0.item(1))
        self.course_error_obstacle_4 = self.state_0.item(3) - self.desired_course(self.obstacle_4[0],self.obstacle_4[1],self.state_0.item(0),self.state_0.item(1))
        self.course_error_obstacle_5 = self.state_0.item(3) - self.desired_course(self.obstacle_5[0],self.obstacle_5[1],self.state_0.item(0),self.state_0.item(1))

        self.distance_obstacle = list([self.distance_obstacle_1, self.distance_obstacle_2, self.distance_obstacle_3, self.distance_obstacle_4,self.distance_obstacle_5])
        self.course_error_obstacle = list([self.course_error_obstacle_1, self.course_error_obstacle_2, self.course_error_obstacle_3,self.course_error_obstacle_4, self.course_error_obstacle_5])
        #
        # # find the nearest obstacle
        i_obstacle = self.distance_obstacle.index(min(self.distance_obstacle))
        observation = np.array([self.distance_target, self.course_error_target, self.distance_obstacle[i_obstacle], self.course_error_obstacle[i_obstacle]])
        if self.n_features == 5:
            observation = np.array([self.distance_target, self.course_error_target, self.distance_obstacle[i_obstacle],
                                    self.course_error_obstacle[i_obstacle], 1])
        return observation

    def desired_course(self,setpoint_x,setpoint_y,realposition_x,realposition_y):

        '''calculate the desired course based on the real-time location and set point'''

        if setpoint_x == realposition_x and setpoint_y > realposition_y:
            phid = pi/2
        elif setpoint_x == realposition_x and setpoint_y < realposition_y:
            phid = -pi/2
        elif setpoint_x > realposition_x and setpoint_y >= realposition_y:
            phid = atan((setpoint_y-realposition_y)/(setpoint_x-realposition_x))
        elif setpoint_x < realposition_x and setpoint_y >= realposition_y:
            phid = atan((setpoint_y-realposition_y)/(setpoint_x-realposition_x)) + pi
        elif setpoint_x < realposition_x and setpoint_y < realposition_y:
            phid = atan((setpoint_y-realposition_y)/(setpoint_x-realposition_x)) - pi
        else:
            phid = atan((setpoint_y-realposition_y)/(setpoint_x-realposition_x))

        return (phid)

    def f(self, state, angle):
        #  float's position and attitude vector
        eta1 = state[0:4]
        #eta1[2] = self.H / 2 * sin(self.omega * t)
        thrust = 20*sin(0.6*self.t) + 20
        WF = np.array([[thrust], [0], [0], [0]])
        #  float's velocity vector
        V1 = state[4:8]

        #  float's relative velocity vector
        V1_r = V1 - Vc(self.c_dir, self.c_speed, eta1)
        wg = WG(eta1, eta1, V1, V1, self.c_dir, self.c_speed)
        rudder = Rudder(eta1, V1, self.c_dir, self.c_speed)
        # float's kinematic equations
        eta1_dot = np.dot(J(eta1), V1)

        Minv_1 = np.linalg.inv(wg.MRB_1() + wg.MA_1())

        MV1_dot = - np.dot(wg.CRB_1(), V1) - np.dot(wg.CA_1(), V1_r) - np.dot(wg.D_1(), V1_r) - wg.d_1() + rudder.force(angle) + WF

        V1_dot = np.dot(Minv_1, MV1_dot)

        return np.vstack((eta1_dot, V1_dot))

    def change_angle(self, degree):
        if degree > pi:
            output = degree - 2*pi
        elif degree < -pi:
            output = degree + 2*pi
        else:
            output = degree
        return output

    def obser(self, rudder_angle):

        k1 = self.f(self.state_0, rudder_angle)
        k2 = self.f(self.state_0 + 0.5 * k1 * self.time_step, rudder_angle)
        k3 = self.f(self.state_0 + 0.5 * k2 * self.time_step, rudder_angle)
        k4 = self.f(self.state_0 + k3 * self.time_step, rudder_angle)
        self.state_0 += (1 / 6) * (k1 + 2 * k2 + 2 * k3 + k4) * self.time_step
        self.state_0[3] = self.change_angle(self.state_0.item(3))
        self.t += 1

        self._t.append(self.t)
        self.x1.append(self.state_0.item(0))
        self.y1.append(self.state_0.item(1))
        self.z1.append(self.state_0.item(2))
        self.phi1.append(self.state_0.item(3))
        self.u1.append(self.state_0.item(4))
        self.v1.append(self.state_0.item(5))
        self.w1.append(self.state_0.item(6))
        self.r1.append(self.state_0.item(7))
        self.Rudder_angle.append(rudder_angle)
        data_storage(self.x1, self.y1, self.phi1, self.t, u1=self.u1, rudder_angle=self.Rudder_angle)  # store data in local files
        self.distance_target = math.hypot(self.state_0.item(0) - self.target_position[0],self.state_0.item(1) - self.target_position[1]) / 100
        self.distance_obstacle_1 = (math.hypot(self.state_0.item(0) - self.obstacle_1[0],self.state_0.item(1) - self.obstacle_1[1])-self.obst_R1) / 100
        self.distance_obstacle_2 = (math.hypot(self.state_0.item(0) - self.obstacle_2[0],self.state_0.item(1) - self.obstacle_2[1])-self.obst_R2) / 100
        self.distance_obstacle_3 = (math.hypot(self.state_0.item(0) - self.obstacle_3[0],self.state_0.item(1) - self.obstacle_3[1])-self.obst_R3) / 100
        self.distance_obstacle_4 = (math.hypot(self.state_0.item(0) - self.obstacle_4[0], self.state_0.item(1) - self.obstacle_4[1])-self.obst_R4) / 100
        self.distance_obstacle_5 = (math.hypot(self.state_0.item(0) - self.obstacle_5[0],self.state_0.item(1) - self.obstacle_5[1])-self.obst_R5) / 100

        self.course_error_target = self.state_0.item(3) - self.desired_course(self.target_position[0],self.target_position[1],self.state_0.item(0),self.state_0.item(1))
        self.course_error_obstacle_1 = self.state_0.item(3) - self.desired_course(self.obstacle_1[0],self.obstacle_1[1],self.state_0.item(0),self.state_0.item(1))
        self.course_error_obstacle_2 = self.state_0.item(3) - self.desired_course(self.obstacle_2[0],self.obstacle_2[1],self.state_0.item(0),self.state_0.item(1))
        self.course_error_obstacle_3 = self.state_0.item(3) - self.desired_course(self.obstacle_3[0],self.obstacle_3[1],self.state_0.item(0),self.state_0.item(1))
        self.course_error_obstacle_4 = self.state_0.item(3) - self.desired_course(self.obstacle_4[0],self.obstacle_4[1],self.state_0.item(0),self.state_0.item(1))
        self.course_error_obstacle_5 = self.state_0.item(3) - self.desired_course(self.obstacle_5[0],self.obstacle_5[1],self.state_0.item(0),self.state_0.item(1))

        self.distance_obstacle = list([self.distance_obstacle_1, self.distance_obstacle_2, self.distance_obstacle_3, self.distance_obstacle_4,self.distance_obstacle_5])
        self.course_error_obstacle = list([self.course_error_obstacle_1, self.course_error_obstacle_2, self.course_error_obstacle_3,self.course_error_obstacle_4, self.course_error_obstacle_5])
        i_obstacle = self.distance_obstacle.index(min(self.distance_obstacle))
        eplison = 1
        if self.distance_obstacle[i_obstacle] < 10:
            eplison = -1
        observation = np.array([self.distance_target, self.course_error_target, self.distance_obstacle[i_obstacle], self.course_error_obstacle[i_obstacle]])

        if self.n_features == 5:
            observation = np.array([self.distance_target, self.course_error_target, self.distance_obstacle[i_obstacle],
                                    self.course_error_obstacle[i_obstacle], eplison])
        return observation

    def step(self, action):
        s_ = np.array([0, 0, 0, 0, 0])
        a_1 = 5*pi/180
        a_2 = 2*pi/180
        a_3 = 0
        a_4 = -2*pi/180
        a_5 = -5*pi/180

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

        if self.t >= 400:
            done = True
        elif s_[0] < 0.05:
            done = True
        elif s_[2] <= 0.02:
            done = True
        else:
            done = False
        return s_, done

    def render(self):
        data_viewer(self.x1, self.y1, u1=self.u1, phit=self.phi1, rudder_angle=self.Rudder_angle, t=self._t, xlim_left=self.xlim_left, xlim_right=self.xlim_right, ylim_left=self.ylim_left, ylim_right=self.ylim_right,
                        goal_x=self.target_position[0], goal_y=self.target_position[1])

###############################  DDQN  ####################################
class DoubleDQN:
    def __init__(self, sess, n_features, n_actions):
        self.sess = sess
        self.n_features = n_features
        self.n_actions = n_actions
        self._build_net(S)
        #self.new_saver = tf.train.import_meta_graph('D:/Wave glider modelling/data/#6/OA_Final.meta')
        self.new_saver = tf.train.Saver()
        #self._build_net()

    def _build_net(self, s):
        # ------------------ build evaluate_net ------------------
        # self.s = tf.placeholder(tf.float32, [None, self.n_features], name='s')  # input
        #self.q_target = tf.placeholder(tf.float32, [None, self.n_actions], name='Q_target')  # for calculating loss
        with tf.variable_scope('eval_net'):
            # c_names(collections_names) are the collections to store variables
            c_names, n_l1, n_l2, n_l3, w_initializer, b_initializer = \
                ['eval_net_params', tf.GraphKeys.GLOBAL_VARIABLES], 200, 100, 50,\
                tf.random_normal_initializer(0., 0.3), tf.constant_initializer(0.1)  # config of layers

            # first layer. collections is used later when assign to target net
            with tf.variable_scope('l1'):
                w1 = tf.get_variable('w1', [self.n_features, n_l1], initializer=w_initializer, collections=c_names)
                b1 = tf.get_variable('b1', [1, n_l1], initializer=b_initializer, collections=c_names)
                l1 = tf.nn.relu(tf.matmul(s, w1) + b1)

            # second layer. collections is used later when assign to target net
            with tf.variable_scope('l2'):
                w2 = tf.get_variable('w2', [n_l1, n_l2], initializer=w_initializer, collections=c_names)
                b2 = tf.get_variable('b2', [1, n_l2], initializer=b_initializer, collections=c_names)
                l2 = tf.nn.relu(tf.matmul(l1, w2) + b2)

            with tf.variable_scope('l3'):
                w3 = tf.get_variable('w3', [n_l2, n_l3], initializer=w_initializer, collections=c_names)
                b3 = tf.get_variable('b3', [1, n_l3], initializer=b_initializer, collections=c_names)
                l3 = tf.nn.relu(tf.matmul(l2, w3) + b3)
            # output layer.
            with tf.variable_scope('l4'):
                w4 = tf.get_variable('w4', [n_l3, self.n_actions], initializer=w_initializer, collections=c_names)
                b4 = tf.get_variable('b4', [1, self.n_actions], initializer=b_initializer, collections=c_names)
                self.q_eval = tf.matmul(l3, w4) + b4


env = Waveglider()
state_dim = env.n_features
actions = 5

with tf.name_scope('S'):
    S = tf.placeholder(tf.float32, shape=[None, state_dim], name='s')
with tf.name_scope('S_'):
    S_ = tf.placeholder(tf.float32, shape=[None, state_dim], name='s_')


sess = tf.Session()
#new_saver = tf.train.import_meta_graph('/home/wp/waveglider_RL/Environment/data/Mymodel-100000.meta')
#graph = tf.get_default_graph()
DDQN = DoubleDQN(sess, state_dim, actions)
new_saver = DDQN.new_saver
new_saver.restore(sess, tf.train.latest_checkpoint('/home/wp/waveglider_RL/Environment/DDQNdata/'))

s = env.reset()
s = s[np.newaxis, :]
while True:
    env.render()
    q = sess.run(DDQN.q_eval, feed_dict={S: s})
    action = np.argmax(q)
    s_, done = env.step(action)
    s = s_
    s = s[np.newaxis, :]
    if done:
        break
print('Amazing! Eve give you a kiss!!!')