"""
Deep Deterministic Policy Gradient (DDPG), Reinforcement Learning.
DDPG is Actor Critic based algorithm.
Pendulum example.
View more on my tutorial page: https://morvanzhou.github.io/tutorials/
Using:
tensorflow 1.0
gym 0.8.0
"""

import tensorflow as tf
import numpy as np
import math
from math import *
import gym
import time
from Environment.USV_modeling_DDPG import Waveglider

np.random.seed(1)
tf.set_random_seed(1)

#####################  hyper parameters  ####################

MAX_EPISODES = 10000
#MAX_EP_STEPS = 200
LR_A = 0.0002    # learning rate for actor
LR_C = 0.0002   # learning rate for critic
GAMMA = 0.95     # reward discount
REPLACEMENT = [
    dict(name='soft', tau=0.01),
    dict(name='hard', rep_iter_a=200, rep_iter_c=200)
][0]            # you can try different target replacement strategies
MEMORY_CAPACITY = 5000
BATCH_SIZE = 32

RENDER = False
OUTPUT_GRAPH = False
#ENV_NAME = 'Pendulum-v0'

###############################  Actor  ####################################


class Actor(object):
    def __init__(self, sess, action_dim, action_bound, learning_rate, replacement):
        self.sess = sess
        self.a_dim = action_dim
        self.action_bound = action_bound
        self.lr = learning_rate
        self.replacement = replacement
        self.t_replace_counter = 0
        self.dueling = True

        with tf.variable_scope('Actor'):
            # input s, output a
            self.a = self._build_net(S, scope='eval_net', trainable=True)

            # input s_, output a, get a_ for critic
            self.a_ = self._build_net(S_, scope='target_net', trainable=False)

        self.e_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Actor/eval_net')
        self.t_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Actor/target_net')

        if self.replacement['name'] == 'hard':
            self.t_replace_counter = 0
            self.hard_replace = [tf.assign(t, e) for t, e in zip(self.t_params, self.e_params)]
        else:
            self.soft_replace = [tf.assign(t, (1 - self.replacement['tau']) * t + self.replacement['tau'] * e)
                                 for t, e in zip(self.t_params, self.e_params)]

    def _build_net(self, s, scope, trainable):
        with tf.variable_scope(scope):
            init_w = tf.random_normal_initializer(0., 0.3)
            init_b = tf.constant_initializer(0.1)
            net = tf.layers.dense(s, 200, activation=tf.nn.relu,
                                  kernel_initializer=init_w, bias_initializer=init_b, name='l1',
                                  trainable=trainable)
            with tf.variable_scope('l1'):
                l1 = tf.layers.dense(net, 100, activation=tf.nn.tanh, kernel_initializer=init_w,
                                          bias_initializer=init_b, name='a', trainable=trainable)
            with tf.variable_scope('l2'):
                l2 = tf.layers.dense(l1, 50, activation=tf.nn.tanh, kernel_initializer=init_w,
                                          bias_initializer=init_b, name='a', trainable=trainable)

            if self.dueling:
                #Dueling DDPG
                with tf.variable_scope('aO'):
                    actions_O = tf.layers.dense(l2, self.a_dim, activation=tf.nn.tanh, kernel_initializer=init_w,
                                         bias_initializer=init_b, name='a', trainable=trainable)
                    scaled_aO = tf.multiply(actions_O, self.action_bound, name='scaled_aO')
                    #aO = np.clip(np.random.normal(scaled_aO, self.var), -1 * pi / 180, 1 * pi / 180)

                with tf.variable_scope('aT'):
                    actions_T = tf.layers.dense(l2, self.a_dim, activation=tf.nn.tanh, kernel_initializer=init_w,
                                         bias_initializer=init_b, name='a', trainable=trainable)
                    scaled_aT = tf.multiply(actions_T, self.action_bound, name='scaled_aT')
                    #aT = np.clip(np.random.normal(scaled_aT, self.var), -1 * pi / 180, 1 * pi / 180)

                with tf.variable_scope('aR'):
                    actions_R = tf.layers.dense(l2, self.a_dim, activation=tf.nn.tanh, kernel_initializer=init_w,
                                         bias_initializer=init_b, name='a', trainable=trainable)
                    scaled_aR = tf.multiply(actions_R, self.action_bound, name='scaled_aR')
                    #aR = np.clip(np.random.normal(scaled_aR, self.var), -1 * pi / 180, 1 * pi / 180)

                with tf.variable_scope('a'):
                    scaled_a =  (1/3)*scaled_aO + (1/3)*scaled_aT + (1/3)*scaled_aR

            else:
                with tf.variable_scope('a'):
                    actions = tf.layers.dense(l2, self.a_dim, activation=tf.nn.tanh, kernel_initializer=init_w,
                                              bias_initializer=init_b, name='a', trainable=trainable)
                    scaled_a = tf.multiply(actions, self.action_bound, name='scaled_a')  # Scale output to -action_bound to action_bound
            return scaled_a

    def learn(self, s):   # batch update
        self.sess.run(self.train_op, feed_dict={S: s})

        if self.replacement['name'] == 'soft':
            self.sess.run(self.soft_replace)
        else:
            if self.t_replace_counter % self.replacement['rep_iter_a'] == 0:
                self.sess.run(self.hard_replace)
            self.t_replace_counter += 1

    def choose_action(self, s):
        s = s[np.newaxis, :]    # single state
        return self.sess.run(self.a, feed_dict={S: s})[0]  # single action

    def add_grad_to_graph(self, a_grads):
        with tf.variable_scope('policy_grads'):
            # ys = policy;
            # xs = policy's parameters;
            # a_grads = the gradients of the policy to get more Q
            # tf.gradients will calculate dys/dxs with a initial gradients for ys, so this is dq/da * da/dparams
            self.policy_grads = tf.gradients(ys=self.a, xs=self.e_params, grad_ys=a_grads)

        with tf.variable_scope('A_train'):
            opt = tf.train.AdamOptimizer(-self.lr)  # (- learning rate) for ascent policy
            self.train_op = opt.apply_gradients(zip(self.policy_grads, self.e_params))

    def saver(self, step):
        saver = tf.train.Saver()
        saver.save(self.sess, '/home/wp/waveglider_RL/Environment/data/Mymodel', global_step= step)
###############################  Critic  ####################################

class Critic(object):
    def __init__(self, sess, state_dim, action_dim, learning_rate, gamma, replacement, a, a_):
        self.sess = sess
        self.s_dim = state_dim
        self.a_dim = action_dim
        self.lr = learning_rate
        self.gamma = gamma
        self.replacement = replacement

        with tf.variable_scope('Critic'):
            # Input (s, a), output q
            self.a = tf.stop_gradient(a)    # stop critic update flows to actor
            self.q = self._build_net(S, self.a, 'eval_net', trainable=True)

            # Input (s_, a_), output q_ for q_target
            self.q_ = self._build_net(S_, a_, 'target_net', trainable=False)    # target_q is based on a_ from Actor's target_net

            self.e_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Critic/eval_net')
            self.t_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Critic/target_net')

        with tf.variable_scope('target_q'):
            self.target_q = R + self.gamma * self.q_

        with tf.variable_scope('TD_error'):
            self.loss = tf.reduce_mean(tf.squared_difference(self.target_q, self.q))

        with tf.variable_scope('C_train'):
            self.train_op = tf.train.AdamOptimizer(self.lr).minimize(self.loss)

        with tf.variable_scope('a_grad'):
            self.a_grads = tf.gradients(self.q, self.a)[0]   # tensor of gradients of each sample (None, a_dim)

        if self.replacement['name'] == 'hard':
            self.t_replace_counter = 0
            self.hard_replacement = [tf.assign(t, e) for t, e in zip(self.t_params, self.e_params)]
        else:
            self.soft_replacement = [tf.assign(t, (1 - self.replacement['tau']) * t + self.replacement['tau'] * e)
                                     for t, e in zip(self.t_params, self.e_params)]

    def _build_net(self, s, a, scope, trainable):
        with tf.variable_scope(scope):
            init_w = tf.random_normal_initializer(0., 0.1)
            init_b = tf.constant_initializer(0.1)

            with tf.variable_scope('l1'):
                n_l1 = 200
                w1_s = tf.get_variable('w1_s', [self.s_dim, n_l1], initializer=init_w, trainable=trainable)
                w1_a = tf.get_variable('w1_a', [self.a_dim, n_l1], initializer=init_w, trainable=trainable)
                b1 = tf.get_variable('b1', [1, n_l1], initializer=init_b, trainable=trainable)
                l1 = tf.nn.relu(tf.matmul(s, w1_s) + tf.matmul(a, w1_a) + b1)

            with tf.variable_scope('l2'):
                l2 = tf.layers.dense(l1, 100, activation=tf.nn.tanh, kernel_initializer=init_w,
                                          bias_initializer=init_b, name='a', trainable=trainable)
            with tf.variable_scope('l3'):
                l3 = tf.layers.dense(l2, 50, activation=tf.nn.tanh, kernel_initializer=init_w,
                                          bias_initializer=init_b, name='a', trainable=trainable)
            with tf.variable_scope('q'):
                q = tf.layers.dense(l3, 1, kernel_initializer=init_w, bias_initializer=init_b, trainable=trainable)   # Q(s,a)
        return q

    def learn(self, s, a, r, s_):
        self.sess.run(self.train_op, feed_dict={S: s, self.a: a, R: r, S_: s_})
        if self.replacement['name'] == 'soft':
            self.sess.run(self.soft_replacement)
        else:
            if self.t_replace_counter % self.replacement['rep_iter_c'] == 0:
                self.sess.run(self.hard_replacement)
            self.t_replace_counter += 1


#####################  Memory  ####################

class Memory(object):
    def __init__(self, capacity, dims):
        self.capacity = capacity
        self.data = np.zeros((capacity, dims))
        self.pointer = 0

    def store_transition(self, s, a, r, s_):
        transition = np.hstack((s, a, [r], s_))
        index = self.pointer % self.capacity  # replace the old memory with new memory
        self.data[index, :] = transition
        self.pointer += 1

    def sample(self, n):
        assert self.pointer >= self.capacity, 'Memory has not been fulfilled'
        indices = np.random.choice(self.capacity, size=n)
        return self.data[indices, :]


#env = gym.make(ENV_NAME)
#env = env.unwrapped
#env.seed(1)
env = Waveglider()

state_dim = env.n_features
action_dim = 1
action_bound = pi/180
var = 3  # control exploration


# all placeholder for tf
with tf.name_scope('S'):
    S = tf.placeholder(tf.float32, shape=[None, state_dim], name='s')
with tf.name_scope('R'):
    R = tf.placeholder(tf.float32, [None, 1], name='r')
with tf.name_scope('S_'):
    S_ = tf.placeholder(tf.float32, shape=[None, state_dim], name='s_')


sess = tf.Session()

# Create actor and critic.
# They are actually connected to each other, details can be seen in tensorboard or in this picture:
actor = Actor(sess, action_dim, action_bound, LR_A, REPLACEMENT)
critic = Critic(sess, state_dim, action_dim, LR_C, GAMMA, REPLACEMENT, actor.a, actor.a_)
actor.add_grad_to_graph(critic.a_grads)

sess.run(tf.global_variables_initializer())

M = Memory(MEMORY_CAPACITY, dims=2 * state_dim + action_dim + 1)

if OUTPUT_GRAPH:
    tf.summary.FileWriter("logs/", sess.graph)


step = 0
success = 0
t1 = time.time()
totalreward_save = '/home/wp/waveglider_RL/Environment/data/totalreward.json'

for i in range(MAX_EPISODES):
    s = env.reset()
    ep_reward = 0

    while True:

        if ep_reward > 400 and step % 2 == 0:
            env.render()

        # Add exploration noise
        a = actor.choose_action(s)
        #a = max(min(a, max(-1*pi/180, 1*pi/180)), min(-1*pi/180, 1*pi/180))  #contrain the output
        a = np.clip(np.random.normal(a, var), -1*pi/180, 1*pi/180)# add randomness to action selection for exploration
        s_, r, done, reach = env.step(float(a), s)
        if reach == 1:
            success += 1

        M.store_transition(s, a, r, s_)

        if M.pointer > MEMORY_CAPACITY:
            var *= .999995    # decay the action randomness
            b_M = M.sample(BATCH_SIZE)
            b_s = b_M[:, :state_dim]
            b_a = b_M[:, state_dim: state_dim + action_dim]
            b_r = b_M[:, -state_dim - 1: -state_dim]
            b_s_ = b_M[:, -state_dim:]

            if step % 2 == 0:
                critic.learn(b_s, b_a, b_r, b_s_)
                actor.learn(b_s)

        s = s_
        ep_reward += r

        if done:
            break
        step += 1

        if (step > 1) and (step % 400000 == 0):
            actor.saver(step)

    with open(totalreward_save, 'a') as obj:
        obj.write('\n' + str(ep_reward))

    print('Episode:', i, ',', 'Reach:', success, ',', 'Step:', step, ',', 'Explore:', var, ',', 'Reward:', ep_reward)

print('Running time: ', time.time()-t1)

#this is a test try