import numpy as np
import pandas as pd
import tensorflow as tf

np.random.seed(1)
tf.set_random_seed(1)


# Recursive Q Network off-policy
class RQN:
    def __init__(
            self,
            n_actions,
            n_features,
            learning_rate=0.01,
            reward_decay=0.9,
            e_greedy=0.9,
            memory_size=1000,
            replace_target_iter=5,
            batch_size=72,
            e_greedy_increment=None,
            output_graph=False,
            TIME_STEP=8,
    ):
        self.n_actions = n_actions
        self.n_features = n_features
        self.lr = learning_rate
        self.gamma = reward_decay
        self.epsilon_max = e_greedy
        self.memory_size = memory_size
        self.replace_target_iter = replace_target_iter
        self.batch_size = batch_size
        self.epsilon_increment = e_greedy_increment
        self.epsilon = 0 if e_greedy_increment is not None else self.epsilon_max
        self.TIME_STEP = TIME_STEP

        # total learning step
        self.learn_step_counter = 0

        # initialize zero memory [s, a, r, s_]
        self.memory = np.zeros((self.memory_size, n_features * 2 + 2))

        self.observation_input = np.zeros((self.TIME_STEP, self.n_features))

        # consist of [target_net, evaluate_net]
        self._build_net()
        t_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES,scope='target_net')
        e_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES,scope='eval_net')
        self.replace_target_op = [tf.assign(t, e) for t, e in zip(t_params, e_params)]

        self.sess = tf.Session()

        if output_graph:
            # $ tensorboard --logdir=logs
            # tf.train.SummaryWriter soon be deprecated, use following
            tf.summary.FileWriter("logs/", self.sess.graph)

        self.sess.run(tf.global_variables_initializer())
        self.cost_his = []


    def _build_net(self):
        # ------------------ build evaluate_net ------------------
        self.s = tf.placeholder(tf.float32, [None, self.n_features], name='s')  # input
        self.q_target = tf.placeholder(tf.float32, [None, self.n_actions], name='Q_target')  # for calculating loss
        with tf.variable_scope('eval_net'):

            input_x = tf.reshape(self.s, [-1, self.TIME_STEP, self.n_features])
            # RNN
            rnn_cell = tf.nn.rnn_cell.LSTMCell(num_units=64)
            outputs, (h_c, h_n) = tf.nn.dynamic_rnn(
                rnn_cell,  # cell you have chosen
                input_x,  # input
                initial_state=None,  # the initial hidden state
                dtype=tf.float32,  # must given if set initial_state = None
                time_major=False,  # False: (batch, time step, input); True: (time step, batch, input)
            )
            # output layer.
            self.q_eval = tf.layers.dense(h_n, 5)


        with tf.variable_scope('loss'):
            self.loss = tf.reduce_mean(tf.squared_difference(self.q_target, self.q_eval))
        with tf.variable_scope('train'):
            self._train_op = tf.train.RMSPropOptimizer(self.lr).minimize(self.loss)

            # ------------------ build evaluate_net ------------------
        self.s_ = tf.placeholder(tf.float32, [None, self.n_features], name='s_')  # input
        with tf.variable_scope('target_net'):
            input_x = tf.reshape(self.s_, [-1, self.TIME_STEP, self.n_features])
            # RNN
            rnn_cell = tf.nn.rnn_cell.LSTMCell(num_units=64)
            outputs, (h_c, h_n) = tf.nn.dynamic_rnn(
                rnn_cell,  # cell you have chosen
                input_x,  # input
                initial_state=None,  # the initial hidden state
                dtype=tf.float32,  # must given if set initial_state = None
                time_major=False,  # False: (batch, time step, input); True: (time step, batch, input)
            )
            # output layer.
            self.q_next = tf.layers.dense(h_n, 5)


    def store_transition(self, s, a, r, s_):
        if not hasattr(self, 'memory_counter'):
            self.memory_counter = 0

        transition = np.hstack((s, [a, r], s_))

        if self.memory_counter < self.memory_size:
            # replace the old memory with new memory
            index = self.memory_counter % self.memory_size
            self.memory[index, :] = transition
            self.memory_counter += 1
        else:
            np.delete(self.memory, 0 ,axis=0)
            np.append(self.memory, transition[np.newaxis,:],axis=0)

    def choose_action(self, observation):
        if not hasattr(self, 'observation_counter'):
            self.observation_counter = 0

        if self.observation_counter < self.TIME_STEP:
            # replace the old memory with new memory
            index = self.observation_counter % self.TIME_STEP
            self.observation_input[index, :] = observation
            self.observation_counter += 1
        else:
            np.delete(self.observation_input, 0 ,axis=0)
            np.append(self.observation_input, observation[np.newaxis,:],axis=0)


        if np.random.uniform() < self.epsilon:
            # forward feed the observation and get q value for every actions
            actions_value = self.sess.run(self.q_eval, feed_dict={self.s: self.observation_input})
            action = np.argmax(actions_value)
        else:
            action = np.random.randint(0, self.n_actions)
        return action

    def learn(self):
        # check to replace target parameters
        if self.learn_step_counter % self.replace_target_iter == 0:
            self.sess.run(self.replace_target_op)
            print('\ntarget_params_replaced\n')

        # sample batch memory from all memory
        if self.memory_counter > self.memory_size:
            sample_index = np.random.choice(self.memory_size, size=self.batch_size)
        else:
            sample_index = np.random.choice(self.memory_counter, size=self.batch_size)
        batch_memory = self.memory[sample_index, :]

        q_next, q_eval = self.sess.run(
            [self.q_next, self.q_eval],
            feed_dict={
                self.s_: batch_memory[:, -self.n_features:],
                self.s: batch_memory[:, :self.n_features],
            })

        # change q_target w.r.t q_eval's action
        q_target = q_eval.copy()

        q_index = np.arange(self.batch_size/self.TIME_STEP, dtype=np.int32)
        eval_act_index = batch_memory[7::8, self.n_features].astype(int)
        reward = batch_memory[7::8, self.n_features + 1]
        q_target[q_index, eval_act_index] = reward + self.gamma * np.max(q_next, axis=1)


        # train eval network
        _, self.cost = self.sess.run([self._train_op, self.loss],
                                     feed_dict={self.s: batch_memory[:, :self.n_features],
                                                self.q_target: q_target})
        self.cost_his.append(self.cost)

        # increasing epsilon
        #self.epsilon = self.epsilon + self.epsilon_increment if self.epsilon < self.epsilon_max else self.epsilon_max
        self.learn_step_counter += 1

    def plot_cost(self):
        import matplotlib.pyplot as plt
        plt.plot(np.arange(len(self.cost_his)), self.cost_his)
        plt.ylabel('Cost')
        plt.xlabel('training steps')
        plt.show()



