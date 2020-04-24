from Environment.Waveglider_simulation import Waveglider
from DRQN import DeepQNetwork


def run_WG():

    for episode in range(100):
        step = 0
        # initial observation
        observation = env.reset()

        while True:
            # fresh env
            env.render()

            # RL choose action based on observation
            action = RL.choose_action(observation)

            # RL take action and get next observation and reward
            observation_, reward, done = env.step(action)

            RL.store_transition(observation, action, reward, observation_)

            if (step > 100) and (step % 5 == 0):

                RL.learn()
            # swap observation
            observation = observation_

            # break while loop when end of t10his episode
            if done:
                break
            step += 1
            print(step)

    # end of game
    print('train over')



if __name__ == "__main__":
    # maze game
    env = Waveglider()
    RL = DeepQNetwork(env.n_actions, env.n_features,
                      learning_rate=0.01,
                      reward_decay=0.9,
                      e_greedy=0.9,
                      memory_size=1000,
                      # output_graph=True
                      )
    run_WG()
    RL.plot_cost()