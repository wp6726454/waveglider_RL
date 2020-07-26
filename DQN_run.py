from Environment.USV_modeling import Waveglider
from DQN import DeepQNetwork


def run_WG():
    success = 0
    step = 0

    reach_save = '/home/wp/waveglider_RL/Environment/data/reach.json'
    episode_save = '/home/wp/waveglider_RL/Environment/data/episode.json'
    totalreward_save = '/home/wp/waveglider_RL/Environment/data/totalreward.json'

    for episode in range(5000):
        total_reward = 0
        # initial observation
        observation = env.reset()

        while True:
            # fresh env
            if step % 5 == 0:
                env.render()

            # RL choose action based on observation
            action = RL.choose_action(observation)

            # RL take action and get next observation and reward
            observation_, reward, done, reach = env.step(action, observation)

            total_reward += reward

            if reach == 1:
                success += 1

            RL.store_transition(observation, action, reward, observation_)

            if (step > 200) and (step % 2 == 0):

                RL.learn()
            # swap observation
            observation = observation_

            # break while loop when end of t10his episode
            if done:
                break
            step += 1

        with open(totalreward_save, 'a') as obj:
            obj.write('\n' + str(total_reward))

        if (episode > 1) and (episode % 100 == 0):
            RL.saver()
            with open(episode_save, 'a') as obj:
                obj.write('\n' + str(episode))
            with open(reach_save, 'a') as obj:
                obj.write('\n' + str(success))

        print('Episode:', episode, ',', 'Reach:', success, ',', 'Step:', step)
    # end of game
    print('train over')



if __name__ == "__main__":
    # maze game
    env = Waveglider()
    RL = DeepQNetwork(env.n_actions, env.n_features,
                      learning_rate=0.0005,
                      reward_decay=0.95,
                      e_greedy=0.99,
                      memory_size=10000,
                      batch_size=32,
                      e_greedy_increment=0.000005
                      # output_graph=True
                      )
    run_WG()
    RL.plot_cost()