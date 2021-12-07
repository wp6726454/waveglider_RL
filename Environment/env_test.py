import numpy as np
import math


env_test1 = np.zeros((100,1,3))
env_test2 = np.zeros((100,2,3))
env_test3 = np.zeros((100,3,3))
env_test4 = np.zeros((100,4,3))
env_test5 = np.zeros((100,5,3))

#env_test1 development
for i in range(100):
    for j in range(1):
        env_test1[i][j][0]= np.random.randint(15,85)
        env_test1[i][j][1]= np.random.randint(15,85)
        env_test1[i][j][2]= np.random.randint(5,15)

#env_test2 development
for i in range(100):
    while True:
        for j in range(2):
            env_test2[i][j][0]= np.random.randint(15,85)
            env_test2[i][j][1]= np.random.randint(15,85)
            env_test2[i][j][2]= np.random.randint(5, 15)
        dis_1 = math.hypot(env_test2[i][0][0] - env_test2[i][1][0], env_test2[i][0][1] - env_test2[i][1][1])- \
                env_test2[i][0][2]-env_test2[i][1][2]
        if dis_1 > 20:
            break

#env_test3 development
for i in range(100):
    while True:
        for j in range(3):
            env_test3[i][j][0]= np.random.randint(15,85)
            env_test3[i][j][1]= np.random.randint(15,85)
            env_test3[i][j][2]= np.random.randint(5, 15)
        dis_1 = math.hypot(env_test3[i][0][0] - env_test3[i][1][0], env_test3[i][0][1] - env_test3[i][1][1])- \
                env_test3[i][0][2]-env_test3[i][1][2]
        dis_2 = math.hypot(env_test3[i][0][0] - env_test3[i][2][0], env_test3[i][0][1] - env_test3[i][2][1])- \
                env_test3[i][0][2]-env_test3[i][2][2]
        dis_3 = math.hypot(env_test3[i][1][0] - env_test3[i][2][0], env_test3[i][1][1] - env_test3[i][2][1])- \
                env_test3[i][1][2]-env_test3[i][2][2]
        if dis_1 > 20 and dis_2 > 20 and dis_3 > 20:
            break
#env_test4 development
for i in range(100):
    while True:
        for j in range(4):
            env_test4[i][j][0]= np.random.randint(15,85)
            env_test4[i][j][1]= np.random.randint(15,85)
            env_test4[i][j][2]= np.random.randint(5, 15)
        dis_1 = math.hypot(env_test4[i][0][0] - env_test4[i][1][0], env_test4[i][0][1] - env_test4[i][1][1]) - \
                env_test4[i][0][2] - env_test4[i][1][2]
        dis_2 = math.hypot(env_test4[i][0][0] - env_test4[i][2][0], env_test4[i][0][1] - env_test4[i][2][1]) - \
                env_test4[i][0][2] - env_test4[i][2][2]
        dis_3 = math.hypot(env_test4[i][0][0] - env_test4[i][3][0], env_test4[i][0][1] - env_test4[i][3][1]) - \
                env_test4[i][0][2] - env_test4[i][3][2]
        dis_4 = math.hypot(env_test4[i][1][0] - env_test4[i][2][0], env_test4[i][1][1] - env_test4[i][2][1]) - \
                env_test4[i][1][2] - env_test4[i][2][2]
        dis_5 = math.hypot(env_test4[i][1][0] - env_test4[i][3][0], env_test4[i][1][1] - env_test4[i][3][1]) - \
                env_test4[i][1][2] - env_test4[i][3][2]
        dis_6 = math.hypot(env_test4[i][2][0] - env_test4[i][3][0], env_test4[i][2][1] - env_test4[i][3][1]) - \
                env_test4[i][2][2] - env_test4[i][3][2]
        if dis_1 > 20 and dis_2 > 20 and dis_3 > 20 and dis_4 > 20 and dis_5 > 20 and dis_6 > 20:
            break
#env_test5 development
for i in range(5):
    while True:
        for j in range(5):
            env_test5[i][j][0]= np.random.randint(15,85)
            env_test5[i][j][1]= np.random.randint(15,85)
            env_test5[i][j][2]= np.random.randint(5, 15)
        dis_1 = math.hypot(env_test5[i][0][0] - env_test5[i][1][0], env_test5[i][0][1] - env_test5[i][1][1]) - \
                env_test5[i][0][2] - env_test5[i][1][2]
        dis_2 = math.hypot(env_test5[i][0][0] - env_test5[i][2][0], env_test5[i][0][1] - env_test5[i][2][1]) - \
                env_test5[i][0][2] - env_test5[i][2][2]
        dis_3 = math.hypot(env_test5[i][0][0] - env_test5[i][3][0], env_test5[i][0][1] - env_test5[i][3][1]) - \
                env_test5[i][0][2] - env_test5[i][3][2]
        dis_4 = math.hypot(env_test5[i][1][0] - env_test5[i][2][0], env_test5[i][1][1] - env_test5[i][2][1]) - \
                env_test5[i][1][2] - env_test5[i][2][2]
        dis_5 = math.hypot(env_test5[i][1][0] - env_test5[i][3][0], env_test5[i][1][1] - env_test5[i][3][1]) - \
                env_test5[i][1][2] - env_test5[i][3][2]
        dis_6 = math.hypot(env_test5[i][2][0] - env_test5[i][3][0], env_test5[i][2][1] - env_test5[i][3][1]) - \
                env_test5[i][2][2] - env_test5[i][3][2]
        dis_7 = math.hypot(env_test5[i][0][0] - env_test5[i][4][0], env_test5[i][0][1] - env_test5[i][4][1]) - \
                env_test5[i][0][2] - env_test5[i][4][2]
        dis_8 = math.hypot(env_test5[i][1][0] - env_test5[i][4][0], env_test5[i][1][1] - env_test5[i][4][1]) - \
                env_test5[i][1][2] - env_test5[i][4][2]
        dis_9 = math.hypot(env_test5[i][2][0] - env_test5[i][4][0], env_test5[i][2][1] - env_test5[i][4][1]) - \
                env_test5[i][2][2] - env_test5[i][4][2]
        dis_0 = math.hypot(env_test5[i][3][0] - env_test5[i][4][0], env_test5[i][3][1] - env_test5[i][4][1]) - \
                env_test5[i][3][2] - env_test5[i][4][2]
        if dis_1 > 20 and dis_2 > 20 and dis_3 > 20 and dis_4 > 20 and dis_5 > 20 and dis_6 > 20 and dis_7 > 20 \
                and dis_8 > 20 and dis_9 > 20 and dis_0 > 20:
            break
np.save("/home/wp/waveglider_RL/Environment/data/env_test5.npy", env_test5)
a = np.load("/home/wp/waveglider_RL/Environment/data/env_test5.npy")
print(a)