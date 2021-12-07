import matplotlib.pyplot as plt
import  numpy as np
#environment number
n=4

a = np.load("/home/wp/waveglider_RL/Environment/data/env_test5.npy")

plt.figure(1, figsize=(4, 10))
plt.axis([-10, 110, -10, 110])
theta = np.arange(0,2*np.pi, 0.01)
plt.plot(0+1*np.cos(theta), 0+1*np.sin(theta), color='b') #start position
plt.plot(100+2*np.cos(theta), 100+2*np.sin(theta), color='r') #target position

for i in range(5):
    plt.plot(a[n][i][1] + a[n][i][2] * np.cos(theta), a[n][i][0]+ a[n][i][2] * np.sin(theta), color='k')

plt.show()