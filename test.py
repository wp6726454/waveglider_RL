import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt

obser = np.array([[1,2,3],[2,3,4],[3,4,5],[5,6,7],[6,7,8],[7,7,8],[1,2,3],[2,3,4],[3,4,5],[5,6,7]])
x = obser[np.newaxis, :, np.newaxis]
outs = tf.reshape(x, [-1, 5, 3])
#print(x)
print(outs)