
import numpy as np
''''
from scipy.integrate import odeint
import matplotlib.pyplot as plt
#二阶微分方程
def diff_equation(y_list,x):
    #y''+y=0 二阶的话，要换成两个一阶微分方程的方程组
    #设y'=z
    #那么z'=y''=-y
    #手工解也很容易知道是 y=C1sin(x)+C2cos(x)
    y,z=y_list
    return np.array([z,-y])
x=np.linspace(0,np.pi*4,num=100)
y0=np.array([1,0])#y(0)=1,y'(0)=1
result=odeint(diff_equation,y0,x)
plt.plot(x,result[:,0],label='y')#y的图像，y=cos(x)+sin(x)
plt.plot(x,result[:,1],label='z')#z的图像，也就是y'的图像，z=-sin(x)+cos(x)
plt.legend()
plt.grid()
plt.show()

'''
state_0 = np.array([[0], [0], [0], [0],  # eta1
                    [0], [0], [0], [0],  # V1
                    [0], [0], [6.2], [0],  # eta2
                    [0], [0], [0], [0]], float)  # V2
print(state_0)