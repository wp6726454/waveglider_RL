
import numpy as np
from scipy.integrate import odeint
from odetw import odeintw
'''

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
‘’‘



import numpy as np
from odetw import odeintw
import matplotlib.pyplot as plt


def system(M, t):
    A, B, C = M
    dA_dt = A.dot(C) + B.dot(C)
    dB_dt = B.dot(C)
    dC_dt = C
    return np.array([dA_dt, dB_dt, dC_dt])


t = np.linspace(0, 1.5, 1000)

#A_initial= [1, 2, 2.3, 4.3, 2.1, 5.2, 2.13, 3.43]
A_initial = np.array([[1 + 2.1j, 2 + 5.2j], [2.3 + 2.13j, 4.3 + 3.43j]])

# B_initial= [7, 2.7, 1.23, 3.3, 3.1, 5.12, 1.13, 3]
B_initial = np.array([[7 + 3.1j, 2.7 + 5.12j], [1.23 + 1.13j, 3.3 + 3j]])

# C_initial= [0.5, 0.9, 0.63, 0.43, 0.21, 0.5, 0.11, 0.3]
C_initial = np.array([[0.5 + 0.21j, 0.9 + 0.5j], [0.63 + 0.11j, 0.43 + 0.3j]])

M_initial = np.array([A_initial, B_initial, C_initial])
sol = odeintw(system, M_initial, t)

A = sol[:, 0, :, :]
B = sol[:, 1, :, :]
C = sol[:, 2, :, :]

plt.figure(1)
plt.plot(t, A[:, 0, 0].real, label='A(t)[0,0].real')
plt.plot(t, A[:, 0, 0].imag, label='A(t)[0,0].imag')
plt.legend(loc='best')
plt.grid(True)
plt.xlabel('t')

A_evals = np.linalg.eigvals(A)

plt.figure(2)
plt.plot(t, A_evals[:,0].real, 'b.', markersize=3, mec='b')
plt.plot(t, A_evals[:,0].imag, 'r.', markersize=3, mec='r')
plt.plot(t, A_evals[:,1].real, 'b.', markersize=3, mec='b')
plt.plot(t, A_evals[:,1].imag, 'r.', markersize=3, mec='r')
plt.ylim(-200, 1200)
plt.grid(True)
plt.title('Real and imaginary parts of the eigenvalues of A(t)')
plt.xlabel('t')
plt.show()

'''

def f(A, t, Ab):
    y, z = A
    return np.array([z, np.dot(Ab, y)])

Ab = np.array([[-0.25, 0, 0],
               [0.25, -0.2, 0],
               [0, 0.2, -0.1]])
time = np.linspace(0, 2, 20)
A0 = np.array([[[10],
               [20],
               [30]],
              [[1],
               [2],
               [3]]])
#B0 = np.array([1,2,0])
MA = odeintw(f, A0, time, args = (Ab,))
print(MA[:,0,0])