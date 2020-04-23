import matplotlib.pyplot as plt
from Environment.data_process import data_delete_first_line


# plot the result figures according to the stored data
# data_delete_first_line()    #Turn on when error is reported

x1_json = 'D:\\Wave glider modelling\\data\\x1.json'
x1 = []
with open(x1_json) as f:
    for line in f:
        x1.append(line.strip('\n'))
x1 = list(map(float, x1))

y1_json = 'D:\\Wave glider modelling\\data\\y1.json'
y1 = []
with open(y1_json) as f:
    for line in f:
        y1.append(line.strip('\n'))
y1 = list(map(float, y1))

phit_json = 'D:\\Wave glider modelling\\data\\phit.json'
phit = []
with open(phit_json) as f:
    for line in f:
        phit.append(line.strip('\n'))
phit = list(map(float, phit))

rudder_angle_json = 'D:\\Wave glider modelling\\data\\rudder_angle.json'
rudder_angle = []
with open(rudder_angle_json) as f:
    for line in f:
        rudder_angle.append(line.strip('\n'))
rudder_angle = list(map(float, rudder_angle))

u1_json = 'D:\\Wave glider modelling\\data\\u1.json'
u1 = []
with open(u1_json) as f:
    for line in f:
        u1.append(line.strip('\n'))
u1 = list(map(float, u1))

t_json = 'D:\\Wave glider modelling\\data\\time.json'
t = []
with open(t_json) as f:
    for line in f:
        t.append(line.strip('\n'))
t = list(map(float, t))



path = plt.figure(1)
plt.plot(y1, x1, '-r')
plt.title('Path')
plt.ylabel('x(m)')
plt.xlabel('y(m)')


heading = plt.figure(2)
plt.plot(t, phit, '-b', label='phit')
plt.title('Heading')
plt.ylabel('Course(rad)')
plt.ylabel('Time(s)')

rudder = plt.figure(3)
plt.plot(t, rudder_angle, '-r')
plt.title('Rudder angle')
plt.ylabel('Rudder angle(deg)')
plt.ylabel('Time(s)')

speed = plt.figure(4)
plt.plot(t, u1, '-r')
plt.title('Sailing speed')
plt.ylabel('Sailing speed(m/s)')
plt.ylabel('Time(s)')

plt.show()

