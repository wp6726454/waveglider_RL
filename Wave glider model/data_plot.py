import matplotlib.pyplot as plt
import fileinput

# plot the result figures according to the stored data
x1_json = 'D:\\Wave glider modelling\\data\\x1.json'
x1 = []
for line in fileinput.input(x1_json, inplace=1):
    if not fileinput.isfirstline():
        print(line.replace('\n', ''))
with open(x1_json) as f:
    for line in f:
        x1.append(line.strip('\n'))
x1 = list(map(float, x1))

y1_json = 'D:\\Wave glider modelling\\data\\y1.json'
y1 = []
for line in fileinput.input(y1_json, inplace=1):
    if not fileinput.isfirstline():
        print(line.replace('\n', ''))
with open(y1_json) as f:
    for line in f:
        y1.append(line.strip('\n'))
y1 = list(map(float, y1))

phit_json = 'D:\\Wave glider modelling\\data\\phit.json'
phit = []
for line in fileinput.input(phit_json, inplace=1):
    if not fileinput.isfirstline():
        print(line.replace('\n', ''))
with open(phit_json) as f:
    for line in f:
        phit.append(line.strip('\n'))
phit = list(map(float, phit))

rudder_angle_json = 'D:\\Wave glider modelling\\data\\rudder_angle.json'
rudder_angle = []
for line in fileinput.input(rudder_angle_json, inplace=1):
    if not fileinput.isfirstline():
        print(line.replace('\n', ''))
with open(rudder_angle_json) as f:
    for line in f:
        rudder_angle.append(line.strip('\n'))
rudder_angle = list(map(float, rudder_angle))

t_json = 'D:\\Wave glider modelling\\data\\time.json'
t = []
for line in fileinput.input(t_json, inplace=1):
    if not fileinput.isfirstline():
        print(line.replace('\n', ''))
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
plt.plot(t, phit, '-b', label = 'phit')
plt.title('Heading')
plt.ylabel('Course(rad)')
plt.ylabel('Time(s)')

rudder_angle = plt.figure(3)
plt.plot(t, rudder_angle, '-r')
plt.title('Rudder angle')
plt.ylabel('Rudder angle(deg)')
plt.ylabel('Time(s)')

plt.show()

