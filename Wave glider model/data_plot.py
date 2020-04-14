# plot the result figures according to the saved data
import matplotlib.pyplot as plt
import fileinput


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

phid_json = 'D:\\Wave glider modelling\\data\\phid.json'
phid = []
for line in fileinput.input(phid_json, inplace=1):
    if not fileinput.isfirstline():
        print(line.replace('\n', ''))
with open(phid_json) as f:
    for line in f:
        phid.append(line.strip('\n'))
phid = list(map(float, phid))

phit_json = 'D:\\Wave glider modelling\\data\\phit.json'
phit = []
for line in fileinput.input(phit_json, inplace=1):
    if not fileinput.isfirstline():
        print(line.replace('\n', ''))
with open(phit_json) as f:
    for line in f:
        phit.append(line.strip('\n'))
phit = list(map(float, phit))

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
plt.plot(t, phid, '-r', label = 'phid')
plt.plot(t, phit, '-b', label = 'phit')
plt.title('Heading')
plt.ylabel('Course(rad)')
plt.ylabel('Time(s)')
plt.legend(bbox_to_anchor=(1,1),#图例边界框起始位置
                 loc="upper right",#图例的位置
                 ncol=1,#列数
                 mode="None",#当值设置为“expend”时，图例会水平扩展至整个坐标轴区域
                 borderaxespad=0,#坐标轴和图例边界之间的间距
                 shadow=False,#是否为线框添加阴影
                 fancybox=True)#线框圆角处理参数

plt.show()

