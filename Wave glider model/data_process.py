import os


# store data in local files
def data_storage(x1, y1, phit, t,
                 u1='', T='', z1='',
                 x2='', y2='', z2='',
                 rudder_angle=''):
    x1_save = 'D:\\Wave glider modelling\\data\\x1.json'
    with open(x1_save, 'a') as obj:
        obj.write('\n' + str(x1[-1]))

    y1_save = 'D:\\Wave glider modelling\\data\\y1.json'
    with open(y1_save, 'a') as obj:
        obj.write('\n' + str(y1[-1]))

    phit_save = 'D:\\Wave glider modelling\\data\\phit.json'
    with open(phit_save, 'a') as obj:
        obj.write('\n' + str(phit[-1]))

    t_save = 'D:\\Wave glider modelling\\data\\time.json'
    with open(t_save, 'a') as obj:
        obj.write('\n' + str(t))

    if u1:
        u1_save = 'D:\\Wave glider modelling\\data\\u1.json'
        with open(u1_save, 'a') as obj:
            obj.write('\n' + str(u1[-1]))

    if T:
        T_save = 'D:\\Wave glider modelling\\data\\T.json'
        with open(T_save, 'a') as obj:
            obj.write('\n' + str(T[-1]))

    if z1:
        z1_save = 'D:\\Wave glider modelling\\data\\z1.json'
        with open(z1_save, 'a') as obj:
            obj.write('\n' + str(z1[-1]))

    if x2:
        x2_save = 'D:\\Wave glider modelling\\data\\x2.json'
        with open(x2_save, 'a') as obj:
            obj.write('\n' + str(x2[-1]))

    if y2:
        y2_save = 'D:\\Wave glider modelling\\data\\y2.json'
        with open(y2_save, 'a') as obj:
            obj.write('\n' + str(y2[-1]))

    if z2:
        z2_save = 'D:\\Wave glider modelling\\data\\z2.json'
        with open(z2_save, 'a') as obj:
            obj.write('\n' + str(z2[-1]))

    if rudder_angle:
        rudder_angle_save = 'D:\\Wave glider modelling\\data\\rudder_angle.json'
        with open(rudder_angle_save, 'a') as obj:
            obj.write('\n' + str(rudder_angle[-1]))

# clear previous stored data.
def data_elimation():
    open('D:\\Wave glider modelling\\data\\x1.json', 'w').close()
    open('D:\\Wave glider modelling\\data\\y1.json', 'w').close()
    open('D:\\Wave glider modelling\\data\\phit.json', 'w').close()
    open('D:\\Wave glider modelling\\data\\time.json', 'w').close()
    open('D:\\Wave glider modelling\\data\\u1.json', 'w').close()
    open('D:\\Wave glider modelling\\data\\T.json', 'w').close()
    open('D:\\Wave glider modelling\\data\\z1.json', 'w').close()
    open('D:\\Wave glider modelling\\data\\x2.json', 'w').close()
    open('D:\\Wave glider modelling\\data\\y2.json', 'w').close()
    open('D:\\Wave glider modelling\\data\\z2.json', 'w').close()
    open('D:\\Wave glider modelling\\data\\rudder_angle.json', 'w').close()
    os.remove(r'D:\\Wave glider modelling\\data\\x1.json')
    os.remove(r'D:\\Wave glider modelling\\data\\y1.json')
    os.remove(r'D:\\Wave glider modelling\\data\\phit.json')
    os.remove(r'D:\\Wave glider modelling\\data\\time.json')
    os.remove(r'D:\\Wave glider modelling\\data\\u1.json')
    os.remove(r'D:\\Wave glider modelling\\data\\T.json')
    os.remove(r'D:\\Wave glider modelling\\data\\z1.json')
    os.remove(r'D:\\Wave glider modelling\\data\\x2.json')
    os.remove(r'D:\\Wave glider modelling\\data\\y2.json')
    os.remove(r'D:\\Wave glider modelling\\data\\z2.json')
    os.remove(r'D:\\Wave glider modelling\\data\\rudder_angle.json')
