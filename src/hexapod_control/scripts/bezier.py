import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from utils import get_phase_list

class Bezier(object):
    def __init__(self):
        self.x_pos = []
        self.y_pos = []


    def addPoint(self, x, y):
        self.x_pos.append(x)
        self.y_pos.append(y)


    def getPos(self, t):

        n_points = len(self.x_pos)

        #prevent the variables allocates at the same location
        x_cal, y_cal = [], []
        for x in self.x_pos: x_cal.append(x)
        for y in self.y_pos: y_cal.append(y)
       

        for i in range(n_points-1):
            for j in range(n_points-i-1):
               x_cal[j] = (1.0-t)*x_cal[j] + t*x_cal[j+1]
               y_cal[j] = (1.0-t)*y_cal[j] + t*y_cal[j+1]

        x_ret = x_cal[0]
        y_ret = y_cal[0]
        return x_ret, y_ret


if __name__ == "__main__":
    bezier = Bezier()
    bezier.addPoint(-1,0)
    bezier.addPoint(-1,1.6)
    # bezier.addPoint(-1.39626,0)
    bezier.addPoint(0,3.2)
    # bezier.addPoint(1.39626,0)
    bezier.addPoint(1,1.6)
    bezier.addPoint(1,0)

    print(bezier.x_pos)

    bezier2 = Bezier()
    bezier2.addPoint(1,0)
    bezier2.addPoint(-1,0)

    bezier3 = Bezier()
    bezier3.addPoint(0,0)
    bezier3.addPoint(0.5,0)
    bezier3.addPoint(0.5,1)
    bezier3.addPoint(1,1)
    
    t = 0
    x_list = []
    y_list = []
    for i in range(100):
    #     walker.step(0.005)
        t += 0.01
        x,y = bezier.getPos(t)
        # print(bezier.x_pos)
        x_list.append(x)
        y_list.append(y)

    # t = 0
    # x_list2 = []
    # y_list2 = []

    # for i in range(1000):
    # #     walker.step(0.005)
    #     t += 0.001
    #     x,y = bezier2.getPos(t)
    #     # print(bezier2.x_pos)
    #     x_list2.append(x)
    #     y_list2.append(y)

    # print(x_list2)

    x_list3 = []
    y_list3 = []
    t = 0

    for i in range(1500):
    #     walker.step(0.005)
        t += 0.001
        x,y = bezier3.getPos(t)
        # print(bezier2.x_pos)
        x_list3.append(x)
        y_list3.append(y)

    print(x_list3)


    plt.figure()
    plt.plot(x_list,y_list)
    # plt.plot(x_list,y_list)
    plt.show()

