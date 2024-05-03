import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

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
    bezier.addPoint(0.83775,0)
    bezier.addPoint(-0.83775,0)
    print(bezier.x_pos)
    t = 0
    x_list = []
    y_list = []
    for i in range(100):
    #     walker.step(0.005)
        t += 0.01
        x,y = bezier.getPos(t)
        print(bezier.x_pos)
        x_list.append(x)
        y_list.append(y)




    fig = plt.figure()
    def update(i):
        plt.clf()
        plt.ylim((-1,1))
        plt.xlim((-1,1))
        plt.plot(x_list,y_list)
        plt.plot(x_list[i],y_list[i],'o')

    ani = animation.FuncAnimation(fig,update,frames=100,interval=1)
    plt.show()

