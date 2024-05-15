from bezier import Bezier2
import matplotlib.pyplot as plt
import numpy as np

x_vec = np.array([-0.017109184265137103, 0.0014454078674314488, 0.02])
y_vec = np.array([0, 0.02, 0])

bezier = Bezier2()
bezier.setPoint(x_vec,y_vec)

if __name__ == '__main__':
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

    print('y_list: ', y_list)

    plt.figure()
    plt.plot(x_list)
    plt.show()
