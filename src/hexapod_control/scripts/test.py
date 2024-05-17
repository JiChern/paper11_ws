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
    x = 10

    if x>1:
        print('first condition met')
    elif x>2:
        print('second condition met')
