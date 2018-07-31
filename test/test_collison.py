from  BasicCurve.collision_check import *
import numpy as np
from KDtreeKnn import kdtree
from math import radians
from matplotlib import pyplot as plt

if __name__ == '__main__':
    ox = np.random.rand(3) * 20.0 - 20.0 / 2.0
    oy = np.random.rand(3) * 20.0 - 20.0 / 2.0

    data = []
    for i in zip(ox, oy):
        data.append({0: i[0], 1: i[1]})
    root = kdtree.create(data, dimensions=2)

    x = [10.0, 5.0]
    y = [10.0, 5.0]
    yaw = [radians(10.0), radians(0.0)]

    flag = check_collision(x, y, yaw, root, ox, oy)
    if flag:
        print("OK")
    else:
        print("Collision")

    plt.plot(ox, oy, ".r")
    plt.grid(True)
    plt.axis("equal")
    plt.show()