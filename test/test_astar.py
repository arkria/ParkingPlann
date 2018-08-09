import unittest, math, random, matplotlib.pyplot as plt
from BasicCurve import reeds_shepp, a_star


class a_starTest(unittest.TestCase):

    def testOncase(self):
        sx,sy,gx,gy = 10.0, 10.0, 50.0, 50.0
        ox, oy = [], []

        for i in range(61):
            ox.append(float(i))
            oy.append(0.0)

        for i in range(61):
            ox.append(60.0)
            oy.append(float(i))

        for i in range(61):
            ox.append(float(i))
            oy.append(60.0)

        for i in range(61):
            ox.append(0.0)
            oy.append(float(i))

        for i in range(41):
            ox.append(2.0)
            oy.append(float(i))

        for i in range(41):
            ox.append(40.0)
            oy.append(60.0-float(i))

        rx, ry = a_star.calc_astar_path(sx, sy, gx, gy, ox, oy, a_star.GRID_RESOLUTION, a_star.VEHICLE_RADIUS)

        plt.plot(ox, oy, '.k', label = 'obstacle')
        plt.plot(sx, sy, 'xr', label = 'start')
        plt.plot(gx, gy, 'xb', label = 'goal')
        plt.plot(rx, ry, '-r', label = 'path')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')

        plt.show()

if __name__ == '__main__':
    print('Start Test')
    # unittest.main()
    # sx, sy, gx, gy = 2.0, 2.0, 8.0, 8.0
    # ox, oy = [], []
    #
    # for i in range(11):
    #     ox.append(float(i))
    #     oy.append(0.0)
    #
    # for i in range(11):
    #     ox.append(10.0)
    #     oy.append(float(i))
    #
    # for i in range(11):
    #     ox.append(float(i))
    #     oy.append(10.0)
    #
    # for i in range(11):
    #     ox.append(0.0)
    #     oy.append(float(i))
    #
    # for i in range(8):
    #     ox.append(4.0)
    #     oy.append(float(i))
    #
    # for i in range(5):
    #     ox.append(6.0)
    #     oy.append(10.0 - float(i))

    sx, sy, gx, gy = 10.0, 10.0, 50.0, 50.0
    ox, oy = [], []

    for i in range(61):
        ox.append(float(i))
        oy.append(0.0)

    for i in range(61):
        ox.append(60.0)
        oy.append(float(i))

    for i in range(61):
        ox.append(float(i))
        oy.append(60.0)

    for i in range(61):
        ox.append(0.0)
        oy.append(float(i))

    for i in range(41):
        ox.append(20.0)
        oy.append(float(i))

    for i in range(41):
        ox.append(40.0)
        oy.append(60.0 - float(i))

    rx, ry = a_star.calc_astar_path(sx, sy, gx, gy, ox, oy, a_star.GRID_RESOLUTION, a_star.VEHICLE_RADIUS)
    pmap = a_star.calc_dist_policy(gx, gy, ox, oy, a_star.GRID_RESOLUTION, a_star.VEHICLE_RADIUS)
    for i in zip(rx,ry):
        print(i)

    plt.plot(ox, oy, '.k', label='obstacle')
    plt.plot(sx, sy, 'xr', label='start')
    plt.plot(gx, gy, 'xb', label='goal')
    plt.plot(rx, ry, '-r', label='path')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')

    plt.show()