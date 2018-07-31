from BasicCurve import hybrid_a_star
import math
import matplotlib.pyplot as plt

if __name__ == '__main__':
    sx, sy, syaw = 20.0, 20.0, math.radians(90.0)
    gx, gy, gyaw = 180.0, 100.0, math.radians(-90.0)
    ox, oy = [], []

    for i in range(201):
        ox.append(float(i))
        oy.append(0.0)

    for i in range(121):
        ox.append(200.0)
        oy.append(float(i))

    for i in range(201):
        ox.append(float(i))
        oy.append(120.0)

    for i in range(121):
        ox.append(0.0)
        oy.append(float(i))

    for i in range(81):
        ox.append(40.0)
        oy.append(float(i))

    for i in range(81):
        ox.append(80.0)
        oy.append(120 - float(i))

    for i in range(41):
        ox.append(120.0)
        oy.append(120 - float(i))
        ox.append(120.0)
        oy.append(float(i))

    for i in range(81):
        ox.append(160.0)
        oy.append(120 - float(i))

    rx, ry, ryaw = hybrid_a_star.calc_hybrid_astar_path(sx, sy, syaw, gx, gy, gyaw, ox, oy,
                                                        hybrid_a_star.XY_GRID_RESOLUTION,
                                                        hybrid_a_star.YAW_GRID_RESOLUTION,
                                                        hybrid_a_star.OB_MAP_RESOLUTION)

    plt.plot(ox, oy, '.k', label = 'obstacles')
    if rx != None:
        plt.plot(rx, ry, '-r', label = "Hybrid A* path")

    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()