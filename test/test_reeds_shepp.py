import unittest, math, random, matplotlib.pyplot as plt
from BasicCurve import reeds_shepp, a_star


class reeds_sheppTest(unittest.TestCase):

    def testSpecial(self):

        start_x = 0.0  # [m]
        start_y = 0.0  # [m]
        start_yaw = math.radians(10.0)  # [rad]
        end_x = 7.0  # [m]
        end_y = -8.0  # [m]
        end_yaw = math.radians(50.0)  # [rad]
        max_curvature = 2.0
        self.check_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature)

        start_x = 0.0  # [m]
        start_y = 0.0  # [m]
        start_yaw = math.radians(10.0)  # [rad]
        end_x = 7.0  # [m]
        end_y = -8.0  # [m]
        end_yaw = math.radians(-50.0)  # [rad]
        max_curvature = 2.0
        self.check_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature)

        start_x = 0.0  # [m]
        start_y = 10.0  # [m]
        start_yaw = math.radians(-10.0)  # [rad]
        end_x = -7.0  # [m]
        end_y = -8.0  # [m]
        end_yaw = math.radians(-50.0)  # [rad]
        max_curvature = 2.0
        self.check_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature)

        start_x = 0.0  # [m]
        start_y = 10.0  # [m]
        start_yaw = math.radians(-10.0)  # [rad]
        end_x = -7.0  # [m]
        end_y = -8.0  # [m]
        end_yaw = math.radians(150.0)  # [rad]
        max_curvature = 1.0
        self.check_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature)

        start_x = 0.0  # [m]
        start_y = 10.0  # [m]
        start_yaw = math.radians(-10.0)  # [rad]
        end_x = 7.0  # [m]
        end_y = 8.0  # [m]
        end_yaw = math.radians(150.0)  # [rad]
        max_curvature = 2.0
        self.check_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature)

        start_x = -40.0  # [m]
        start_y = 549.0  # [m]
        start_yaw = 2.44346  # [rad]
        end_x = 36.0  # [m]
        end_y = 446.0  # [m]
        end_yaw = -0.698132
        max_curvature = 0.05890904077226434
        self.check_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature)

    def testRandom(self):
        for i in range(100):
            start_x = random.random()*100 - 50.0
            start_y = random.random()*100 - 50.0
            start_yaw = math.radians(random.random()*360 - 180.0)
            end_x, end_y, end_yaw = random.random()*100 - 50.0, random.random()*100 - 50.0, math.radians(random.random()*100 - 50.0)
            max_curvature = random.random()/10

            self.check_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature)

    def testOnecase():
        start_x = 3.0  # [m]
        start_y = 10.0  # [m]
        start_yaw = math.radians(0.0)  # [rad]
        end_x = 3.0  # [m]
        end_y = -20.0  # [m]
        end_yaw = math.radians(180.0)  # [rad]
        max_curvature = 0.1

        bpath = reeds_shepp.calc_shortest_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature)

        rc, rds = reeds_shepp.calc_curvature(bpath.x, bpath.y, bpath.yaw, bpath.directions)

        plt.subplot(211)
        plt.plot(bpath.x, bpath.y, '-r', label=reeds_shepp.get_label(bpath))

        plt.plot(start_x, start_y)
        plt.plot(end_x, end_y)

        plt.legend()
        plt.grid(True)
        plt.axis('equal')

        plt.subplot(212)
        plt.plot(rc, '.r', label='reeds shepp')
        plt.grid(True)
        plt.axis('equal')

        plt.show()

    def check_path(self, start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature):
        paths = reeds_shepp.calc_paths(start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature)

        self.failUnless(len(paths) >= 1, 'paths lengths failed')

        for path in paths:
            self.failUnless(abs(path.x[0] - start_x) <= 0.01, 'x start position failed')
            self.failUnless(abs(path.y[0] - start_y) <= 0.01, 'y start position failed')
            self.failUnless(abs(path.yaw[0] - start_yaw) <= 0.01, 'yaw start position failed')
            self.failUnless(abs(path.x[-1] - end_x) <= 0.01, 'x final position failed')
            self.failUnless(abs(path.y[-1] - end_y) <= 0.01, 'y final position failed')
            self.failUnless(abs(path.yaw[-1] - end_yaw) <= 0.01, 'yaw final position failed')

            d = [math.sqrt(dx**2 + dy**2) for (dx, dy) in zip([p-q for p,q in zip(path.x[1:-1], path.x[:-2])],[p-q for p,q in zip(path.y[1:-1], path.y[:-2])])]

            for i in range(len(d)):
                self.failUnless(abs(d[i] - reeds_shepp.STEP_SIZE) <= 0.001, 'step size failed')



if __name__ == '__main__':
    print('Start Test')
    reeds_sheppTest.testOnecase()


