import unittest, my_math

class SquareTest(unittest.TestCase):

    def testIntegeers(self):
        for x in range(-10, 10):
            self.check(x)

    def check(self, x):
        p = my_math.square(x)
        self.failUnless(p == x**2, 'Integer square failed')

if __name__ == '__main__':
    print(my_math.glo)
    unittest.main()