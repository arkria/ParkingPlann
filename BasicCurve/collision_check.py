import KDtreeKnn.kdtree as kdtree, KDtreeKnn.Distance as ds
from math import  sin, cos, atan2, hypot, acos, pi


B = 1.0 #[m] distance from rear to vehicle back end
C = 3.7 #[m] distance from rear to vehicle front end
I = 2.0 #[m] width of vehicle
WBUBBLE_DIST = (B+C)/2.0-B #[m] distance from rear and the center of whole bubble
WBUBBLE_R = (B+C)/2.0 #[m] whole bubble radius

vrx = [C, C, -B, -B, C ]
vry = [-I/2.0, I/2.0, I/2.0, -I/2.0, -I/2.0]

def check_collision(x, y, yaw, kdtreeroot, ox, oy):

    for (ix, iy, iyaw) in zip(x, y, yaw):
        cx = ix + WBUBBLE_DIST*cos(iyaw)
        cy = iy + WBUBBLE_DIST*sin(iyaw)
        # print(cx,cy)
        # Whole bubble check
        f = ds.EuclideanDistance
        rangepoint = kdtreeroot.search_inrange(point = {0:cx ,1:cy}, r = WBUBBLE_R, dist = f)
        tox = [dx[0].data[0] for dx in rangepoint]
        toy = [dy[0].data[1] for dy in rangepoint]
        if len(tox) == 0:
            continue

        if not rect_check(ix, iy, iyaw, tox, toy):
            return False #collision
    return True #OK


def rect_check(ix, iy, iyaw, ox, oy):
    c = cos(-iyaw)
    s = sin(-iyaw)

    for (iox, ioy) in zip(ox, oy):
        tx = iox - ix
        ty = ioy - iy
        lx = (c * tx - s * ty)
        ly = (s * tx + c * ty)

        sumangle = 0.0
        for i in range(len(vrx) - 1):
            x1 = vrx[i] - lx
            y1 = vry[i] - ly
            x2 = vrx[i + 1] - lx
            y2 = vry[i + 1] - ly
            d1 = hypot(x1, y1)
            d2 = hypot(x2, y2)
            theta1 = atan2(y1, x1)
            tty = (-sin(theta1) * x2 + cos(theta1) * y2) # 通过向量积判断是否同向

            tmp = (x1 * x2 + y1 * y2) / (d1 * d2)
            if tmp >= 1.0: tmp = 1.0

            if tty >= 0.0:
                sumangle += acos(tmp)
            else:
                sumangle -= acos(tmp)

        if sumangle >= pi:
            return False
    return True  # OK


