from BasicCurve import reeds_shepp
from BasicCurve import a_star
from BasicCurve import collision_check

from KDtreeKnn import kdtree, Distance as ds
from math import radians, sin, cos, tan, sqrt, pi
import numpy as np
import queue

from dill import dump
from dill import load

VEHICLE_RADIUS = 1.0  # [m]; radius of rear ball; 7.0 
BUBBLE_DIST = 1.7  # [m]; distance to "forward bubble"; 7.0
##### Fast Comp Time values from Alex Liniger ######
OB_MAP_RESOLUTION = 0.1  # [m]; obstacle resolution
YAW_GRID_RESOLUTION = radians(5.0)  # [m]; 10.0 /// 5.0
N_STEER = 5.0  # number of steer command; 10.0 seems OK /// 5
## For Backwards Parking
#   XY_GRID_RESOLUTION = 1. #[m];
#   MOTION_RESOLUTION = 0.3 #[m];
## For Parallel Parking
XY_GRID_RESOLUTION = 0.3  # [m];
MOTION_RESOLUTION = 0.1  # [m];
###################################################
USE_HOLONOMIC_WITH_OBSTACLE_HEURISTIC = True
USE_NONHOLONOMIC_WITHOUT_OBSTACLE_HEURISTIC = False
SB_COST = 10.0  # switch back penalty cost
BACK_COST = 0.0  # backward penalty cost
STEER_CHANGE_COST = 10.0  # steer angle change penalty cost
STEER_COST = 0.0  # steer angle  penalty cost
H_COST = 1.  # Heuristic cost; higher -> heuristic; 1.0
WB = 2.7  # [m]; 7.0
MAX_STEER = 0.6  # deg2rad(35.0) #[rad]

class Node(object):
    def __init__(self, xind, yind, yawind, direction, x, y, yaw, steer, cost, pind):
        self.xind = xind
        self.yind = yind
        self.yawind = yawind
        self.direction  = direction
        self.x, self.y, self.yaw = x, y, yaw
        self.steer = steer
        self.cost = cost
        self.pind = pind

class Config(object):
    def __init__(self, minx, miny, minyaw, maxx, maxy, maxyaw,
                 xw, yw, yaww, xyreso, yawreso,
                 obminx, obminy, obmaxx, obmaxy, obxw, obyw, obreso):
        self.minx, self.miny, self.minyaw = minx, miny, minyaw
        self.maxx, self.maxy, self.maxyaw = maxx, maxy, maxyaw
        self.xw, self.yw, self.yaww = xw, yw, yaww
        self.xyreso, self.yawreso = xyreso, yawreso
        self.obminx, self.obminy, self.obmaxx = obminx, obminy, obmaxx
        self.obmaxy, self.obxw, self.obyw, self.obreso = obmaxy, obxw, obyw, obreso


def calc_hybrid_astar_path(sx, sy, syaw, gx, gy, gyaw, ox, oy, xyreso, yawreso, obreso):
    """
    Calc hybrid astar path
    sx: start x position [m]
    sy: start y position [m]
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    xyreso: grid resolution [m]
    yawreso: yaw angle resolution [rad]
    """

    syaw, gyaw = reeds_shepp.pi_2_pi(syaw), reeds_shepp.pi_2_pi(gyaw)

    c = calc_config(ox, oy, xyreso, yawreso, obreso)
    data = []
    for i in zip(ox, oy):
        data.append({0: i[0], 1: i[1]})
    root = kdtree.create(data, dimensions=2)

    obmap, gkdtree = calc_obstacle_map(ox, oy, c)
    # obmap = loaddata('obmap.pkl')
    # gkdtree = loaddata('gkdtree.pkl')

    nstart = Node(round(sx/xyreso), round(sy/xyreso), round(syaw/yawreso), True, [sx], [sy], [syaw], 0.0, 0.0, -1)
    ngoal = Node(round(gx/xyreso), round(gy/xyreso), round(gyaw/yawreso), True, [gx], [gy], [gyaw], 0.0, 0.0, -1)

    if USE_HOLONOMIC_WITH_OBSTACLE_HEURISTIC:
        h_dp = calc_holonomic_with_obstacle_heuristic(ngoal, ox, oy, xyreso)
        # h_dp = loaddata('h_dp.pkl')
    else:
        h_dp = []

    if USE_NONHOLONOMIC_WITHOUT_OBSTACLE_HEURISTIC:
        h_rs = calc_nonholonomic_without_obstacle_heuristic(ngoal, c)
    else:
        h_rs = []

    open, closed = {}, {}
    open[calc_index(nstart, c)] = nstart
    pq = queue.PriorityQueue()
    pq.put((calc_cost(nstart, h_rs, h_dp, ngoal, c), calc_index(nstart, c)))

    u, d = calc_motion_inputs()
    nmotion = len(u)

    while True:
        if len(open) == 0:
            print("Error: Cannot find path, No open set")
            return None, None, None

        c_v, c_id = pq.get()
        current = open[c_id]

        isupdated, current = update_node_with_analystic_expantion(current, ngoal, obmap, c, root, ox, oy)

        if isupdated:
            closed[calc_index(ngoal, c)] = current
            break #goal

        #move current node from open to closed
        open.pop(c_id)
        closed[c_id] = current

        for i in range(nmotion):
            node = calc_next_node(current, c_id, u[i], d[i], c, gkdtree)

            if not verify_index(node, obmap, c, root, ox, oy):
                continue

            node_ind = calc_index(node, c)

            # If it is already in the closed set, skip it
            if node_ind in closed.keys():
                continue

            if node_ind not in open.keys():
                open[node_ind] = node
                pq.put((calc_cost(node, h_rs, h_dp, ngoal, c), calc_index(node, c)))

    # closed = loaddata('closed.pkl')
    rx, ry, ryaw = get_final_path(closed, ngoal, nstart, c)

    return rx, ry, ryaw


def update_node_with_analystic_expantion(current, ngoal, obmap, c, kdtree, ox, oy):

    apath = analystic_expantion(current, ngoal, obmap, c, kdtree, ox, oy)
    if apath != None:
        # println("Find path! with analystic_expantion")
        current.x = current.x + apath.x[1:-2]
        current.y = current.y + apath.y[1:-2]
        current.yaw = current.yaw + apath.yaw[1:-2]
        current.cost += calc_rs_path_cost(apath)
        return True, current
    return False, current #no update


def calc_rs_path_cost(rspath):

    cost = 0.0
    for l in rspath.lengths:
        if l >= 0:  # forward
            cost += l
        else:  # back
            cost += abs(l) * BACK_COST

    # swich back penalty
    for i in range(len(rspath.lengths) - 1):
        if rspath.lengths[i] * rspath.lengths[i + 1] < 0.0:  # switch back
            cost += SB_COST

    # steer penalyty
    for ctype in rspath.ctypes:
        if ctype != "S":  # curve
            cost += STEER_COST * abs(MAX_STEER)

    # ==steer change penalty
    # calc steer profile
    nctypes = len(rspath.ctypes)
    ulist = [0.0 for _ in range(nctypes)]
    for i in range(nctypes):
        if rspath.ctypes[i] == "R":
            ulist[i] = - MAX_STEER
        elif rspath.ctypes[i] == "L":
            ulist[i] = MAX_STEER

    for i in range(len(rspath.ctypes) - 1):
        cost += STEER_CHANGE_COST * abs(ulist[i + 1] - ulist[i])

    # println("RS cost is ", cost)
    return cost



def analystic_expantion(n, ngoal, obmap, c, kdtree, ox, oy):

    sx = n.x[-1]
    sy = n.y[-1]
    syaw = n.yaw[-1]

    max_curvature = tan(MAX_STEER)/WB
    path = reeds_shepp.calc_shortest_path(sx, sy, syaw,
                                           ngoal.x[-1], ngoal.y[-1], ngoal.yaw[-1],
                                           max_curvature, step_size=MOTION_RESOLUTION)

    if path == None:
        return None

    if not collision_check.check_collision(path.x, path.y, path.yaw, kdtree, ox, oy):
        return None

    # println(paths)
    return path # find good path



def calc_motion_inputs():

    up = [i for i in np.arange(MAX_STEER/N_STEER, MAX_STEER+MAX_STEER/N_STEER, MAX_STEER/N_STEER)]
    u = [0.0] + [i for i in up] + [-i for i in up]
    d = [1.0 for _ in range(len(u))] + [-1.0 for _ in range(len(u))]
    u = u + u

    return u, d


def verify_index(node, obmap, c, kdtree, ox, oy):

    # overflow map
    if (node.xind - c.minx) >= c.xw:
        return False
    elif (node.xind - c.minx) <= 0:
        return False

    if (node.yind - c.miny) >= c.yw:
        return False
    elif (node.yind - c.miny) <= 0:
        return False


    # check collisiton
    # rectangle check
    if not collision_check.check_collision(node.x, node.y,node.yaw, kdtree, ox, oy):
        return False

    return True #index is ok"


def calc_next_node(current, c_id, u, d, c, gkdtree):

    arc_l = XY_GRID_RESOLUTION

    nlist = round(arc_l / MOTION_RESOLUTION) + 1
    xlist = [0.0 for _ in range(nlist)]
    ylist = [0.0 for _ in range(nlist)]
    yawlist = [0.0 for _ in range(nlist)]
    xlist[0] = current.x[-1] + d * MOTION_RESOLUTION * cos(current.yaw[-1])
    ylist[0] = current.y[-1] + d * MOTION_RESOLUTION * sin(current.yaw[-1])
    yawlist[0] = reeds_shepp.pi_2_pi(current.yaw[-1] + d * MOTION_RESOLUTION / WB * tan(u))

    for i in range(nlist-1):
        xlist[i + 1] = xlist[i] + d * MOTION_RESOLUTION * cos(yawlist[i])
        ylist[i + 1] = ylist[i] + d * MOTION_RESOLUTION * sin(yawlist[i])
        yawlist[i + 1] = reeds_shepp.pi_2_pi(yawlist[i] + d * MOTION_RESOLUTION / WB * tan(u))

    xind = round(xlist[-1] / c.xyreso)
    yind = round(ylist[-1] / c.xyreso)
    yawind = round(yawlist[-1] / c.yawreso)

    addedcost = 0.0
    if d > 0:
        direction = True
        addedcost += abs(arc_l)
    else:
        direction = False
        addedcost += abs(arc_l) * BACK_COST

    # swich back penalty
    if direction != current.direction:  # switch back penalty
        addedcost += SB_COST

    # steer penalyty
    addedcost += STEER_COST * abs(u)

    # steer change penalty
    addedcost += STEER_CHANGE_COST * abs(current.steer - u)

    cost = current.cost + addedcost
    node = Node(xind, yind, yawind, direction, xlist, ylist, yawlist, u, cost, c_id)
    # println(node)

    return node



def calc_holonomic_with_obstacle_heuristic(gnode, ox, oy, xyreso):
    # println("Calc distance policy")
    h_dp = a_star.calc_dist_policy(gnode.x[-1], gnode.y[-1], ox, oy, xyreso, VEHICLE_RADIUS)
    out = open('h_dp.pkl', 'wb')
    # Pickle the knn_model using the highest protocol available.
    dump(h_dp, out, -1)
    out.close()
    return h_dp


def calc_nonholonomic_without_obstacle_heuristic(ngoal, c):

    h_rs = np.full((c.xw,c.yw,c.yaww), 0.0)
    max_curvature = tan(MAX_STEER)/WB

    for ix in range(c.xw):
        for iy in range(c.yw):
            for iyaw in range(c.yaww):
                sx = (ix + c.minx)*c.xyreso
                sy = (iy + c.miny)*c.xyreso
                syaw = reeds_shepp.pi_2_pi((iyaw + c.minyaw)*c.yawreso)
                L = reeds_shepp.calc_shortest_path_length(sx, sy, syaw,
                    ngoal.x[-1], ngoal.y[-1], ngoal.yaw[-1],
                    max_curvature, step_size=MOTION_RESOLUTION)
                h_rs[ix, iy, iyaw] = L

    return h_rs


def calc_config(ox, oy, xyreso, yawreso, obreso):
    minx = int(round(min(ox)/xyreso))
    miny = int(round(min(oy)/xyreso))
    maxx = int(round(max(ox)/xyreso))
    maxy = int(round(max(oy)/xyreso))
    obminx = int(round(min(ox)/obreso))
    obminy = int(round(min(oy)/obreso))
    obmaxx = int(round(max(ox)/obreso))
    obmaxy = int(round(max(oy)/obreso))

    xw = int(round((maxx - minx)))
    yw = int(round((maxy - miny)))
    obxw = int(round((obmaxx - obminx)))
    obyw = int(round((obmaxy - obminy)))

    minyaw = int(round(- pi/yawreso) - 1)
    maxyaw = int(round( pi/yawreso))
    yaww = int(round((maxyaw - minyaw)))

    config = Config(minx, miny, minyaw, maxx, maxy, maxyaw, xw, yw, yaww,
                    xyreso, yawreso, obminx, obminy, obmaxx, obmaxy, obxw, obyw, obreso)

    return config


def calc_obstacle_map(ox, oy, c):

    ox = [iox/c.obreso for iox in ox]
    oy = [ioy/c.obreso for ioy in oy]

    obmap = np.full((c.obxw+1, c.obyw+1), False)

    data = []
    for i in zip(ox, oy):
        data.append({0: i[0], 1: i[1]})
    root = kdtree.create(data, dimensions=2)

    for ix in range(c.obxw+1):
        x = ix + c.obminx
        for iy in range(c.obyw+1):

            y = iy + c.obminy
            # print(x,y)
            f = ds.EuclideanDistance
            ans = root.search_knn(point={0: x, 1: y}, k=1, dist=f)
            onedist = ans[0][1]

            if onedist <= VEHICLE_RADIUS/c.obreso:
                obmap[ix,iy] = True
    out = open('obmap.pkl', 'wb')
    # Pickle the knn_model using the highest protocol available.
    dump(obmap, out, -1)
    out.close()

    out1 = open('gkdtree.pkl', 'wb')
    # Pickle the knn_model using the highest protocol available.
    dump(root, out1, -1)
    out1.close()

    return obmap, root



def get_final_path(closed, ngoal, nstart, c):

    rx, ry, ryaw = ngoal.x, ngoal.y, ngoal.yaw
    nid = calc_index(ngoal, c)
    # println("Fianl cost is ", closed[nid].cost)
    while True:
        n = closed[nid]
        tx, ty, tyaw = n.x.copy(), n.y.copy(), n.yaw.copy()
        tx.reverse()
        ty.reverse()
        tyaw.reverse()
        rx = rx+tx
        ry = ry+ty
        ryaw = ryaw+tyaw
        nid = n.pind
        if is_same_grid(n, nstart):
            # println("done")
            break

    rx.reverse()
    ry.reverse()
    ryaw.reverse()

    dist = sum([sqrt(idx**2+idy**2) for (idx,idy) in zip(np.diff(rx), np.diff(ry))])
    # println("Fianl path distance is ", dist)

    return rx, ry, ryaw


def calc_index(node, c):
    ind = (node.yawind - c.minyaw)*c.xw*c.yw+(node.yind - c.miny)*c.xw + (node.xind - c.minx)
    if ind <= 0:
        print("Error(calc_index):", ind)

    return ind



def is_same_grid(node1, node2):

    if node1.xind != node2.xind:
        return False

    if node1.yind != node2.yind:
        return False

    if node1.yawind != node2.yawind:
        return False

    return True


def calc_cost(n, h_rs, h_dp, ngoal, c):

    if len(h_rs) > 1 and len(h_dp) > 1:  # Both heuristic cost are activated
        c_h_dp = h_dp[n.xind - c.minx, n.yind - c.miny]
        c_h_rs = h_rs[n.xind - c.minx, n.yind - c.miny, n.yawind - c.minyaw]
        return (n.cost + H_COST*max(c_h_dp, c_h_rs))
    elif len(h_dp) > 1: # Distance policy heuristics is activated
        return (n.cost + H_COST*h_dp[n.xind - c.minx, n.yind - c.miny])
    elif len(h_rs) > 1: # Reed Sheep path heuristics is activated
        return (n.cost + H_COST*h_rs[n.xind - c.minx, n.yind - c.miny, n.yawind - c.minyaw])

    return (n.cost + H_COST*calc_euclid_dist(n.x[-1] - ngoal.x[-1],n.y[-1] - ngoal.y[-1], n.yaw[-1] - ngoal.yaw[-1]))


def calc_euclid_dist( x, y, yaw):
    """
    Heuristic cost function
    """
    if yaw >= pi:
        yaw -= pi
    elif yaw <= -pi:
        yaw += pi

    return sqrt(x**2+y**2+yaw**2)

def savedata(mydata, outfile):
    out = open(outfile, 'wb')
    # Pickle the knn_model using the highest protocol available.
    dump(mydata, out,-1)
    out.close()


def loaddata(srcfile):
    src = open(srcfile, 'rb')
    mydata = load(src)
    src.close()
    return mydata