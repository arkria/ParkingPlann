from math import  sqrt, inf
import KDtreeKnn.kdtree as kdtree, KDtreeKnn.Distance as ds
import queue
import numpy as np

VEHICLE_RADIUS = 0.5
GRID_RESOLUTION = 1.0


class Node(object):
    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind


def calc_dist_policy(gx, gy, ox, oy, reso, vr):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    vr: vehicle radius[m]
    """

    ngoal = Node(round(gx/reso),round(gy/reso),0.0, -1)

    ox = [iox/reso for iox in ox]
    oy = [ioy/reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, vr)

    #open, closed set
    open, closed = {}, {}
    open[calc_index(ngoal, xw, minx, miny)] = ngoal

    motion = get_motion_model()
    nmotion = len(motion)
    pq = queue.PriorityQueue()
    pq.put((ngoal.cost, calc_index(ngoal, xw, minx, miny)))
    while True:
        if len(open) == 0:
            break

        c_v, c_id = pq.get()
        current = open[c_id]

        open.pop(c_id)
        closed[c_id] = current

        for i in range(nmotion): # expand search grid based on motion model
            node = Node(current.x+motion[i][0], current.y+motion[i][1], current.cost+motion[i][2], c_id)
            if not verify_node(node, minx, miny, xw, yw, obmap):
                continue

            node_ind = calc_index(node, xw, minx, miny)

            # If it is already in the closed set, skip it
            if node_ind in closed.keys():
                continue

            if node_ind in open.keys():
                if open[node_ind].cost > node.cost:
                    # If so, update the node to have a new parent
                    open[node_ind].cost = node.cost
                    open[node_ind].pind = c_id
            else: # add to open set
                open[node_ind] = node
                pq.put((node.cost, calc_index(node, xw, minx, miny)))

    pmap = calc_policy_map(closed, xw, yw, minx, miny)

    return pmap

def calc_policy_map(closed, xw, yw, minx, miny):

    pmap = np.full((xw+1, yw+1), inf)
    for n in closed.values():
        pmap[n.x-minx, n.y-miny] = n.cost

    return pmap


def calc_astar_path(sx, sy, gx, gy, ox, oy, reso, vr):
    """
    sx: start x position [m]
    sy: start y position [m]
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    """

    nstart = Node(round(sx/reso), round(sy/reso), 0.0, -1)
    ngoal = Node(round(gx/reso), round(gy/reso), 0.0, -1)

    ox = [iox/reso for iox in ox]
    oy = [ioy/reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, vr)

    #open, closed set
    open, closed = {}, {}
    open[calc_index(nstart, xw, minx, miny)] = nstart

    motion = get_motion_model()
    nmotion = len(motion)
    pq = queue.PriorityQueue()
    pq.put((calc_cost(nstart, ngoal), calc_index(nstart, xw, minx, miny)))

    while True:
        if len(open) == 0:
            print("Error: No open set")
            break

        c_v, c_id = pq.get()
        current = open[c_id]

        if current.x == ngoal.x and current.y == ngoal.y: # check goal
            # println("Goal!!")
            closed[c_id] = current
            break
        open.pop(c_id)
        closed[c_id] = current

        for i in range(nmotion): # expand search grid based on motion model
            node = Node(current.x+motion[i][0], current.y+motion[i][1], current.cost+motion[i][2], c_id)

            if not verify_node(node, minx, miny, xw, yw, obmap):
                continue

            node_ind = calc_index(node, xw, minx, miny)

            # If it is already in the closed set, skip it
            if node_ind in closed.keys():
                continue

            if node_ind in open.keys():
                if open[node_ind].cost > node.cost:
                    # If so, update the node to have a new parent
                    open[node_ind].cost = node.cost
                    open[node_ind].pind = c_id

            else: # add to open set
                open[node_ind] = node
                showx, showy, showcost = node.x, node.y, node.cost
                cost = calc_cost(node, ngoal)
                pq.put((calc_cost(node, ngoal), calc_index(node, xw, minx, miny)))

    rx, ry = get_final_path(closed, ngoal, nstart, xw, minx, miny, reso)

    return rx, ry


def verify_node(node, minx, miny, xw, yw, obmap):

    if (node.x - minx) >= xw:
        return False
    elif (node.x - minx) <= 0:
        return False

    if (node.y - miny) >= yw:
        return False
    elif (node.y - miny) <= 0:
        return False

    #collision check
    if obmap[node.x-minx, node.y-miny]:
        return False

    return True


def calc_cost(n, ngoal):
    return (n.cost + h(n.x - ngoal.x, n.y - ngoal.y))


def get_motion_model():
    # dx, dy, cost
    motion=[[1, 0, 1],[0, 1, 1], [-1, 0, 1], [0, -1, 1], [-1, -1, sqrt(2)], [-1, 1, sqrt(2)], [1, -1, sqrt(2)], [1, 1, sqrt(2)]]
    return motion


def calc_obstacle_map(ox, oy, reso, vr):

    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))

    xwidth = round(maxx - minx)
    ywidth = round(maxy - miny)

    obmap = np.full((xwidth+1, ywidth+1), False)
    data = []
    for i in zip(ox,oy):
        data.append({0: i[0], 1: i[1]})
    root = kdtree.create(data, dimensions=2)
    for ix in range(xwidth+1):
        x = ix + minx
        for iy in range(ywidth+1):
            y = iy + miny
            f = ds.EuclideanDistance
            ans = root.search_knn(point={0: x, 1: y}, k=1, dist=f)
            onedist = ans[0][1]
            if onedist <= vr/reso:
                obmap[ix, iy] = True

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth


def get_final_path(closed, ngoal, nstart, xw, minx, miny, reso):
    rx, ry = [ngoal.x], [ngoal.y]
    nid = calc_index(ngoal, xw, minx, miny)
    while True:
        n = closed[nid]
        rx.append(n.x)
        ry.append(n.y)
        nid = n.pind

        if rx[-1] == nstart.x and ry[-1] == nstart.y:
            break
    rx.reverse()
    ry.reverse()
    rx = [i*reso for i in rx]
    ry = [i*reso for i in ry]

    return rx, ry


def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin)*xwidth + (node.x - xmin)


def search_min_cost_node(open, ngoal):
    mnode = None
    mcost = inf
    for n in open.values():
        cost = n.cost + h(n.x - ngoal.x, n.y - ngoal.y)
        if mcost > cost:
            mnode = n
            mcost = cost
    return mnode

def h(x, y):
    return sqrt(x**2 + y**2)