from math import pi, sqrt, atan2, asin, acos, sin, cos, tan, isnan, inf


STEP_SIZE = 0.1


class Path(object):
    def __init__(self, lengths, ctypes, L, x, y, yaw, directions):
        self.lengths = lengths
        self.ctypes = ctypes
        self.L = L
        self.x = x; self.y = y; self.yaw = yaw
        self.directions = directions


def pi_2_pi(iangle):
    while iangle > pi:
        iangle -= 2.0 * pi
    while iangle < -pi:
        iangle += 2.0 * pi
    return iangle


def calc_shortest_path(sx, sy, syaw, gx, gy, gyaw, maxc, step_size = STEP_SIZE):
    paths = calc_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size=step_size)

    minL = inf
    best_path_index = -1
    for i in range(len(paths)):
        if paths[i].L <= minL:
            minL = paths[i].L
            best_path_index = i
    return paths[best_path_index]


def calc_shortest_path_length(sx, sy, syaw, gx, gy, gyaw, maxc, step_size = STEP_SIZE):
    q0 = [sx, sy, syaw]
    q1 = [gx, gy, gyaw]
    paths = generate_path(q0, q1, maxc)

    minL = inf
    for i in range(len(paths)):
        L = paths[i].L/maxc
        if L <= minL:
            minL = L
    return minL


def calc_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size = STEP_SIZE):
    q0 = [sx, sy, syaw]
    q1 = [gx, gy, gyaw]

    paths = generate_path(q0, q1, maxc)
    for path in paths:
        x, y, yaw, directions = generate_local_course(path.L, path.lengths, path.ctypes, maxc, step_size*maxc)

        path.x = [cos(-q0[2]) * ix + sin(-q0[2]) * iy + q0[0] for (ix, iy) in zip(x, y)]
        path.y = [-sin(-q0[2]) * ix + cos(-q0[2]) * iy + q0[1] for (ix, iy) in zip(x, y)]
        path.yaw = [pi_2_pi(iyaw + q0[2]) for iyaw in yaw]
        path.directions = directions
        path.lengths = [l/maxc for l in path.lengths]
        path.L = path.L/maxc
    return paths


def get_label(path):
    label = ""

    for (m,l) in zip(path.ctypes, path.lengths):
        label = label+m
        if l > 0.0:
            label = label + '+'
        else:
            label = label + '-'

    return label


def polar(x, y):
    r = sqrt(x**2+y**2)
    theta = atan2(y, x)
    return r, theta


def mod2pi(x): # 将弧度转到-pi~pi之间，注意这里pi与-pi是共存的，但是二者是等价的
    v = x % (2*pi)
    if v < -pi:
        v += 2.0 * pi
    else:
        if v > pi:
            v -= 2.0 * pi
    return v


def LSL(x, y, phi):
    u, t = polar(x - sin(phi), y - 1.0 + cos(phi))
    if t >= 0.0:
        v = mod2pi(phi - t)
        if v >= 0.0:
            return True, t, u, v
    return False, 0.0, 0.0, 0.0


def LSR(x, y, phi):
    u1, t1 = polar(x + sin(phi), y - 1.0 -cos(phi))
    u1 = u1**2
    if u1 >= 4.0:
        u = sqrt(u1 - 4.0)
        theta = atan2(2.0, u)
        t = mod2pi(t1 + theta)
        v = mod2pi(t - phi)

        if t >= 0.0 and v >= 0.0:
            return True, t, u, v
    return False, 0.0, 0.0, 0.0


def LRL(x, y, phi):
    u1, t1 = polar(x - sin(phi), y - 1.0 + cos(phi))

    if u1 <= 4.0:
        u = -2.0 * asin(0.25 * u1)
        t = mod2pi(t1 + 0.5 * u +pi)
        v = mod2pi(phi - t + u)

        if t >= 0.0 and u <= 0.0:
            return True, t, u, v
    return False, 0.0, 0.0, 0.0


def set_path(paths, lengths, ctypes):

    path = Path([],[],0.0,[],[],[],[])
    path.ctypes = ctypes
    path.lengths = lengths

    for tpath in paths:
        typeissame = (tpath.ctypes == path.ctypes)
        if typeissame:
            if sum([q-p for q,p in zip(tpath.lengths, path.lengths)]) <= 0.01:
                return paths

    path.L = sum([abs(i) for i in lengths])

    paths.append(path)

    return paths


def SCS(x, y, phi, paths):
    flag, t, u, v = SLS(x, y, phi)
    if flag:
       paths = set_path(paths, [t, u, v], ["S", "L", "S"])

    flag,t,u,v = SLS(x, -y, -phi)

    if flag:
        paths = set_path(paths, [t, u, v], ["S", "R", "S"])

    return paths


def SLS(x, y, phi):
    phi = mod2pi(phi)
    if y > 0.0 and 0.0 < phi < pi * 0.99:
        xd = -y/tan(phi) + x
        t  = xd - tan(phi/2.0)
        u = phi
        v = sqrt((x-xd)**2+y**2)-tan(phi/2.0)
        return True, t, u, v
    elif y < 0.0 and phi > 0.0 and phi < pi * 0.99:
        xd = -y/tan(phi) + x
        t = xd - tan(phi/2.0)
        u = phi
        v = -sqrt((x-xd)**2+y**2)-tan(phi/2.0)
        return True, t, u, v
    return False, 0.0, 0.0, 0.0

def CSC(x, y, phi, paths):
    flag, t, u, v = LSL(x, y, phi) 
    if flag:
        # println("CSC1")
        paths = set_path(paths, [t, u, v], ["L","S","L"])
    flag, t, u, v = LSL(-x, y, -phi) 
    if flag:
        # println("CSC2")
        paths = set_path(paths, [-t, -u, -v], ["L","S","L"])
    flag, t, u, v = LSL(x, -y, -phi) 
    if flag:
        # println("CSC3")
        paths = set_path(paths, [t, u, v], ["R","S","R"])
    flag, t, u, v = LSL(-x, -y, phi) 
    if flag:
        # println("CSC4")
        paths = set_path(paths, [-t, -u, -v], ["R","S","R"])

    flag, t, u, v = LSR(x, y, phi) 
    if flag:
        # println("CSC5")
        paths = set_path(paths, [t, u, v], ["L","S","R"])
    
    flag, t, u, v = LSR(-x, y, -phi) 
    if flag:
        # println("CSC6")
        paths = set_path(paths, [-t, -u, -v], ["L","S","R"])
    
    flag, t, u, v = LSR(x, -y, -phi) 
    if flag:
        # println("CSC7")
        paths = set_path(paths, [t, u, v], ["R","S","L"])
    
    flag, t, u, v = LSR(-x, -y, phi) 
    if flag:
        # println("CSC8")
        paths = set_path(paths, [-t, -u, -v], ["R","S","L"])
    
    return paths


def CCC(x, y, phi, paths):

    flag, t, u, v = LRL(x, y, phi) 
    if flag:
        # println("CCC1")
        paths = set_path(paths, [t, u, v], ["L","R","L"])
    
    flag, t, u, v = LRL(-x, y, -phi) 
    if flag:
        # println("CCC2")
        paths = set_path(paths, [-t, -u, -v], ["L","R","L"])
    
    flag, t, u, v = LRL(x, -y, -phi) 
    if flag:
        # println("CCC3")
        paths = set_path(paths, [t, u, v], ["R","L","R"])
    
    flag, t, u, v = LRL(-x, -y, phi) 
    if flag:
        # println("CCC4")
        paths = set_path(paths, [-t, -u, -v], ["R","L","R"])
    # backwards
    xb = x*cos(phi) + y*sin(phi)
    yb = x*sin(phi) - y*cos(phi)
    # println(xb, ",", yb,",",x,",",y)

    flag, t, u, v = LRL(xb, yb, phi)
    if flag:
        # println("CCC5")
        paths = set_path(paths, [v, u, t], ["L","R","L"])
    
    flag, t, u, v = LRL(-xb, yb, -phi)
    if flag:
        # println("CCC6")
        paths = set_path(paths, [-v, -u, -t], ["L","R","L"])
    
    flag, t, u, v = LRL(xb, -yb, -phi)
    if flag:
        # println("CCC7")
        paths = set_path(paths, [v, u, t], ["R","L","R"])
    
    flag, t, u, v = LRL(-xb, -yb, phi)
    if flag:
        # println("CCC8")
        paths = set_path(paths, [-v, -u, -t], ["R","L","R"])
    
    return paths


def calc_tauOmega(u, v, xi, eta, phi):
    delta = mod2pi(u-v)
    A = sin(u) - sin(delta)
    B = cos(u) - cos(delta) - 1.0

    t1 = atan2(eta*A - xi*B, xi*A + eta*B)
    t2 = 2.0 * (cos(delta) - cos(v) - cos(u)) + 3.0

    if t2 < 0:
        tau = mod2pi(t1+pi)
    else:
        tau = mod2pi(t1)
    
    omega = mod2pi(tau - u + v - phi)

    return tau, omega


def LRLRn(x, y, phi):
    xi = x + sin(phi)
    eta = y - 1.0 - cos(phi)
    rho = 0.25 * (2.0 + sqrt(xi*xi + eta*eta))

    if rho <= 1.0:
        u = acos(rho)
        t, v = calc_tauOmega(u, -u, xi, eta, phi)
        if t >= 0.0 and v <= 0.0:
            return True, t, u, v
        
    return False, 0.0, 0.0, 0.0


def LRLRp(x, y, phi):
    xi = x + sin(phi)
    eta = y - 1.0 - cos(phi)
    rho = (20.0 - xi*xi - eta*eta) / 16.0
    # println(xi,",",eta,",",rho)

    if 0.0 <= rho<=1.0:
        u = -acos(rho)
        if u >= -0.5 * pi:
            t, v = calc_tauOmega(u, u, xi, eta, phi)
            if t >= 0.0 and v >= 0.0:
                return True, t, u, v
    return False, 0.0, 0.0, 0.0


def CCCC(x, y, phi, paths):

    flag, t, u, v = LRLRn(x, y, phi) 
    if flag:
        # println("CCCC1")
        paths = set_path(paths, [t, u, -u, v], ["L","R","L","R"])
    flag, t, u, v = LRLRn(-x, y, -phi) 
    if flag:
        # println("CCCC2")
        paths = set_path(paths, [-t, -u, u, -v], ["L","R","L","R"])
 
    flag, t, u, v = LRLRn(x, -y, -phi) 
    if flag:
        # println("CCCC3")
        paths = set_path(paths, [t, u, -u, v], ["R","L","R","L"])
 
    flag, t, u, v = LRLRn(-x, -y, phi) 
    if flag:
        # println("CCCC4")
        paths = set_path(paths, [-t, -u, u, -v], ["R","L","R","L"])

    flag, t, u, v = LRLRp(x, y, phi) 
    if flag:
        # println("CCCC5")
        paths = set_path(paths, [t, u, u, v], ["L","R","L","R"])

    flag, t, u, v = LRLRp(-x, y, -phi) 
    if flag:
        # println("CCCC6")
        paths = set_path(paths, [-t, -u, -u, -v], ["L","R","L","R"])

    flag, t, u, v = LRLRp(x, -y, -phi) 
    if flag:
        # println("CCCC7")
        paths = set_path(paths, [t, u, u, v], ["R","L","R","L"])

    flag, t, u, v = LRLRp(-x, -y, phi) 
    if flag:
        # println("CCCC8")
        paths = set_path(paths, [-t, -u, -u, -v], ["R","L","R","L"])

    return paths


def LRSR(x, y, phi):

    xi = x + sin(phi)
    eta = y - 1.0 - cos(phi)
    rho, theta = polar(-eta, xi)

    if rho >= 2.0:
        t = theta
        u = 2.0 - rho
        v = mod2pi(t + 0.5*pi - phi)
        if t >= 0.0 and u <= 0.0 and v <=0.0:
            return True, t, u, v
    return False, 0.0, 0.0, 0.0


def LRSL(x, y, phi):
    xi = x - sin(phi)
    eta = y - 1.0 + cos(phi)
    rho, theta = polar(xi, eta)

    if rho >= 2.0:
        r = sqrt(rho*rho - 4.0)
        u = 2.0 - r
        t = mod2pi(theta + atan2(r, -2.0))
        v = mod2pi(phi - 0.5*pi - t)
        if t >= 0.0 and u<= 0.0 and v<=0.0:
            return True, t, u, v
    return False, 0.0, 0.0, 0.0


def CCSC(x, y, phi, paths):

    flag, t, u, v = LRSL(x, y, phi) 
    if flag:
        # println("CCSC1")
        paths = set_path(paths, [t, -0.5*pi, u, v], ["L","R","S","L"])

    flag, t, u, v = LRSL(-x, y, -phi) 
    if flag:
        # println("CCSC2")
        paths = set_path(paths, [-t, 0.5*pi, -u, -v], ["L","R","S","L"])

    flag, t, u, v = LRSL(x, -y, -phi) 
    if flag:
        # println("CCSC3")
        paths = set_path(paths, [t, -0.5*pi, u, v], ["R","L","S","R"])

    flag, t, u, v = LRSL(-x, -y, phi) 
    if flag:
        # println("CCSC4")
        paths = set_path(paths, [-t, 0.5*pi, -u, -v], ["R","L","S","R"])

    flag, t, u, v = LRSR(x, y, phi) 
    if flag:
        # println("CCSC5")
        paths = set_path(paths, [t, -0.5*pi, u, v], ["L","R","S","R"])

    flag, t, u, v = LRSR(-x, y, -phi) 
    if flag:
        # println("CCSC6")
        paths = set_path(paths, [-t, 0.5*pi, -u, -v], ["L","R","S","R"])

    flag, t, u, v = LRSR(x, -y, -phi) 
    if flag:
        # println("CCSC7")
        paths = set_path(paths, [t, -0.5*pi, u, v], ["R","L","S","L"])

    flag, t, u, v = LRSR(-x, -y, phi) 
    if flag:
        # println("CCSC8")
        paths = set_path(paths, [-t, 0.5*pi, -u, -v], ["R","L","S","L"])

    # backwards
    xb = x*cos(phi) + y*sin(phi)
    yb = x*sin(phi) - y*cos(phi)
    flag, t, u, v = LRSL(xb, yb, phi) 
    if flag:
        # println("CCSC9")
        paths = set_path(paths, [v, u, -0.5*pi, t], ["L","S","R","L"])

    flag, t, u, v = LRSL(-xb, yb, -phi) 
    if flag:
        # println("CCSC10")
        paths = set_path(paths, [-v, -u, 0.5*pi, -t], ["L","S","R","L"])

    flag, t, u, v = LRSL(xb, -yb, -phi) 
    if flag:
        # println("CCSC11")
        paths = set_path(paths, [v, u, -0.5*pi, t], ["R","S","L","R"])

    flag, t, u, v = LRSL(-xb, -yb, phi) 
    if flag:
        # println("CCSC12")
        paths = set_path(paths, [-v, -u, 0.5*pi, -t], ["R","S","L","R"])

    flag, t, u, v = LRSR(xb, yb, phi) 
    if flag:
        # println("CCSC13")
        paths = set_path(paths, [v, u, -0.5*pi, t], ["R","S","R","L"])

    flag, t, u, v = LRSR(-xb, yb, -phi) 
    if flag:
        # println("CCSC14")
        paths = set_path(paths, [-v, -u, 0.5*pi, -t], ["R","S","R","L"])

    flag, t, u, v = LRSR(xb, -yb, -phi) 
    if flag:
        # println("CCSC15")
        paths = set_path(paths, [v, u, -0.5*pi, t], ["L","S","L","R"])

    flag, t, u, v = LRSR(-xb, -yb, phi) 
    if flag:
        # println("CCSC16")
        paths = set_path(paths, [-v, -u, 0.5*pi, -t], ["L","S","L","R"])

    return paths


def LRSLR(x, y, phi):
    # formula 8.11 *** TYPO IN PAPER ***
    xi = x + sin(phi)
    eta = y - 1.0 - cos(phi)
    rho, theta = polar(xi, eta)
    if rho >= 2.0:
        u = 4.0 - sqrt(rho*rho - 4.0)
        if u <= 0.0:
            t = mod2pi(atan2((4.0-u)*xi -2.0*eta, -2.0*xi + (u-4.0)*eta))
            v = mod2pi(t - phi)

            if t >= 0.0 and v >=0.0:
                return True, t, u, v
    return False, 0.0, 0.0, 0.0


def CCSCC(x, y, phi, paths):
    flag, t, u, v = LRSLR(x, y, phi) 
    if flag:
        # println("CCSCC1")
        paths = set_path(paths, [t, -0.5*pi, u, -0.5*pi, v], ["L","R","S","L","R"])
    
    flag, t, u, v = LRSLR(-x, y, -phi) 
    if flag:
        # println("CCSCC2")
        paths = set_path(paths, [-t, 0.5*pi, -u, 0.5*pi, -v], ["L","R","S","L","R"])
    
    flag, t, u, v = LRSLR(x, -y, -phi) 
    if flag:
        # println("CCSCC3")
        paths = set_path(paths, [t, -0.5*pi, u, -0.5*pi, v], ["R","L","S","R","L"])
    
    flag, t, u, v = LRSLR(-x, -y, phi) 
    if flag:
        # println("CCSCC4")
        paths = set_path(paths, [-t, 0.5*pi, -u, 0.5*pi, -v], ["R","L","S","R","L"])
    
    return paths


def generate_local_course(L, lengths, mode, maxc, step_size):
    npoint = int(L/step_size) + len(lengths)+3
    # println(npoint, ",", L, ",", step_size, ",", L/step_size)
    
    px = [0.0]*npoint
    py = [0.0]*npoint
    pyaw = [0.0]*npoint
    directions = [0]*npoint
    ind = 1

    if lengths[0] > 0.0:
        directions[0] = 1
    else:
        directions[0] = -1

    if lengths[0] > 0.0:
        d = step_size
    else:
        d = -step_size

    pd = d
    ll = 0.0

    for (m, l, i) in zip(mode, lengths, range(len(mode))):

        if l > 0.0:
            d = step_size
        else:
            d = -step_size

        # set prigin state
        ox, oy, oyaw = px[ind], py[ind], pyaw[ind]

        ind -= 1
        if i >= 1 and (lengths[i-1]*lengths[i])>0:
            pd = - d - ll
        else:
            pd = d - ll

        while abs(pd) <= abs(l):
            ind += 1
            px, py, pyaw, directions = interpolate(ind, pd, m, maxc, ox, oy, oyaw, px, py, pyaw, directions)
            pd += d
        
        ll = l - pd - d # calc remain length
        ind += 1
        px, py, pyaw, directions = interpolate(ind, l, m, maxc, ox, oy, oyaw, px, py, pyaw, directions)
        # remove unused data
    while px[-1] == 0.0:
        px.pop()
        py.pop()
        pyaw.pop()
        directions.pop()
    return px, py, pyaw, directions


def interpolate(ind, l, m, maxc, ox, oy, oyaw, px, py, pyaw, directions):

    if m == "S":
        px[ind] = ox + l / maxc * cos(oyaw)
        py[ind] = oy + l / maxc * sin(oyaw)
        pyaw[ind] = oyaw
    else: # curve
        ldx = sin(l) / maxc
        if m == "L":  # left turn
            ldy = (1.0 - cos(l)) / maxc
        elif m == "R":  # right turn
            ldy = (1.0 - cos(l)) / -maxc
        gdx = cos(-oyaw) * ldx + sin(-oyaw) * ldy
        gdy = -sin(-oyaw) * ldx + cos(-oyaw) * ldy
        px[ind] = ox + gdx
        py[ind] = oy + gdy
    if m == "L": # left turn
        pyaw[ind] = oyaw + l
    elif m == "R":  # right turn
        pyaw[ind] = oyaw - l

    if l > 0.0:
        directions[ind] = 1
    else:
        directions[ind] = -1

    return px, py, pyaw, directions


def generate_path(q0, q1, maxc):
    dx = q1[0] - q0[0]
    dy = q1[1] - q0[1]
    dth = q1[2] - q0[2]
    c = cos(q0[2]); s = sin(q0[2])
    x = (c*dx + s*dy)*maxc
    y = (-s*dx + c*dy)*maxc
    paths = []
    paths = SCS(x, y, dth, paths)
    paths = CSC(x, y, dth, paths)
    paths = CCC(x, y, dth, paths)
    paths = CCCC(x, y, dth, paths)
    paths = CCSC(x, y, dth, paths)
    paths = CCSCC(x, y, dth, paths)

    return paths


def calc_curvature(x,y,yaw, directions):
    c, ds = [], []
    for i in range(1, len(x)-1):
        dxn = x[i]-x[i-1]
        dxp = x[i+1]-x[i]
        dyn = y[i]-y[i-1]
        dyp = y[i+1]-y[i]
        dn =sqrt(dxn**2.0+dyn**2.0)
        dp =sqrt(dxp**2.0+dyp**2.0)
        dx = 1.0/(dn+dp)*(dp/dn*dxn+dn/dp*dxp)
        ddx = 2.0/(dn+dp)*(dxp/dp-dxn/dn)
        dy = 1.0/(dn+dp)*(dp/dn*dyn+dn/dp*dyp)
        ddy = 2.0/(dn+dp)*(dyp/dp-dyn/dn)
        curvature = (ddy*dx-ddx*dy)/(dx**2+dy**2)
        d = (dn+dp)/2.0

        if isnan(curvature):
            curvature = 0.0

        if directions[i] <= 0.0:
            curvature = -curvature

        if len(c) == 0:
            ds.append(d)
            c.append(curvature)

        ds.append(d)
        c.append(curvature)

    ds.append(ds[-1])
    c.append(c[-1])

    return c, ds
