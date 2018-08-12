from math import pi
import numpy as np
# from plotTraj import plotTraj
from plotAnimation import AnimiTraj
from matplotlib import  animation

from BasicCurve import hybrid_a_star
import matplotlib.pyplot as plt

scenario = 'backwards'

fixTime = 0

TsPF = 0.05
if scenario == 'backwards':
    sampleN = 3
    if fixTime == 1:
        Ts = 0.55/3 * sampleN
    else:
        Ts = 0.6/3 * sampleN
else:
    sampleN = 3
    if fixTime == 1:
        Ts = 0.95 / 3 * sampleN
    else:
        Ts = 0.9 / 3 * sampleN
#wheelbase
L  = 2.7

# step length of Hybrid A*",
motionStep = 0.1


# "nominal" shape of ego/controlled car,  ego object is later rotated around the car center
# center of rear wheel axis is reference point
# size of car is: (x_upper + x_lower) + (y_upper + y_lower)
#	   [x_upper, y_upper, -x_lower, -y_lower ]
ego  = [ 3.7   , 1      ,  1      ,  1       ]


if scenario == "backwards":
    print("Backwards Parking")
elif scenario == "parallel":
    print("Parallel Parking")
else:
    print("ERROR: please specify parking scenario")

if scenario == 'backwards':

    nObPlot = 3
    vObPlot = [4, 4, 4]
    lObPlot = [   [ [-20, 5], [-1.3, 5], [-1.3, -5], [-20, -5], [-20, 5] ]  ,
	 	  [ [1.3, 5], [20, 5], [20, -5], [1.3, -5], [1.3, 5] ] ,
		  [ [-20, 15], [20, 15], [20, 11], [-20, 11], [-20, 15] ]		]

    nOb = 3  # number of obstacles
    vOb = [3, 3, 2]  # number of vertices of each obstacle, vector of dimenion nOb
    vObMPC = [i-1 for i in vOb]
    lOb = [[[-20, 5], [-1.3, 5], [-1.3, -5]],
           [[1.3, -5], [1.3, 5], [20, 5]],
           [[20, 11], [-20, 11]]]  # vetices given in CLOCK-WISE direction

    xF = [0, 1.3, pi/2, 0]

    ox = []
    oy = []
    for i in np.arange(-12, -1.2, 0.1):
        ox.append(i)
        oy.append(5.0)
    for i in np.arange(-2, 5.1, 0.1):
        ox.append(-1.3)
        oy.append(float(i))
    for i in np.arange(-2, 5.1, 0.1):
        ox.append(1.3)
        oy.append(float(i))
    for i in np.arange(1.3, 12.1, 0.1):
        ox.append(i)
        oy.append(5.0)
    for i in np.arange(-12, 12.1, 0.1):
        ox.append(float(i))
        oy.append(11.0)

elif scenario == 'parallel':

    nObPlot = 4  # number of obstacles
    vObPlot = [4, 4, 4, 4]  # number of vertices of each obstacle, vector of dimenion nOb
    #     	[ 	[[obst1_x1;obst1_y1],[obst1_x2;obst1_y2],[obst1_x3;obst1_y4],...,[obst1_x1;obst1_y1]]    , 		[[obst2_x1;obst2_y1],[obst2_x2;obst2_y2],[obst2_x3;obst2_y4],...,[obst2_x1;obst2_y1]]     ,     ...   ]
    lObPlot = [[[-15, 5], [-3, 5], [-3, 0], [-15, 0], [-15, 5]],
               [[3, 5], [15, 5], [15, 0], [3, 0], [3, 5]],
               [[-3, 0], [-3, 2.5], [3, 2.5], [3, 0], [-3, 0]],
               [[-15, 15], [15, 15], [15, 11], [-15, 11], [-15, 15]]]

    # define obstacles for optimization problem
    nOb =  4 	# number of obstacles
    vOb = [3, 3, 2, 2]	# number of vertices of each obstacle, vector of dimenion nOb
    vObMPC = [i - 1 for i in vOb]
    lOb = [   [ [-20, 5], [-3.0, 5], [-3.0, 0]],
              [ [3.0, 0] , [3.0, 5] , [20, 5] ],
              [ [-3, 2.5], [ 3, 2.5]],
              [ [ 20, 11 ], [-20, 11]]]  # vetices given in CLOCK-WISE direction

    # [ [ 3;11 ], [-3;11]]

    # final state
    xF = [-L/2, 4, 0, 0]

    x0X_range = np.arange(-9, 10, 1)
    x0Y_range = np.arange(6.5, 9.5, 1.5)

    ox = []
    oy = []

    for i in np.arange(-12, -2.9, 0.1):
        ox.append(i)
        oy.append(5.0)

    for i in np.arange(-2, 5.1, 0.1):
        ox.append(-3.0)
        oy.append(float(i))

    for i in np.arange(-3, 3.1, 0.1):
        ox.append(float(i))
        oy.append(2.5)

    for i in np.arange(-2, 5.1, 0.1):
        ox.append(3.0)
        oy.append(float(i))

    for i in np.arange(3, 12.1, 0.1):
        ox.append(i)
        oy.append(5)

    for i in np.arange(-12, 12.1, 0.1):
        ox.append(float(i))
        oy.append(11.5)

XYbounds = [-15, 15, 1, 10]
x0 = [-6, 9.5, 0.0, 0.0]

rx, ry, ryaw = hybrid_a_star.calc_hybrid_astar_path(x0[0], x0[1], x0[2], xF[0], xF[1], xF[2], ox, oy,
                                                hybrid_a_star.XY_GRID_RESOLUTION,
                                                hybrid_a_star.YAW_GRID_RESOLUTION, hybrid_a_star.OB_MAP_RESOLUTION)
xp = np.mat([rx, ry, ryaw])
xp = xp.T

# hybrid_a_star.savedata(xp, 'xp.pkl')
# plt.plot(ox, oy, '.k', label = 'obstacles')
# if rx != None:
#     plt.plot(rx, ry, '-r', label = "Hybrid A* path")
#
# plt.legend()
# plt.grid(True)
# plt.axis('equal')
# plt.show()

# xp = hybrid_a_star.loaddata('xp.pkl')
N = xp.shape[0]

# plotTraj(xp, N, ego,L,nObPlot,vObPlot,lObPlot,"Signed Distance Approach (Min. Penetration)",1)
AnimiTraj(xp, N, ego,L,nObPlot,vObPlot,lObPlot,"Parallel Parking",1)

