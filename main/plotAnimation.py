import numpy as np
import matplotlib.pyplot as plt
import time
from matplotlib import animation

def plotAni(xp, N, ego, L, nOb, vOb, lOb, disp_title, plotNumb):

    W_ev = ego[1] + ego[3]
    L_ev = ego[0] + ego[2]

    # final position no input

    w = W_ev / 2
    offset = L_ev / 2 - ego[2]

    # initial state
    x0_s = xp[0, :]
    Rot0 = np.mat([[np.cos(x0_s[0,2]), -np.sin(x0_s[0,2])],
                   [np.sin(x0_s[0,2]), np.cos(x0_s[0,2])]])
    x0 = np.mat([[x0_s[0,0]], [x0_s[0,1]]])
    centerCar0 = x0 + Rot0 * np.mat([[offset],[0]])

    # end state
    xF_s = xp[-1, :]
    RotF = np.mat([[np.cos(xF_s[0,2]), -np.sin(xF_s[0,2])], [np.sin(xF_s[0,2]), np.cos(xF_s[0,2])]])
    xF = np.mat([[xF_s[0,0]], [xF_s[0,1]]])

    centerCarF = xF + RotF * np.mat([[offset],[0]])


    fig = plt.figure(plotNumb)

    plt.title(disp_title)
    # plt.hold(True)

    for j in range(nOb):
        for k in range(0, vOb[j]):
            plt.plot([lOb[j][k][0], lOb[j][k + 1][0]], [lOb[j][k][1], lOb[j][k + 1][1]], "k")

    plt.plot(x0[0, 0], x0[1, 0], "ob")
    carBox(centerCar0, x0_s[0, 2], W_ev / 2, L_ev / 2)
    carBox(x0 + (Rot0 * np.mat([[L], [w - 0.15]])), x0_s[0, 2], 0.15, 0.3)
    carBox(x0 + (Rot0 * np.mat([[L], [-w + 0.15]])), x0_s[0, 2], 0.15, 0.3)
    carBox(x0 + (Rot0 * np.mat([[0], [w - 0.15]])), x0_s[0, 2], 0.15, 0.3)
    carBox(x0 + (Rot0 * np.mat([[0], [-w + 0.15]])), x0_s[0, 2], 0.15, 0.3)

    # plot end position
    carBox_dashed(centerCarF, xF_s[0, 2], W_ev / 2, L_ev / 2)
    carBox_dashed(xF + (RotF * np.mat([[L], [w - 0.15]])), xF_s[0, 2], 0.15, 0.3)
    carBox_dashed(xF + (RotF * np.mat([[L], [-w + 0.15]])), xF_s[0, 2], 0.15, 0.3)
    carBox_dashed(xF + (RotF * np.mat([[0], [w - 0.15]])), xF_s[0, 2], 0.15, 0.3)
    carBox_dashed(xF + (RotF * np.mat([[0], [-w + 0.15]])), xF_s[0, 2], 0.15, 0.3)






def carBox(x0, phi, w, l):

    car1 = x0[0:2] + np.mat([[np.cos(phi) * l], [np.sin(phi) * l]]) + np.mat([[np.sin(phi) * w], [-np.cos(phi) * w]])
    car2 = x0[0:2] + np.mat([[np.cos(phi) * l], [np.sin(phi) * l]]) - np.mat([[np.sin(phi) * w], [-np.cos(phi) * w]])
    car3 = x0[0:2] - np.mat([[np.cos(phi) * l], [np.sin(phi) * l]]) + np.mat([[np.sin(phi) * w], [-np.cos(phi) * w]])
    car4 = x0[0:2] - np.mat([[np.cos(phi) * l], [np.sin(phi) * l]]) - np.mat([[np.sin(phi) * w], [-np.cos(phi) * w]])
    plt.plot([car1[0,0],car2[0,0],car4[0,0],car3[0,0],car1[0,0]],[car1[1,0],car2[1,0],car4[1,0],car3[1,0],car1[1,0]],"k")


def carBox_dashed(x0, phi, w, l):
    car1 = x0[0:2] + np.mat([[np.cos(phi) * l], [np.sin(phi) * l]]) + np.mat([[np.sin(phi) * w], [-np.cos(phi) * w]])
    car2 = x0[0:2] + np.mat([[np.cos(phi) * l], [np.sin(phi) * l]]) - np.mat([[np.sin(phi) * w], [-np.cos(phi) * w]])
    car3 = x0[0:2] - np.mat([[np.cos(phi) * l], [np.sin(phi) * l]]) + np.mat([[np.sin(phi) * w], [-np.cos(phi) * w]])
    car4 = x0[0:2] - np.mat([[np.cos(phi) * l], [np.sin(phi) * l]]) - np.mat([[np.sin(phi) * w], [-np.cos(phi) * w]])
    plt.plot([car1[0,0], car2[0,0], car4[0,0], car3[0,0], car1[0,0]], [car1[1,0], car2[1,0], car4[1,0], car3[1,0], car1[1,0]], ":k")


def animation(i):
    global xp, N, xF, offset, W_ev, L_ev, L
    plt.plot(xp[:i, 0], xp[:i, 1], 'b')
    Rot = np.mat([[np.cos(xp[i, 2]), -np.sin(xp[i, 2])],
               [np.sin(xp[i, 2]), np.cos(xp[i, 2])]])

    x_cur = np.mat([[xp[i,0]],[xp[i,1]]])

    centerCar = x_cur + Rot*np.mat([[offset],[0]])

    carBox(centerCar, xp[i, 2], W_ev/2, L_ev/2)
    carBox(x_cur + (Rot * np.mat([[L],[w-0.15]])), xp[i+1, 2], 0.15, 0.3)
    carBox(x_cur + (Rot * np.mat([[L],[-w+0.15]])), xp[i+1, 2], 0.15, 0.3)
    carBox(x_cur + (Rot * np.mat([[0], [w-0.15]])), xp[i, 2], 0.15, 0.3)
    carBox(x_cur + (Rot * np.mat([[0], [-w+0.15]])), xp[i, 2], 0.15, 0.3)

    if i == N:
        plt.plot(xF[1], xF[2], "ob")

    plt.axis("equal")
    plt.draw()
    time.sleep(0.05)
    plt.cla()
    plt.close()