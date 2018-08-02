import numpy as np
import matplotlib.pyplot as plt
import time
from matplotlib import animation

# plt.ion()
def AnimiTraj(xp, N, ego, L, nOb, vOb, lOb, disp_title, plotNumb):

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

    fig = plt.figure(figsize=(44*10/44, 34*10/44))
    ax = plt.axes(xlim=(-22, 22), ylim=(-12, 22))

    lines = []

    for i in range(5):
        line = ax.plot([], [], 'k')[0]
        lines.append(line)

    for i in range(5):
        line = ax.plot([], [], ':k')[0]
        lines.append(line)

    for i in range(5):
        line = ax.plot([], [], 'k')[0]
        lines.append(line)

    line = plt.plot([], [], 'b')[0]
    lines.append(line)

    def init():
        ax.set_xlim(-22, 22)
        ax.set_ylim(-12, 22)
        for j in range(nOb):
            for k in range(0, vOb[j]):
                plt.plot([lOb[j][k][0],lOb[j][k+1][0]] , [lOb[j][k][1],lOb[j][k+1][1]] ,"k")

        plt.plot(x0[0, 0], x0[1, 0], "ob")
        plt.plot(xF[0, 0], xF[1, 0], "ob")


        #
        for i in range(16):
            lines[i].set_data([], [])

        line.set_data([], [])
        plt.title(disp_title)
        # plt.axis("equal")

        return lines


    def update(i):

        Rot = np.mat([[np.cos(xp[i, 2]), -np.sin(xp[i, 2])],
                      [np.sin(xp[i, 2]), np.cos(xp[i, 2])]])

        x_cur = np.mat([[xp[i, 0]], [xp[i, 1]]])

        centerCar = x_cur + Rot * np.mat([[offset], [0]])

        xa = []
        xa.append(list(carBox(centerCar0, x0_s[0, 2], W_ev / 2, L_ev / 2)))
        xa.append(list(carBox(x0 + (Rot0 * np.mat([[L], [w - 0.15]])), x0_s[0, 2], 0.15, 0.3)))
        xa.append(list(carBox(x0 + (Rot0 * np.mat([[L], [-w + 0.15]])), x0_s[0, 2], 0.15, 0.3)))
        xa.append(list(carBox(x0 + (Rot0 * np.mat([[0], [w - 0.15]])), x0_s[0, 2], 0.15, 0.3)))
        xa.append(list(carBox(x0 + (Rot0 * np.mat([[0], [-w + 0.15]])), x0_s[0, 2], 0.15, 0.3)))

        # plot end position
        xa.append(list(carBox(centerCarF, xF_s[0, 2], W_ev / 2, L_ev / 2)))
        xa.append(list(carBox(xF + (RotF * np.mat([[L], [w - 0.15]])), xF_s[0, 2], 0.15, 0.3)))
        xa.append(list(carBox(xF + (RotF * np.mat([[L], [-w + 0.15]])), xF_s[0, 2], 0.15, 0.3)))
        xa.append(list(carBox(xF + (RotF * np.mat([[0], [w - 0.15]])), xF_s[0, 2], 0.15, 0.3)))
        xa.append(list(carBox(xF + (RotF * np.mat([[0], [-w + 0.15]])), xF_s[0, 2], 0.15, 0.3)))

        xa.append(list(carBox(centerCar, xp[i, 2], W_ev / 2, L_ev / 2)))
        xa.append(list(carBox(x_cur + (Rot * np.mat([[L], [w - 0.15]])), xp[i + 1, 2], 0.15, 0.3)))
        xa.append(list(carBox(x_cur + (Rot * np.mat([[L], [-w + 0.15]])), xp[i + 1, 2], 0.15, 0.3)))
        xa.append(list(carBox(x_cur + (Rot * np.mat([[0], [w - 0.15]])), xp[i, 2], 0.15, 0.3)))
        xa.append(list(carBox(x_cur + (Rot * np.mat([[0], [-w + 0.15]])), xp[i, 2], 0.15, 0.3)))

        # xa = np.mat(xa)
        for j in range(15):
            lines[j].set_data(xa[j][0], xa[j][1])
        # print(xp[0, 0], xp[0, 1])
        lines[-1].set_data(xp[:i, 0], xp[:i, 1])

        return lines

    # print(xp[:5, :])
    anim = animation.FuncAnimation(fig, update,  frames= range(N-1), init_func=init, interval = 50, repeat= False, blit = True)
    anim.save('parallel.gif', writer='imagemagick')
    plt.show()




def carBox(x0, phi, w, l):

    car1 = x0[0:2] + np.mat([[np.cos(phi) * l], [np.sin(phi) * l]]) + np.mat([[np.sin(phi) * w], [-np.cos(phi) * w]])
    car2 = x0[0:2] + np.mat([[np.cos(phi) * l], [np.sin(phi) * l]]) - np.mat([[np.sin(phi) * w], [-np.cos(phi) * w]])
    car3 = x0[0:2] - np.mat([[np.cos(phi) * l], [np.sin(phi) * l]]) + np.mat([[np.sin(phi) * w], [-np.cos(phi) * w]])
    car4 = x0[0:2] - np.mat([[np.cos(phi) * l], [np.sin(phi) * l]]) - np.mat([[np.sin(phi) * w], [-np.cos(phi) * w]])
    x = [car1[0,0],car2[0,0],car4[0,0],car3[0,0],car1[0,0]]
    y = [car1[1,0],car2[1,0],car4[1,0],car3[1,0],car1[1,0]]
    return x, y
