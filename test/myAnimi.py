from matplotlib import pyplot as plt
from matplotlib import  animation
import numpy as np

fig = plt.figure(figsize=(10,10))
ax = plt.axes(xlim=(0, 2*np.pi), ylim=(-1,1))
xdata, ydata = [], []
lines = []
ln = ax.plot([], [], 'ro')[0]
lines.append(ln)
ln = ax.plot([], [], ':k')[0]
lines.append(ln)
x = np.linspace(0, 2*np.pi, 128)
y = np.sin(x)
xp = np.mat([x, y])
xp = xp.T
def init():
    plt.plot(1,0,'ro')
    ax.set_xlim(0, 2*np.pi)
    ax.set_ylim(-1, 1)
    return lines

def update(i):

    lines[0].set_data(xp[:i, 0], xp[:i, 1])
    lines[1].set_data([xp[i,0], xp[i,0]], [-1, 1])
    return lines

ani = animation.FuncAnimation(fig, update, frames=range(128),
                    init_func=init, blit=True)
plt.show()