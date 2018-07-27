import numpy as np
import kdtree, Distance as ds
ox, oy = [], []

for i in range(11):
    ox.append(float(i))
    oy.append(0.0)

for i in range(11):
    ox.append(10.0)
    oy.append(float(i))

for i in range(11):
    ox.append(float(i))
    oy.append(10.0)

for i in range(11):
    ox.append(0.0)
    oy.append(float(i))

for i in range(8):
    ox.append(4.0)
    oy.append(float(i))

for i in range(5):
    ox.append(6.0)
    oy.append(10.0 - float(i))

minx = round(min(ox))
miny = round(min(oy))
maxx = round(max(ox))
maxy = round(max(oy))

xwidth = round(maxx - minx)
ywidth = round(maxy - miny)
obmap = np.full((xwidth+1, ywidth+1), False)
data = []
for i in zip(ox,oy):
    data.append({1: i[0], 2: i[1]})
    root = kdtree.create(data, dimensions=2)

x, y = 2, 2
f = ds.EuclideanDistance
ans = root.search_knn(point={1: x, 2: y}, k=1, dist=f)