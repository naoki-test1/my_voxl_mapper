import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

with open("trajectory.csv") as file:
    lines = file.readlines()

n_segments = int(lines[0].rstrip('\n'))
lines = lines[1:]

data = []

class Segment:
    def __init__(self, duration_s, cx, cy, cz):
        self.duration_s = duration_s
        self.cx = np.poly1d(cx[::-1])
        self.cy = np.poly1d(cy[::-1])
        self.cz = np.poly1d(cz[::-1])

for i in range(n_segments):
    duration_s = float(lines[0].rstrip('\n'))
    cx = [float(i) for i in lines[1].rstrip(', \n').split(", ")]
    cy = [float(i) for i in lines[2].rstrip(', \n').split(", ")]
    cz = [float(i) for i in lines[3].rstrip(', \n').split(", ")]

    data.append(Segment(duration_s, cx, cy, cz))

    if i != n_segments - 1:
        lines = lines[4:]

fig = plt.figure()
ax = plt.axes(projection="3d")

for s in data:
    ts = np.linspace(0, s.duration_s, num=int(s.duration_s / 0.05))

    xs = s.cx(ts)
    ys = s.cy(ts)
    zs = s.cz(ts)

    ax.plot3D(xs, ys, zs)
    ax.scatter(xs, ys, zs)

plt.show()

fig = plt.figure()
ax = plt.axes(projection="3d")

# Velocity plots
for s in data:
    ts = np.linspace(0, s.duration_s, num=int(s.duration_s / 0.05))

    xs = s.cx.deriv()(ts)
    ys = s.cy.deriv()(ts)
    zs = s.cz.deriv()(ts)

    ax.plot3D(xs, ys, zs)
    ax.scatter(xs, ys, zs)

plt.show()

fig = plt.figure()
ax = plt.axes(projection="3d")

# Acc plots
for s in data:
    ts = np.linspace(0, s.duration_s, num=int(s.duration_s / 0.05))

    xs = s.cx.deriv(2)(ts)
    ys = s.cy.deriv(2)(ts)
    zs = s.cz.deriv(2)(ts)

    ax.plot3D(xs, ys, zs)
    ax.scatter(xs, ys, zs)

plt.show()