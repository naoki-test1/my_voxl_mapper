from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from scipy import spatial

pc = np.genfromtxt('pc.csv', delimiter=", ")
pc = pc.reshape(-1, 3)

path = np.genfromtxt('path.csv', delimiter=", ")

# Visualization
fig = plt.figure()
ax = plt.axes(projection="3d")

min_mat = spatial.distance.cdist(pc, path, "euclidean")
min_dist = np.min(min_mat)
min_idx = np.unravel_index(np.argmin(min_mat, axis=None), min_mat.shape)

print("Closest distance between pointcloud and trajectory = ", min_dist)
print("Closest distance occurs between Pc point = ", min_idx[0], " and Path = ", min_idx[1])
closest = np.array([pc[min_idx[0]], path[min_idx[1]]])

ax.scatter(pc[:, 0], pc[:, 1], pc[:, 2])
ax.scatter(path[:, 0], path[:, 1], path[:, 2])
ax.plot(closest[:, 0], closest[:, 1], closest[:, 2], 'r')
plt.show()