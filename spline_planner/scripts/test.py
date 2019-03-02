from spline import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def plot_spline(spline, ax, **kwargs):
  # type: (QuinticSpline, Axes3D, **dict) -> None
  t = np.linspace(0, 1, 100)
  points = np.array([spline.evaluate(x) for x in t])
  xs = points[:, 0]
  ys = points[:, 1]
  zs = points[:, 2]
  ax.plot(xs, ys, zs=zs, **kwargs)

if __name__ == '__main__':
  points = np.array([
    [18.0, -23.0, 5.3],
    [18.15111,  3.631447,  6.3064975],
    [16.749825, 39.56997,  6.7624995],
    [2.32982075, 27.86797,  2.54649985],
    [2.199832, 9.001728, 1.99375],
    [-7.308671, -12.13678,   3.229941],
    [-7.61018, -28.472035,   2.6425],
    [-9.0005400e-03, -3.3913000e+01,  2.1031115e+00],
    [5.8678205, -28.440035,   2.56749995],
    [7.249821, -11.79203,   2.54649985],
    [-9.32867,  7.773174,  2.019941],
    [-9.148669, 30.62316,  2.897941]
  ])
  path = propose_geometric_spline_path(points)
  fig = plt.figure()
  ax = Axes3D(fig)
  for p in path:
    plot_spline(p, ax, c='r')
  ax.plot(points[:, 0], points[:, 1], points[:, 2], c='b')
  ax.scatter3D(points[0][0], points[0][1], points[0][2], marker='x')
  ax.scatter3D(points[1][0], points[1][1], points[1][2], marker='x')  
  ax.scatter3D(points[2][0], points[2][1], points[2][2], marker='x')
  waypoints = sample_path(path, 1)

  xs = []
  ys = []
  zs = []
  for (_,_), (pt,_,_) in waypoints:
    xs.append(pt[0])
    ys.append(pt[1])
    zs.append(pt[2])
  ax.scatter3D(xs, ys, zs, marker='o', c='orange')
  plt.show()



