from spline import QuniticSpline
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

if __name__ == '__main__':
  points = np.array([
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
  spline = QuniticSpline(points[0], points[1],
                     np.array([-1, 1, 0]), np.array([-1, 1, 0]),
                     np.array([0, 0, 0]), np.array([0, 0, 0]))
  t = np.linspace(0, 1, 1000)
  pts = np.array([spline.evaluate(x) for x in t])
  fig = plt.figure()
  ax = Axes3D(fig)
  ax.plot(pts[:, 0], pts[:, 1], pts[:, 2])
  ax.scatter3D(points[:2, 0], points[:2, 1], points[:2, 2], marker='x')
  plt.show()


