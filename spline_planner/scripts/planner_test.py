import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from spline import QuinticSpline

def plot_spline(spline, ax, **kwargs):
  # type: (QuinticSpline, Axes3D, **dict) -> None
  t = np.linspace(0, 1, 100)
  points = np.array([spline.evaluate(x) for x in t])
  xs = points[:, 0]
  ys = points[:, 1]
  zs = points[:, 2]
  ax.plot(xs, ys, zs=zs, **kwargs)

def propose_geometric_spline_path(p1, p2, p3, t1 = None, t3 = None, a1 = None, a3 = None):
  # type: (np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray) -> list[QuinticSpline]
  direct12 = p2 - p1
  direct23 = p3 - p2

  # default t1 and t3 is the same as the direction of p1 -> p2 and p2 -> p3. 
  # default a1 and a3 is 0
  if t1 is None:
    t1 = 0.5 * direct12
  if t3 is None:
    t3 = 0.5 * direct23
  if a1 is None:
    a1 = np.array([0, 0, 0], dtype=np.float)
  if a3 is None:
    a3 = np.array([0, 0, 0], dtype=np.float)
  
  norm_12 = np.linalg.norm(direct12)
  norm_23 = np.linalg.norm(direct23)
  t2 = direct12 / norm_12

  axis = np.cross(direct12, direct23)
  axis /= np.linalg.norm(axis)
  angle = np.arccos(np.dot(direct12, direct23) / (norm_12 * norm_23)) / 2
  # by Rodrigues' rotation formula
  t2 = np.cos(angle) * t2 + np.sin(angle) * np.cross(axis, t2) + (1 - np.cos(angle)) * np.dot(axis, t2) * axis
  t2 *= (min(norm_12, norm_23))
  # see Sprunk[2008] ch4.1.2 for reference
  alpha = norm_23 / (norm_12 + norm_23)
  beta = norm_12 / (norm_12 + norm_23)
  a2 = alpha * (6 * p1 + 2 * t1 + 4 * t2 - 6 * p2) + beta * (-6 * p2 - 4 * t2 - 2 * t3 + 6 * p3)

  return [QuinticSpline(p1, p2, t1, t2, a1, a2),
          QuinticSpline(p2, p3, t2, t3, a2, a3)]

def sample_path(path, ds = 0.1, samples_per_seg = 300):
  # type: (List[QuinticSpline], float, int) -> List[object]
  num_seg = len(path)
  t = np.linspace(0, 1, samples_per_seg)
  samples = []
  for i in range(num_seg):
    for x in t:
      samples.append((x + i, path[i].evaluate_with_derivatives(x)))
  # perform numeric integral to the the approximated curve length
  last_t, (pt, d1, d2) = samples[0]
  last_d1_norm = np.linalg.norm(d1)
  last_s = 0
  
  result = []
  result.append(((0, 0), (pt, d1, d2)))

  s = 0
  for t, (pt, d1, d2) in samples[1:]:
    print(s)
    d1_norm = np.linalg.norm(d1)
    s += (0.5 * (d1_norm + last_d1_norm) * (t - last_t))
    last_d1_norm = d1_norm
    last_t = t

    if s - last_s >= ds:
      last_s = s
      result.append(((s, t), (pt, d1, d2)))

  return result

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
  s1, s2 = propose_geometric_spline_path(points[0], points[1], points[2])
  fig = plt.figure()
  ax = Axes3D(fig)
  plot_spline(s1, ax, c='r')
  plot_spline(s2, ax, c='g')
  ax.plot(points[:3, 0], points[:3, 1], points[:3, 2], c='b')
  ax.scatter3D(points[0][0], points[0][1], points[0][2], marker='x')
  ax.scatter3D(points[1][0], points[1][1], points[1][2], marker='x')  
  ax.scatter3D(points[2][0], points[2][1], points[2][2], marker='x')
  waypoints = sample_path((s1, s2), 1)

  xs = []
  ys = []
  zs = []
  for (_,_), (pt,_,_) in waypoints:
    xs.append(pt[0])
    ys.append(pt[1])
    zs.append(pt[2])
  ax.scatter3D(xs, ys, zs, marker='o', c='orange')
  plt.show()
