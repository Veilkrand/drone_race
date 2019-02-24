import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from spline_planner.eigen_spline import CubicSpline3DWrapper
import math

def fit_and_plot(ax, points, color, derivs=None, indices=None):
  if derivs is not None:
    spline = CubicSpline3DWrapper(points, derivs, indices)
  else:
    spline = CubicSpline3DWrapper(points)
  points = spline.sampleAndCollect(0.1, 200)
  line_x = []
  line_y = []
  line_z = []
  for (t, s, (pt, d1, d2)) in points:
    line_x.append(pt[0])
    line_y.append(pt[1])
    line_z.append(pt[2])
  ax.plot(line_x, line_y, zs=line_z, color=color)
  return points

def chord_length(pts):
  p = np.array(pts)
  p1 = p[:-1]
  p2 = p[1:]
  diff = p1 - p2
  norm = np.linalg.norm(diff, axis=1)
  return np.sum(norm)

def scale(v, s):
  return list(k * s for k in v)

if __name__ == '__main__':
    points = [
        (0.977078310701, -26.9431577766, 2.2416099253),
        (0.977078310701, -20.9431577766, 2.2416099253),        
        (2.32982075, 29.36797, 2.54649985),
        (2.32982075, 28.36797, 2.54649985),
        (2.199832, 9.501728, 1.99375),
        (2.199832, 8.501728, 1.99375),
        (-6.4086695, -11.63678, 3.229941),
        (-6.4086695, -12.63678, 3.229941),
        (-0.509000539307, -33.912973681, 2.1031115),
        (0.490999459307, -33.913026319, 2.1031115),
    ]
    fig1 = plt.figure()
    ax = Axes3D(fig1)
    fit_and_plot(ax, points, 'green')
    pts2 = fit_and_plot(ax, points[:4], 'red')
    norm2 = chord_length(points[:4])

    cont_pt_idx = 20
    t1, s1, (pt1, d11, d12) = pts2[cont_pt_idx]
    """ print(d11)
    print(d12) """
    p_new = [pt1] + points[2:4]
    norm3 = chord_length(p_new)
    d = np.array([d11])
    factor = norm3 / norm2
    delta = 1
    num_try = 0
    best = factor
    best_score = 0
    while num_try < 20:
      rd = factor * d
      pts3 = fit_and_plot(ax, p_new, 'blue', rd.tolist(), [0])
      a = np.linalg.norm(np.array(pts3[0][2][1])) / np.linalg.norm(d11)
      b = np.linalg.norm(np.array(pts3[0][2][2])) / np.linalg.norm(d12)
      new_score = a ** 2 / b
      print(factor, a, b, new_score)
      if abs(new_score - 1) < abs(best_score - 1):
        best_score = new_score
        best = factor
      if new_score > 1:
        factor /= (1 + abs(new_score - 1))
      else:
        factor *= (1 + abs(new_score - 1))
      num_try += 1
    print(best, best_score)
    fit_and_plot(ax, p_new, 'orange', (best * d).tolist(), [0])
    fap = np.array(p_new)
    """ print(fap[0, :]) """
    ax.scatter3D(fap[:, 0], fap[:, 1], fap[:, 2], marker='o')
    
    ax.set_xlim(-30, 30)
    ax.set_ylim(-30, 30)
    ax.set_zlim(0, 5)
    plt.show()
