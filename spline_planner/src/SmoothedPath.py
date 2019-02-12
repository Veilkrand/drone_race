"""
This class is responsible for doing the dirty-job: fit path from given waypoints and provides
querying methods for the fit path
"""

import math
import operator
from scipy.interpolate import splprep, spalde
import numpy as np

class SmoothedPath(object):

  @staticmethod
  def _distance(point1, point2):
    return math.sqrt( (point2.x-point1.x)**2 + (point2.y-point1.y)**2 + (point2.z-point1.z)**2 )

  def fit(self, points):
    """
    Fit a path with the given points
    """
    # similar to points_to_spline in ros_geometry.py, this method fit a cubic BSpline with the given points
    #
    # the difference is this method use splrep instead of interp1d with kind='cubic'
    #
    # splprep/spalde from FITPACK library offers derivatives querying methods which is useful for later processing
    #
    # see: https://docs.scipy.org/doc/scipy/reference/tutorial/interpolate.html#spline-interpolation
    distance_so_far = 0.0
    last_point = points[0]
    distances = []
    for point in points:
        distances.append(distance_so_far + SmoothedPath._distance(last_point, point))
        distance_so_far = distances[-1]
    order = 3
    self.tck = splprep([[p.x for p in points],
                        [p.y for p in points],
                        [p.z for p in points]],
                        u=distances,k=order)[0]
    self.chord_length_total = distances[-1]

  def visit_at_interval(self, ds, callback_fn, lookahead_max=-1):
    """
    Query the points on the path at the given interval, specified by parameter ds.

    When a point on the path is visited, `callback_fn` will be called with:
    * the visited point
    * first order derivative at the visited point, w.r.t. to distance along the path (a.k.a. arc length)
    * second order derivative at the visited point, w.r.t. to distance along the path (a.k.a. arc length)
    * the approximated distance from the beginning of the path

    At which point it should visit and how long the distance is are approximated vbia numerical integration method.
    """
    # dt defines the interval of sampled points along the knot vector (chord length). it should not be too large since
    # it is also used in numerical integration
    dt = 0.1
    last_s = 0
    current_s = 0
    current_t = 0
    last_deriv1_norm = 0
    while True:
      # query point and derivatives from the fit spline.
      # zero-order derivative is the point on the spline
      alde = np.array(spalde(current_t, self.tck))
      pt, deriv1, deriv2 = alde[:, 0], alde[:, 1], alde[:, 2]
      
      # by definition (see https://en.wikipedia.org/wiki/Arc_length#Definition_for_a_smooth_curve),
      # arc length at some chord length t is: s(t) = \int_0^t ||s'(t)|| dt, 
      # where s'(t) is the 1st order derivative at t
      #
      # by using the trapeziod method to do numeric integation, 
      # if we have s(t_p) be some value and we already know ||s'(t_p)||,
      # then for t_c = t_p + dt:
      # s(t_c) = s(t_p) + 0.5 * (||s'(t_p)|| + ||s'(t_c)||) * dt
      deriv1_norm = np.linalg.norm(deriv1)
      if current_t == 0:
        last_deriv1_norm = deriv1_norm
      else:
        current_s += 0.5 * (deriv1_norm + last_deriv1_norm) * dt
        last_deriv1_norm = deriv1_norm

      current_t += dt

      if current_s - last_s >= ds or current_s == 0 or current_t > self.chord_length_total:
        last_s = current_s
        if callback_fn is not None:
          callback_fn(pt, deriv1, deriv2, current_s)

      if current_t > self.chord_length_total:
        break

      if lookahead_max > 0 and current_s > lookahead_max:
        break

